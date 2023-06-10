// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#include "pvr_context.h"
#include "pvr_device.h"
#include "pvr_drv.h"
#include "pvr_gem.h"
#include "pvr_hwrt.h"
#include "pvr_job.h"
#include "pvr_power.h"
#include "pvr_rogue_fwif.h"
#include "pvr_rogue_fwif_client.h"
#include "pvr_stream.h"
#include "pvr_stream_defs.h"

#include <drm/drm_gem.h>
#include <drm/drm_syncobj.h>
#include <linux/dma-fence.h>
#include <linux/dma-fence-array.h>
#include <linux/dma-fence-unwrap.h>
#include <linux/dma-resv.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/ww_mutex.h>
#include <linux/xarray.h>
#include <uapi/drm/pvr_drm.h>

static void pvr_job_release(struct kref *kref)
{
	struct pvr_job *job = container_of(kref, struct pvr_job, ref_count);

	xa_erase(&job->pvr_dev->job_ids, job->id);

	pvr_hwrt_data_put(job->hwrt);
	pvr_context_put(job->ctx);

	pvr_queue_job_cleanup(job);
	pvr_job_release_pm_ref(job);

	kfree(job->cmd);
	kfree(job);
}

/**
 * pvr_job_evict_signaled_native_deps() - Evict signaled native deps
 * @job: Job to operate on.
 *
 * Returns: Number of unsignalled native deps remaining.
 */
unsigned long pvr_job_evict_signaled_native_deps(struct pvr_job *job)
{
	unsigned long remaining_count = 0;
	struct dma_fence *fence = NULL;
	unsigned long index;

	xa_for_each(&job->base.dependencies, index, fence) {
		if (dma_fence_is_signaled(fence)) {
			xa_erase(&job->base.dependencies, index);
			dma_fence_put(fence);
		} else {
			remaining_count++;
		}
	}

	return remaining_count;
}

/**
 * pvr_job_put() - Release reference on job
 * @job: Target job.
 */
void
pvr_job_put(struct pvr_job *job)
{
	if (job)
		kref_put(&job->ref_count, pvr_job_release);
}

static int
pvr_check_sync_op(const struct drm_pvr_sync_op *sync_op)
{
	u8 handle_type;

	if (sync_op->flags & ~DRM_PVR_SYNC_OP_FLAGS_MASK)
		return -EINVAL;

	handle_type = sync_op->flags & DRM_PVR_SYNC_OP_FLAG_HANDLE_TYPE_MASK;
	if (handle_type != DRM_PVR_SYNC_OP_FLAG_HANDLE_TYPE_SYNCOBJ &&
	    handle_type != DRM_PVR_SYNC_OP_FLAG_HANDLE_TYPE_TIMELINE_SYNCOBJ)
		return -EINVAL;

	if (handle_type == DRM_PVR_SYNC_OP_FLAG_HANDLE_TYPE_SYNCOBJ &&
	    sync_op->value != 0)
		return -EINVAL;

	return 0;
}

/**
 * struct pvr_sync_signal - Object encoding a syncobj signal operation
 *
 * The job submission logic collects all signal operations in an array of
 * pvr_sync_signal objects. This array also serves as a cache to get the
 * latest dma_fence when multiple jobs are submitted at once, and one job
 * signals a syncobj point that's later waited on by a subsequent job.
 */
struct pvr_sync_signal {
	/** @handle: Handle of the syncobj to signal. */
	u32 handle;

	/**
	 * @point: Point to signal in the syncobj.
	 *
	 * Only relevant for timeline syncobjs.
	 */
	u64 point;

	/** @syncobj: Syncobj retrieved from the handle. */
	struct drm_syncobj *syncobj;

	/**
	 * @chain: Chain object used to link the new fence with the
	 *	   existing timeline syncobj.
	 *
	 * Should be zero when manipulating a regular syncobj.
	 */
	struct dma_fence_chain *chain;

	/**
	 * @fence: New fence object to attach to the syncobj.
	 *
	 * This pointer starts with the current fence bound to
	 * the <handle,point> pair.
	 */
	struct dma_fence *fence;
};

static void
pvr_sync_signal_free(struct pvr_sync_signal *sig_sync)
{
	if (!sig_sync)
		return;

	drm_syncobj_put(sig_sync->syncobj);
	dma_fence_chain_free(sig_sync->chain);
	dma_fence_put(sig_sync->fence);
	kfree(sig_sync);
}

static void
pvr_sync_signal_array_cleanup(struct xarray *array)
{
	struct pvr_sync_signal *sig_sync;
	unsigned long i;

	xa_for_each(array, i, sig_sync)
		pvr_sync_signal_free(sig_sync);

	xa_destroy(array);
}

static struct pvr_sync_signal *
pvr_sync_signal_array_add(struct xarray *array, struct drm_file *file, u32 handle, u64 point)
{
	struct pvr_sync_signal *sig_sync;
	int err;
	u32 id;

	sig_sync = kzalloc(sizeof(*sig_sync), GFP_KERNEL);
	if (!sig_sync)
		return ERR_PTR(-ENOMEM);

	sig_sync->handle = handle;
	sig_sync->point = point;

	if (point > 0) {
		sig_sync->chain = dma_fence_chain_alloc();
		if (!sig_sync->chain) {
			err = -ENOMEM;
			goto err_free_sig_sync;
		}
	}

	sig_sync->syncobj = drm_syncobj_find(file, handle);
	if (!sig_sync->syncobj) {
		err = -EINVAL;
		goto err_free_sig_sync;
	}

	/* Retrieve the current fence attached to that point. It's
	 * perfectly fine to get a NULL fence here, it just means there's
	 * no fence attached to that point yet.
	 */
	drm_syncobj_find_fence(file, handle, point, 0, &sig_sync->fence);

	err = xa_alloc(array, &id, sig_sync, xa_limit_32b, GFP_KERNEL);
	if (err)
		goto err_free_sig_sync;

	return sig_sync;

err_free_sig_sync:
	pvr_sync_signal_free(sig_sync);
	return ERR_PTR(err);
}

static struct pvr_sync_signal *
pvr_sync_signal_array_search(struct xarray *array, u32 handle, u64 point)
{
	struct pvr_sync_signal *sig_sync;
	unsigned long i;

	xa_for_each(array, i, sig_sync) {
		if (handle == sig_sync->handle && point == sig_sync->point)
			return sig_sync;
	}

	return NULL;
}

static struct pvr_sync_signal *
pvr_sync_signal_array_get(struct xarray *array, struct drm_file *file, u32 handle, u64 point)
{
	struct pvr_sync_signal *sig_sync;

	sig_sync = pvr_sync_signal_array_search(array, handle, point);
	if (sig_sync)
		return sig_sync;

	return pvr_sync_signal_array_add(array, file, handle, point);
}

static int
pvr_sync_signal_array_collect_ops(struct xarray *array,
				  struct drm_file *file,
				  u32 sync_op_count,
				  struct drm_pvr_sync_op *sync_ops)
{
	for (u32 i = 0; i < sync_op_count; i++) {
		struct pvr_sync_signal *sig_sync;
		int ret;

		if (!(sync_ops[i].flags & DRM_PVR_SYNC_OP_FLAG_SIGNAL))
			continue;

		ret = pvr_check_sync_op(&sync_ops[i]);
		if (ret)
			return ret;

		sig_sync = pvr_sync_signal_array_get(array, file,
						     sync_ops[i].handle,
						     sync_ops[i].value);
		if (IS_ERR(sig_sync))
			return PTR_ERR(sig_sync);
	}

	return 0;
}

static int
pvr_sync_signal_array_update_fences(struct xarray *array,
				    u32 sync_op_count,
				    const struct drm_pvr_sync_op *sync_ops,
				    struct dma_fence *done_fence)
{
	for (u32 i = 0; i < sync_op_count; i++) {
		struct dma_fence *old_fence;
		struct pvr_sync_signal *sig_sync;

		if (!(sync_ops[i].flags & DRM_PVR_SYNC_OP_FLAG_SIGNAL))
			continue;

		sig_sync = pvr_sync_signal_array_search(array, sync_ops[i].handle,
							sync_ops[i].value);
		if (WARN_ON(!sig_sync))
			return -EINVAL;

		old_fence = sig_sync->fence;
		sig_sync->fence = dma_fence_get(done_fence);
		dma_fence_put(old_fence);

		if (WARN_ON(!sig_sync->fence))
			return -EINVAL;
	}

	return 0;
}

static void
pvr_sync_signal_array_push_fences(struct xarray *array)
{
	struct pvr_sync_signal *sig_sync;
	unsigned long i;

	xa_for_each(array, i, sig_sync) {
		if (sig_sync->chain) {
			drm_syncobj_add_point(sig_sync->syncobj, sig_sync->chain,
					      sig_sync->fence, sig_sync->point);
			sig_sync->chain = NULL;
		} else {
			drm_syncobj_replace_fence(sig_sync->syncobj, sig_sync->fence);
		}
	}
}

static int
pvr_job_add_deps(struct pvr_file *pvr_file, struct pvr_job *job,
		 u32 sync_op_count, const struct drm_pvr_sync_op *sync_ops,
		 struct xarray *signal_array)
{
	int err = 0;

	if (!sync_op_count)
		return 0;

	for (u32 i = 0; i < sync_op_count; i++) {
		struct dma_fence *unwrapped_fence, *fence;
		struct pvr_sync_signal *sig_sync;
		struct dma_fence_unwrap iter;
		u32 native_fence_count = 0;

		if (sync_ops[i].flags & DRM_PVR_SYNC_OP_FLAG_SIGNAL)
			continue;

		err = pvr_check_sync_op(&sync_ops[i]);
		if (err)
			return err;

		sig_sync = pvr_sync_signal_array_search(signal_array, sync_ops[i].handle,
							sync_ops[i].value);
		if (sig_sync) {
			if (WARN_ON(!sig_sync->fence))
				return -EINVAL;

			fence = dma_fence_get(sig_sync->fence);
		} else {
			err = drm_syncobj_find_fence(from_pvr_file(pvr_file), sync_ops[i].handle,
						     sync_ops[i].value, 0, &fence);
			if (err)
				return err;
		}

		dma_fence_unwrap_for_each(unwrapped_fence, &iter, fence) {
			if (pvr_queue_fence_is_ufo_backed(unwrapped_fence))
				native_fence_count++;
		}

		if (!native_fence_count) {
			/* No need to unwrap the fence if it's fully non-native. */
			err = drm_sched_job_add_dependency(&job->base, fence);
			if (err)
				return err;
		} else {
			dma_fence_unwrap_for_each(unwrapped_fence, &iter, fence) {
				/* There's no dma_fence_unwrap_stop() helper cleaning up the refs
				 * owned by dma_fence_unwrap(), so let's just iterate over all
				 * entries without doing anything when something failed.
				 */
				if (err)
					continue;

				dma_fence_get(unwrapped_fence);
				err = drm_sched_job_add_dependency(&job->base, unwrapped_fence);
			}

			dma_fence_put(fence);

			if (err)
				return err;
		}
	}

	return 0;
}

/**
 * pvr_job_process_stream() - Build job FW structure from stream
 * @pvr_dev: Device pointer.
 * @cmd_defs: Stream definition.
 * @stream: Pointer to command stream.
 * @stream_size: Size of command stream, in bytes.
 * @job: Pointer to job.
 *
 * Caller is responsible for freeing the output structure.
 *
 * Returns:
 *  * 0 on success,
 *  * -%ENOMEM on out of memory, or
 *  * -%EINVAL on malformed stream.
 */
static int
pvr_job_process_stream(struct pvr_device *pvr_dev, const struct pvr_stream_cmd_defs *cmd_defs,
		       void *stream, u32 stream_size, struct pvr_job *job)
{
	int err;

	job->cmd = kzalloc(cmd_defs->dest_size, GFP_KERNEL);
	if (!job->cmd) {
		err = -ENOMEM;
		goto err_out;
	}

	job->cmd_len = cmd_defs->dest_size;

	err = pvr_stream_process(pvr_dev, cmd_defs, stream, stream_size, job->cmd);
	if (err)
		kfree(job->cmd);

err_out:
	return err;
}

static int pvr_fw_cmd_init(struct pvr_device *pvr_dev, struct pvr_job *job,
			   const struct pvr_stream_cmd_defs *stream_def,
			   u64 stream_userptr, u32 stream_len)
{
	void *stream;
	int err;

	stream = kzalloc(stream_len, GFP_KERNEL);
	if (!stream) {
		err = -ENOMEM;
		goto err_out;
	}

	if (copy_from_user(stream, u64_to_user_ptr(stream_userptr), stream_len)) {
		err = -EFAULT;
		goto err_free_stream;
	}

	err = pvr_job_process_stream(pvr_dev, stream_def, stream, stream_len, job);

err_free_stream:
	kfree(stream);

err_out:
	return err;
}

static u32
convert_geom_flags(u32 in_flags)
{
	u32 out_flags = 0;

	if (in_flags & DRM_PVR_SUBMIT_JOB_GEOM_CMD_FIRST)
		out_flags |= ROGUE_GEOM_FLAGS_FIRSTKICK;
	if (in_flags & DRM_PVR_SUBMIT_JOB_GEOM_CMD_LAST)
		out_flags |= ROGUE_GEOM_FLAGS_LASTKICK;
	if (in_flags & DRM_PVR_SUBMIT_JOB_GEOM_CMD_SINGLE_CORE)
		out_flags |= ROGUE_GEOM_FLAGS_SINGLE_CORE;

	return out_flags;
}

static u32
convert_frag_flags(u32 in_flags)
{
	u32 out_flags = 0;

	if (in_flags & DRM_PVR_SUBMIT_JOB_FRAG_CMD_SINGLE_CORE)
		out_flags |= ROGUE_FRAG_FLAGS_SINGLE_CORE;
	if (in_flags & DRM_PVR_SUBMIT_JOB_FRAG_CMD_DEPTHBUFFER)
		out_flags |= ROGUE_FRAG_FLAGS_DEPTHBUFFER;
	if (in_flags & DRM_PVR_SUBMIT_JOB_FRAG_CMD_STENCILBUFFER)
		out_flags |= ROGUE_FRAG_FLAGS_STENCILBUFFER;
	if (in_flags & DRM_PVR_SUBMIT_JOB_FRAG_CMD_PREVENT_CDM_OVERLAP)
		out_flags |= ROGUE_FRAG_FLAGS_PREVENT_CDM_OVERLAP;
	if (in_flags & DRM_PVR_SUBMIT_JOB_FRAG_CMD_GET_VIS_RESULTS)
		out_flags |= ROGUE_FRAG_FLAGS_GET_VIS_RESULTS;

	return out_flags;
}

static int
pvr_geom_job_fw_cmd_init(struct pvr_job *job,
			 struct drm_pvr_job *args)
{
	struct rogue_fwif_cmd_geom *cmd;
	int err;

	if (args->flags & ~DRM_PVR_SUBMIT_JOB_GEOM_CMD_FLAGS_MASK)
		return -EINVAL;

	if (job->ctx->type != DRM_PVR_CTX_TYPE_RENDER)
		return -EINVAL;

	if (!job->hwrt)
		return -EINVAL;

	job->fw_ccb_cmd_type = ROGUE_FWIF_CCB_CMD_TYPE_GEOM;
	err = pvr_fw_cmd_init(job->pvr_dev, job, &pvr_cmd_geom_stream,
			      args->cmd_stream, args->cmd_stream_len);
	if (err)
		return err;

	cmd = job->cmd;
	cmd->cmd_shared.cmn.frame_num = 0;
	cmd->flags = convert_geom_flags(args->flags);
	pvr_fw_object_get_fw_addr(job->hwrt->fw_obj, &cmd->cmd_shared.hwrt_data_fw_addr);
	return 0;
}

static int
pvr_frag_job_fw_cmd_init(struct pvr_job *job,
			 struct drm_pvr_job *args)
{
	struct rogue_fwif_cmd_frag *cmd;
	int err;

	if (args->flags & ~DRM_PVR_SUBMIT_JOB_FRAG_CMD_FLAGS_MASK)
		return -EINVAL;

	if (job->ctx->type != DRM_PVR_CTX_TYPE_RENDER)
		return -EINVAL;

	if (!job->hwrt)
		return -EINVAL;

	job->fw_ccb_cmd_type = ROGUE_FWIF_CCB_CMD_TYPE_FRAG;
	err = pvr_fw_cmd_init(job->pvr_dev, job, &pvr_cmd_frag_stream,
			      args->cmd_stream, args->cmd_stream_len);
	if (err)
		return err;

	cmd = job->cmd;
	cmd->cmd_shared.cmn.frame_num = 0;
	cmd->flags = convert_frag_flags(args->flags);
	pvr_fw_object_get_fw_addr(job->hwrt->fw_obj, &cmd->cmd_shared.hwrt_data_fw_addr);
	return 0;
}

static u32
convert_compute_flags(u32 in_flags)
{
	u32 out_flags = 0;

	if (in_flags & DRM_PVR_SUBMIT_JOB_COMPUTE_CMD_PREVENT_ALL_OVERLAP)
		out_flags |= ROGUE_COMPUTE_FLAG_PREVENT_ALL_OVERLAP;
	if (in_flags & DRM_PVR_SUBMIT_JOB_COMPUTE_CMD_SINGLE_CORE)
		out_flags |= ROGUE_COMPUTE_FLAG_SINGLE_CORE;

	return out_flags;
}

static int
pvr_compute_job_fw_cmd_init(struct pvr_job *job,
			    struct drm_pvr_job *args)
{
	struct rogue_fwif_cmd_compute *cmd;
	int err;

	if (args->flags & ~DRM_PVR_SUBMIT_JOB_COMPUTE_CMD_FLAGS_MASK)
		return -EINVAL;

	if (job->ctx->type != DRM_PVR_CTX_TYPE_COMPUTE)
		return -EINVAL;

	job->fw_ccb_cmd_type = ROGUE_FWIF_CCB_CMD_TYPE_CDM;
	err = pvr_fw_cmd_init(job->pvr_dev, job, &pvr_cmd_compute_stream,
			      args->cmd_stream, args->cmd_stream_len);
	if (err)
		return err;

	cmd = job->cmd;
	cmd->common.frame_num = 0;
	cmd->flags = convert_compute_flags(args->flags);
	return 0;
}

static u32
convert_transfer_flags(u32 in_flags)
{
	u32 out_flags = 0;

	if (in_flags & DRM_PVR_SUBMIT_JOB_TRANSFER_CMD_SINGLE_CORE)
		out_flags |= ROGUE_TRANSFER_FLAGS_SINGLE_CORE;

	return out_flags;
}

static int
pvr_transfer_job_fw_cmd_init(struct pvr_job *job,
			     struct drm_pvr_job *args)
{
	struct rogue_fwif_cmd_transfer *cmd;
	int err;

	if (args->flags & ~DRM_PVR_SUBMIT_JOB_TRANSFER_CMD_FLAGS_MASK)
		return -EINVAL;

	if (job->ctx->type != DRM_PVR_CTX_TYPE_TRANSFER_FRAG)
		return -EINVAL;

	job->fw_ccb_cmd_type = ROGUE_FWIF_CCB_CMD_TYPE_TQ_3D;
	err = pvr_fw_cmd_init(job->pvr_dev, job, &pvr_cmd_transfer_stream,
			      args->cmd_stream, args->cmd_stream_len);
	if (err)
		return err;

	cmd = job->cmd;
	cmd->common.frame_num = 0;
	cmd->flags = convert_transfer_flags(args->flags);
	return 0;
}

static int
pvr_job_fw_cmd_init(struct pvr_job *job,
		    struct drm_pvr_job *args)
{
	switch (args->type) {
	case DRM_PVR_JOB_TYPE_GEOMETRY:
		return pvr_geom_job_fw_cmd_init(job, args);

	case DRM_PVR_JOB_TYPE_FRAGMENT:
		return pvr_frag_job_fw_cmd_init(job, args);

	case DRM_PVR_JOB_TYPE_COMPUTE:
		return pvr_compute_job_fw_cmd_init(job, args);

	case DRM_PVR_JOB_TYPE_TRANSFER_FRAG:
		return pvr_transfer_job_fw_cmd_init(job, args);

	default:
		return -EINVAL;
	}
}

static struct pvr_job *
pvr_create_job(struct pvr_device *pvr_dev,
	       struct pvr_file *pvr_file,
	       struct drm_pvr_job *args,
	       struct xarray *signal_array)
{
	struct drm_pvr_sync_op *sync_ops = NULL;
	struct dma_fence *done_fence;
	struct pvr_job *job = NULL;
	int err;

	if (!args->cmd_stream || !args->cmd_stream_len)
		return ERR_PTR(-EINVAL);

	if (args->type != DRM_PVR_JOB_TYPE_GEOMETRY &&
	    args->type != DRM_PVR_JOB_TYPE_FRAGMENT &&
	    (args->hwrt.set_handle || args->hwrt.data_index))
		return ERR_PTR(-EINVAL);

	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (!job)
		return ERR_PTR(-ENOMEM);

	kref_init(&job->ref_count);
	job->type = args->type;
	job->pvr_dev = pvr_dev;

	err = xa_alloc(&pvr_dev->job_ids, &job->id, job, xa_limit_32b, GFP_KERNEL);
	if (err)
		goto err_put_job;

	err = PVR_UOBJ_GET_ARRAY(sync_ops, &args->sync_ops);
	if (err)
		goto err_put_job;

	err = pvr_sync_signal_array_collect_ops(signal_array, from_pvr_file(pvr_file),
						args->sync_ops.count, sync_ops);
	if (err)
		goto err_put_job;

	job->ctx = pvr_context_lookup(pvr_file, args->context_handle);
	if (!job->ctx) {
		err = -EINVAL;
		goto err_put_job;
	}

	if (args->hwrt.set_handle) {
		job->hwrt = pvr_hwrt_data_lookup(pvr_file, args->hwrt.set_handle,
						 args->hwrt.data_index);
		if (!job->hwrt) {
			err = -EINVAL;
			goto err_put_job;
		}
	}

	err = pvr_job_fw_cmd_init(job, args);
	if (err)
		goto err_put_job;

	err = pvr_queue_job_init(job);
	if (err)
		goto err_put_job;

	err = pvr_job_add_deps(pvr_file, job, args->sync_ops.count, sync_ops, signal_array);
	if (err)
		goto err_put_job;

	/* We need to arm the job to get the job done fence. */
	done_fence = pvr_queue_job_arm(job);

	err = pvr_sync_signal_array_update_fences(signal_array,
						  args->sync_ops.count, sync_ops,
						  done_fence);
	if (err)
		goto err_put_job;

	kvfree(sync_ops);
	return job;

err_put_job:
	kvfree(sync_ops);
	pvr_job_put(job);
	return ERR_PTR(err);
}

/**
 * pvr_submit_jobs() - Submit jobs to the GPU
 * @pvr_dev: Target PowerVR device.
 * @pvr_file: Pointer to PowerVR file structure.
 * @args: IOCTL arguments.
 *
 * This initial implementation is entirely synchronous; on return the GPU will
 * be idle. This will not be the case for future implementations.
 *
 * Returns:
 *  * 0 on success,
 *  * -%EFAULT if arguments can not be copied from user space,
 *  * -%EINVAL on invalid arguments, or
 *  * Any other error.
 */
int
pvr_submit_jobs(struct pvr_device *pvr_dev,
		struct pvr_file *pvr_file,
		struct drm_pvr_ioctl_submit_jobs_args *args)
{
	struct drm_pvr_job *jobs_args = NULL;
	DEFINE_XARRAY_ALLOC(signal_array);
	struct pvr_job **jobs = NULL;
	int err;

	if (!args->jobs.count)
		return -EINVAL;

	err = PVR_UOBJ_GET_ARRAY(jobs_args, &args->jobs);
	if (err)
		return err;

	/* NOLINTNEXTLINE(bugprone-sizeof-expression) */
	jobs = kvmalloc_array(args->jobs.count, sizeof(*jobs), GFP_KERNEL | __GFP_ZERO);
	if (!jobs) {
		err = -ENOMEM;
		goto out_free_jobs_args;
	}

	/* FIXME: We should probably not serialize things at the file level,
	 * because that means we prevent parallel submission on separate VkQueue
	 * objects. This mean this lock should go away and be replaced by
	 * something better.
	 *
	 * I see several options here:
	 *
	 * 1. We try to lock all queues being targeted by jobs in the job array
	 *    passed to SUBMIT_JOBS before actually submitting the jobs. This
	 *    requires using ww_mutexes and making the right thing when DEADLCK
	 *    is reported. Note that we don't have to implement our own ww_class,
	 *    we can just use the dma_resv object attached to the FW context
	 *    object (pvr_fw_object->gem->base.base.resv).
	 *    This is a bit convoluted, but I fear we'll have to deal with resv
	 *    objects at some point anyway, even if our driver is explicit-sync
	 *    only (needed if we want to implement memory reclaim).
	 * 2. We group FW contexts into a higher-level abstract matching exactly
	 *    the VkQueue object (we could call those submit contexts) which can
	 *    have several ctx (they currently have four, one gfx, one compute,
	 *    another compute for queries and a transfer context). This approach
	 *    has several advantages: we could make sure the multi-job submission
	 *    only targets a single target submit-context (it doesn't make sense
	 *    to do a multi-job submission on contexts that are not part of the
	 *    same VkQueue), and we get a single lock we can acquire to protect
	 *    access to the queues being targeted by the SUBMIT_JOBS request.
	 *    There's basically 2 ways to implement that:
	 *    A. Rework the CREATE_CONTEXT ioctls so they create those high-level
	 *       submit contexts containing N FW contexts, each of them being
	 *       assigned an index that's directly matching the position of the
	 *       FW context creation args in the
	 *       pvr_create_submit_context::sub_ctxs array
	 *       (pvr_create_submit_context is the new struct taking an array
	 *       of FW context to attach to the submit context). We also need
	 *       to rework the SUBMIT_JOBS ioctl so it get passed a submit context
	 *       handle, and each job is passed the index of the FW context in
	 *       the submit context.
	 *       This also implies reworking the mesa winsys abstraction to
	 *       expose the concept of vk_queue to the winsys and let winsys
	 *       implementation create high-level submit contexts instead of
	 *       asking them to create each FW context independently.
	 *    B. Add 2 new {CREATE,DESTROY}_SUBMIT_CONTEXT ioctls, creating
	 *       those high-level submission contexts, and extend CREATE_CONTEXT
	 *       so it can be passed a submit context handle (if one only wants
	 *       to create an independent FW context, it can just pass a zero
	 *       handle). In the SUBMIT_JOBS path, we iterate over all jobs and
	 *       make sure the contexts they're targeting are part of the same
	 *       submit context, if not, we fail. Once we've done that, we have
	 *       a single lock (attached to the submit context) we can acquire
	 *       to push job to the pvr_queues.
	 *
	 * From a design standpoint, I tend to prefer option 2A., because we
	 * make submit contexts a first class citizen that matches how Vulkan is
	 * going to use the API, rather than trying to retrofit it to original
	 * model. But it's also the most invasive of all options.
	 *
	 * Option 1. might be interesting to look at, so we get things in shape
	 * for the next step: dealing with mem-reclaim in a sane way, adding job
	 * out fences to the VM resv and all external BOs, such that the
	 * memory is guaranteed to be pinned/mapped when the job is being
	 * executed by the GPU. But I like the idea of restricting multi-job
	 * submission to jobs targeting pvr_queues that are part of the same
	 * VkQueue, and this solution doesn't allow that grouping.
	 */
	mutex_lock(&pvr_file->submit_lock);

	for (u32 i = 0; i < args->jobs.count; i++) {
		jobs[i] = pvr_create_job(pvr_dev, pvr_file, &jobs_args[i], &signal_array);
		if (IS_ERR(jobs[i])) {
			err = PTR_ERR(jobs[i]);
			jobs[i] = NULL;
			goto out_submit_unlock;
		}
	}

	for (u32 i = 0; i < args->jobs.count; i++)
		pvr_queue_job_push(jobs[i]);

	pvr_sync_signal_array_push_fences(&signal_array);
	err = 0;

out_submit_unlock:
	mutex_unlock(&pvr_file->submit_lock);

	for (u32 i = 0; i < args->jobs.count; i++)
		pvr_job_put(jobs[i]);

	kvfree(jobs);

out_free_jobs_args:
	pvr_sync_signal_array_cleanup(&signal_array);
	kvfree(jobs_args);
	return err;
}
