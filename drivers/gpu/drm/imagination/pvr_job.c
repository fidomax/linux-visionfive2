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
#include "pvr_sync.h"

#include <drm/drm_gem.h>
#include <linux/types.h>
#include <uapi/drm/pvr_drm.h>

static void pvr_job_release(struct kref *kref)
{
	struct pvr_job *job = container_of(kref, struct pvr_job, ref_count);

	xa_erase(&job->pvr_dev->job_ids, job->id);

	pvr_hwrt_data_put(job->hwrt);
	pvr_context_put(job->ctx);

	WARN_ON(job->paired_job);

	pvr_queue_job_cleanup(job);
	pvr_job_release_pm_ref(job);

	kfree(job->cmd);
	kfree(job);
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
	if (in_flags & DRM_PVR_SUBMIT_JOB_FRAG_CMD_SCRATCHBUFFER)
		out_flags |= ROGUE_FRAG_FLAGS_SCRATCHBUFFER;
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

	job->fw_ccb_cmd_type = (args->flags & DRM_PVR_SUBMIT_JOB_FRAG_CMD_PARTIAL_RENDER) ?
			       ROGUE_FWIF_CCB_CMD_TYPE_FRAG_PR :
			       ROGUE_FWIF_CCB_CMD_TYPE_FRAG;
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

	err = pvr_sync_add_deps_to_job(pvr_file, &job->base,
				       args->sync_ops.count,
				       sync_ops, signal_array);
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

static bool can_combine_jobs(struct pvr_job *a, struct pvr_job *b)
{
	struct pvr_job *geom_job = a, *frag_job = b;
	struct dma_fence *fence;
	unsigned long index;

	/* Geometry and fragment jobs can be combined if they are queued to the
	 * same context and targeting the same HWRT.
	 */
	if (a->type != DRM_PVR_JOB_TYPE_GEOMETRY ||
	    b->type != DRM_PVR_JOB_TYPE_FRAGMENT ||
	    a->ctx != b->ctx ||
	    a->hwrt != b->hwrt)
		return false;

	xa_for_each(&frag_job->base.dependencies, index, fence) {
		/* We combine when we see an explicit geom -> frag dep. */
		if (&geom_job->base.s_fence->scheduled == fence)
			return true;
	}

	return false;
}

static struct dma_fence *
get_last_queued_job_scheduled_fence(struct pvr_queue *queue,
				    struct pvr_job **jobs,
				    u32 cur_job_pos)
{
	/* We iterate over the current job array in reverse order to grab the
	 * last to-be-queued job targetting the same queue.
	 */
	for (u32 i = cur_job_pos; i > 0; i--) {
		struct pvr_job *job = jobs[i - 1];

		if (job->ctx == queue->ctx && job->type == queue->type)
			return dma_fence_get(&job->base.s_fence->scheduled);
	}

	/* If we didn't find any, we just return the last queued job scheduled
	 * fence attached to the queue.
	 */
	return dma_fence_get(queue->last_queued_job_scheduled_fence);
}

static int pvr_jobs_link_geom_frag(struct pvr_job **jobs, u32 job_count)
{
	for (u32 i = 0; i < job_count - 1; i++) {
		struct pvr_job *geom_job = jobs[i], *frag_job = jobs[i + 1];
		struct pvr_queue *frag_queue;
		struct dma_fence *f;
		int err;

		if (!can_combine_jobs(jobs[i], jobs[i + 1]))
			continue;

		/* The fragment job will be submitted by the geometry queue. We need to
		 * make sure it comes after all the other fragment jobs queued before it.
		 */
		frag_queue = pvr_context_get_queue_for_job(frag_job->ctx, frag_job->type);
		f = get_last_queued_job_scheduled_fence(frag_queue, jobs, i);
		if (f) {
			err = drm_sched_job_add_dependency(&geom_job->base, f);
			if (err)
				return err;
		}

		/* The KCCB slot will be reserved by the geometry job, so we can drop the
		 * KCCB fence on the fragment job.
		 */
		pvr_kccb_fence_put(frag_job->kccb_fence);
		frag_job->kccb_fence = NULL;

		geom_job->paired_job = frag_job;
		frag_job->paired_job = geom_job;

		/* Skip the fragment job we just paired to the geometry job. */
		i++;
	}

	return 0;
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

	err = pvr_jobs_link_geom_frag(jobs, args->jobs.count);
	if (err)
		goto out_submit_unlock;

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
