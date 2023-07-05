// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#include "pvr_vm.h"

#include "pvr_device.h"
#include "pvr_drv.h"
#include "pvr_gem.h"
#include "pvr_mmu.h"
#include "pvr_rogue_heap_config.h"

#include <drm/drm_gem.h>
#include <drm/drm_gpuva_mgr.h>

#include <linux/container_of.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/gfp_types.h>
#include <linux/kref.h>
#include <linux/mutex.h>
#include <linux/stddef.h>

/**
 * DOC: Memory context
 *
 * This is the "top level" datatype in the VM code. It's exposed in the public
 * API as an opaque handle.
 */

/**
 * struct pvr_vm_context - Context type which encapsulates an entire page table
 * tree structure.
 * @pvr_dev: The PowerVR device to which this context is bound.
 *
 * This binding is immutable for the life of the context.
 * @mmu_ctx: The context for binding to physical memory.
 * @gpuva_mgr: GPUVA manager object associated with this context.
 * @lock: Global lock on this entire structure of page tables.
 * @fw_mem_ctx_obj: Firmware object representing firmware memory context.
 * @ref_count: Reference count of object.
 */
struct pvr_vm_context {
	struct pvr_device *pvr_dev;
	struct pvr_mmu_context *mmu_ctx;
	struct drm_gpuva_manager gpuva_mgr;
	struct mutex lock;
	struct pvr_fw_object *fw_mem_ctx_obj;
	struct kref ref_count;
};

/**
 * pvr_vm_get_page_table_root_addr() - Get the DMA address of the root of the
 *                                     page table structure behind a VM context.
 * @vm_ctx: Target VM context.
 */
dma_addr_t pvr_vm_get_page_table_root_addr(struct pvr_vm_context *vm_ctx)
{
	return pvr_mmu_get_root_table_dma_addr(vm_ctx->mmu_ctx);
}

/**
 * DOC: Memory mappings
 */

/**
 * pvr_vm_gpuva_mapping_init() - Setup a mapping object with the specified
 * parameters ready for mapping using pvr_vm_gpuva_mapping_map().
 * @va: Pointer to drm_gpuva mapping object.
 * @device_addr: Device-virtual address at the start of the mapping.
 * @size: Size of the desired mapping.
 * @pvr_obj: Target PowerVR memory object.
 * @pvr_obj_offset: Offset into @pvr_obj to begin mapping from.
 *
 * Some parameters of this function are unchecked. It is therefore the callers
 * responsibility to ensure certain constraints are met. Specifically:
 *
 * * @pvr_obj_offset must be less than the size of @pvr_obj,
 * * The sum of @pvr_obj_offset and @size must be less than or equal to the
 *   size of @pvr_obj,
 * * The range specified by @pvr_obj_offset and @size (the "CPU range") must be
 *   CPU page-aligned both in start position and size, and
 * * The range specified by @device_addr and @size (the "device range") must be
 *   device page-aligned both in start position and size.
 *
 * Furthermore, it is up to the caller to make sure that a reference to @pvr_obj
 * is taken prior to mapping @va with the drm_gpuva_manager.
 */
static void
pvr_vm_gpuva_mapping_init(struct drm_gpuva *va, u64 device_addr, u64 size,
			  struct pvr_gem_object *pvr_obj, u64 pvr_obj_offset)
{
	va->va.addr = device_addr;
	va->va.range = size;
	va->gem.obj = gem_from_pvr_gem(pvr_obj);
	va->gem.offset = pvr_obj_offset;
}

struct pvr_vm_gpuva_op_ctx {
	struct pvr_vm_context *vm_ctx;
	struct drm_gpuva_prealloc *pa;
	struct drm_gpuva *new_va, *prev_va, *next_va;
};

/**
 * pvr_vm_gpuva_mapping_fini() - Teardown a mapping.
 * @va: Pointer to drm_gpuva mapping object.
 *
 * This function may not be called on a mapping which is currently active. The
 * caller must call pvr_vm_gpuva_mapping_unmap() on @va (or otherwise ensure
 * @va is not currently mapped) before calling this function.
 */
static void
pvr_vm_gpuva_mapping_fini(struct drm_gpuva *va)
{
	pvr_gem_object_put(gem_to_pvr_gem(va->gem.obj));
}

/**
 * pvr_vm_gpuva_map() - Insert a mapping into a memory context.
 * @op: gpuva op containing the remap details.
 * @op_ctx: Operation context.
 *
 * Context: Called by drm_gpuva_sm_map following a successful mapping while
 * @op_ctx.vm_ctx mutex is held.
 *
 * Return:
 *  * 0 on success, or
 *  * Any error encountered while attempting to obtain a reference to the
 *    buffer bound to @op (see pvr_gem_object_get_pages()), or
 *  * Any error returned by pvr_vm_context_map_sgt().
 */
static int
pvr_vm_gpuva_map(struct drm_gpuva_op *op, void *op_ctx)
{
	struct pvr_gem_object *pvr_gem = gem_to_pvr_gem(op->map.gem.obj);
	struct pvr_vm_gpuva_op_ctx *ctx = op_ctx;
	struct sg_table *sgt;
	int err;

	if (!gem_from_pvr_gem(pvr_gem)->import_attach) {
		err = pvr_gem_object_get_pages(pvr_gem);
		if (err)
			return err;
	}

	if ((op->map.gem.offset | op->map.va.range) & ~PVR_DEVICE_PAGE_MASK)
		return -EINVAL;

	sgt = pvr_gem_object_get_pages_sgt(pvr_gem);
	err = PTR_ERR_OR_ZERO(sgt);
	if (err)
		goto err_put_pages;

	err = pvr_mmu_map(ctx->vm_ctx->mmu_ctx, sgt, op->map.gem.offset,
			  op->map.va.range, pvr_gem->flags, op->map.va.addr);
	if (err)
		goto err_put_pages;

	pvr_vm_gpuva_mapping_init(ctx->new_va, op->map.va.addr,
				  op->map.va.range, pvr_gem, op->map.gem.offset);

	err = drm_gpuva_map(&ctx->vm_ctx->gpuva_mgr, ctx->pa, ctx->new_va);
	if (err)
		goto err_put_pages;

	/*
	 * Increment the refcount on the underlying physical memory resource
	 * to prevent de-allocation while the mapping exists.
	 */
	pvr_gem_object_get(pvr_gem);

	drm_gpuva_link(ctx->new_va);
	ctx->new_va = NULL;

	return 0;

err_put_pages:
	if (!gem_from_pvr_gem(pvr_gem)->import_attach)
		pvr_gem_object_put_pages(pvr_gem);

	return err;
}

/**
 * pvr_vm_gpuva_unmap() - Remove a mapping from a memory context.
 * @op: gpuva op containing the unmap details.
 * @state: Iterator.
 * @op_ctx: Operation context.
 *
 * Context: Called by drm_gpuva_sm_unmap following a successful unmapping while
 * @op_ctx.vm_ctx mutex is held.
 *
 * Return:
 *  * 0 on success, or
 *  * Any error returned by pvr_vm_context_unmap().
 */
static int
pvr_vm_gpuva_unmap(struct drm_gpuva_op *op, drm_gpuva_state_t state, void *op_ctx)
{
	struct pvr_gem_object *pvr_gem = gem_to_pvr_gem(op->unmap.va->gem.obj);
	struct pvr_vm_gpuva_op_ctx *ctx = op_ctx;

	int err = pvr_mmu_unmap(ctx->vm_ctx->mmu_ctx, op->unmap.va->va.addr,
				op->unmap.va->va.range >> PVR_DEVICE_PAGE_SHIFT);
	if (err)
		return err;

	drm_gpuva_unmap(state);
	drm_gpuva_unlink(op->unmap.va);

	if (!gem_from_pvr_gem(pvr_gem)->import_attach)
		pvr_gem_object_put_pages(pvr_gem);

	pvr_gem_object_put(pvr_gem);
	kfree(op->unmap.va);
	return 0;
}

/**
 * pvr_vm_gpuva_remap() - Remap a mapping within a memory context.
 * @op: gpuva op containing the remap details.
 * @state: Iterator.
 * @op_ctx: Operation context.
 *
 * Context: Called by either drm_gpuva_sm_map or drm_gpuva_sm_unmap when a
 * mapping or unmapping operation causes a region to be split. The
 * @op_ctx.vm_ctx mutex is held.
 *
 * Return:
 *  * 0 on success, or
 *  * Any error returned by pvr_vm_gpuva_unmap() or pvr_vm_gpuva_unmap().
 */
static int
pvr_vm_gpuva_remap(struct drm_gpuva_op *op, drm_gpuva_state_t state, void *op_ctx)
{
	struct pvr_vm_gpuva_op_ctx *ctx = op_ctx;
	int err;

	if (op->remap.unmap) {
		const u64 va_start = op->remap.prev ?
				     op->remap.prev->va.addr + op->remap.prev->va.range :
				     op->remap.unmap->va->va.addr;
		const u64 va_end = op->remap.next ?
				   op->remap.next->va.addr :
				   op->remap.unmap->va->va.addr + op->remap.unmap->va->va.range;

		err = pvr_mmu_unmap(ctx->vm_ctx->mmu_ctx, va_start,
				    (va_end - va_start) >> PVR_DEVICE_PAGE_SHIFT);
		if (err)
			return err;
	}

	if (op->remap.prev)
		pvr_vm_gpuva_mapping_init(ctx->prev_va, op->remap.prev->va.addr,
					  op->remap.prev->va.range,
					  gem_to_pvr_gem(op->remap.prev->gem.obj),
					  op->remap.prev->gem.offset);

	if (op->remap.next)
		pvr_vm_gpuva_mapping_init(ctx->next_va, op->remap.next->va.addr,
					  op->remap.next->va.range,
					  gem_to_pvr_gem(op->remap.next->gem.obj),
					  op->remap.next->gem.offset);

	/* No actual remap required: the page table tree depth is fixed to 3, and we use 4k
	 * page table entries only for now.
	 */
	err = drm_gpuva_remap(state, ctx->prev_va, ctx->next_va);
	if (err)
		return err;

	if (op->remap.prev) {
		pvr_gem_object_get(gem_to_pvr_gem(ctx->prev_va->gem.obj));
		drm_gpuva_link(ctx->prev_va);
		ctx->prev_va = NULL;
	}

	if (op->remap.next) {
		pvr_gem_object_get(gem_to_pvr_gem(ctx->next_va->gem.obj));
		drm_gpuva_link(ctx->next_va);
		ctx->next_va = NULL;
	}

	if (op->remap.unmap) {
		struct pvr_gem_object *pvr_gem = gem_to_pvr_gem(op->remap.unmap->va->gem.obj);

		drm_gpuva_unlink(op->unmap.va);
		if (!gem_from_pvr_gem(pvr_gem)->import_attach)
			pvr_gem_object_put_pages(pvr_gem);

		pvr_gem_object_put(pvr_gem);
	}

	return 0;
}

/*
 * Public API
 *
 * For an overview of these functions, see *DOC: Public API* in "pvr_vm.h".
 */

/**
 * pvr_device_addr_is_valid() - Tests whether a device-virtual address
 *                              is valid.
 * @device_addr: Virtual device address to test.
 *
 * Return:
 *  * %true if @device_addr is within the valid range for a device page
 *    table and is aligned to the device page size, or
 *  * %false otherwise.
 */
bool
pvr_device_addr_is_valid(u64 device_addr)
{
	return (device_addr & ~PVR_PAGE_TABLE_ADDR_MASK) == 0 &&
	       (device_addr & ~PVR_DEVICE_PAGE_MASK) == 0;
}

/**
 * pvr_device_addr_and_size_are_valid() - Tests whether a device-virtual
 * address and associated size are both valid.
 * @device_addr: Virtual device address to test.
 * @size: Size of the range based at @device_addr to test.
 *
 * Calling pvr_device_addr_is_valid() twice (once on @size, and again on
 * @device_addr + @size) to verify a device-virtual address range initially
 * seems intuitive, but it produces a false-negative when the address range
 * is right at the end of device-virtual address space.
 *
 * This function catches that corner case, as well as checking that
 * @size is non-zero.
 *
 * Return:
 *  * %true if @device_addr is device page aligned; @size is device page
 *    aligned; the range specified by @device_addr and @size is within the
 *    bounds of the device-virtual address space, and @size is non-zero, or
 *  * %false otherwise.
 */
bool
pvr_device_addr_and_size_are_valid(u64 device_addr, u64 size)
{
	return pvr_device_addr_is_valid(device_addr) &&
	       size != 0 && (size & ~PVR_DEVICE_PAGE_MASK) == 0 &&
	       (device_addr + size <= PVR_PAGE_TABLE_ADDR_SPACE_SIZE);
}

static const struct drm_gpuva_fn_ops pvr_vm_gpuva_ops = {
	.sm_step_map = pvr_vm_gpuva_map,
	.sm_step_remap = pvr_vm_gpuva_remap,
	.sm_step_unmap = pvr_vm_gpuva_unmap,
};

static void
fw_mem_context_init(void *cpu_ptr, void *priv)
{
	struct rogue_fwif_fwmemcontext *fw_mem_ctx = cpu_ptr;
	struct pvr_vm_context *vm_ctx = priv;

	fw_mem_ctx->pc_dev_paddr = pvr_vm_get_page_table_root_addr(vm_ctx);
	fw_mem_ctx->page_cat_base_reg_set = ROGUE_FW_BIF_INVALID_PCSET;
}

/**
 * pvr_vm_create_context() - Create a new VM context.
 * @pvr_dev: Target PowerVR device.
 * @is_userspace_context: %true if this context is for userspace. This will
 *                        create a firmware memory context for the VM context
 *                        and disable warnings when tearing down mappings.
 *
 * Return:
 *  * A handle to the newly-minted VM context on success,
 *  * -%EINVAL if the feature "virtual address space bits" on @pvr_dev is
 *    missing or has an unsupported value,
 *  * -%ENOMEM if allocation of the structure behind the opaque handle fails,
 *    or
 *  * Any error encountered while setting up internal structures.
 */
struct pvr_vm_context *
pvr_vm_create_context(struct pvr_device *pvr_dev, bool is_userspace_context)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);

	struct pvr_vm_context *vm_ctx;
	u16 device_addr_bits;

	int err;

	err = PVR_FEATURE_VALUE(pvr_dev, virtual_address_space_bits,
				&device_addr_bits);
	if (err) {
		drm_err(drm_dev,
			"Failed to get device virtual address space bits\n");
		return ERR_PTR(err);
	}

	if (device_addr_bits != PVR_PAGE_TABLE_ADDR_BITS) {
		drm_err(drm_dev,
			"Device has unsupported virtual address space size\n");
		return ERR_PTR(-EINVAL);
	}

	vm_ctx = kzalloc(sizeof(*vm_ctx), GFP_KERNEL);
	if (!vm_ctx)
		return ERR_PTR(-ENOMEM);

	vm_ctx->pvr_dev = pvr_dev;
	kref_init(&vm_ctx->ref_count);
	mutex_init(&vm_ctx->lock);

	drm_gpuva_manager_init(&vm_ctx->gpuva_mgr,
			       is_userspace_context ? "PowerVR-user-VM" : "PowerVR-FW-VM",
			       0, 1ULL << device_addr_bits, 0, 0, &pvr_vm_gpuva_ops);

	vm_ctx->mmu_ctx = pvr_mmu_context_create(pvr_dev);
	err = PTR_ERR_OR_ZERO(&vm_ctx->mmu_ctx);
	if (err) {
		vm_ctx->mmu_ctx = NULL;
		goto err_put_ctx;
	}

	if (is_userspace_context) {
		err = pvr_fw_object_create(pvr_dev, sizeof(struct rogue_fwif_fwmemcontext),
					   PVR_BO_FW_FLAGS_DEVICE_UNCACHED |
					   DRM_PVR_BO_CREATE_ZEROED,
					   fw_mem_context_init, vm_ctx, &vm_ctx->fw_mem_ctx_obj);

		if (err)
			goto err_page_table_destroy;
	}

	return vm_ctx;

err_page_table_destroy:
	pvr_mmu_context_destroy(vm_ctx->mmu_ctx);

err_put_ctx:
	pvr_vm_context_put(vm_ctx);

	return ERR_PTR(err);
}

/**
 * pvr_vm_context_release() - Teardown a VM context.
 * @ref_count: Pointer to reference counter of the VM context.
 *
 * This function ensures that no mappings are left dangling by unmapping them
 * all in order of ascending device-virtual address.
 */
static void
pvr_vm_context_release(struct kref *ref_count)
{
	struct pvr_vm_context *vm_ctx =
		container_of(ref_count, struct pvr_vm_context, ref_count);
	DRM_GPUVA_ITER(it, &vm_ctx->gpuva_mgr, 0);
	struct drm_gpuva *va;

	if (vm_ctx->fw_mem_ctx_obj)
		pvr_fw_object_destroy(vm_ctx->fw_mem_ctx_obj);

	drm_gpuva_iter_for_each(va, it) {
		bool can_release_gem_obj;

		if (WARN_ON(unlikely(va == &vm_ctx->gpuva_mgr.kernel_alloc_node)))
			continue;

		drm_gpuva_iter_remove(&it);

		drm_gem_gpuva_lock(va->gem.obj);
		can_release_gem_obj =
			!WARN_ON(pvr_mmu_unmap(vm_ctx->mmu_ctx, va->va.addr,
					       va->va.range >> PVR_DEVICE_PAGE_SHIFT));
		if (can_release_gem_obj && !va->gem.obj->import_attach)
			pvr_gem_object_put_pages(gem_to_pvr_gem(va->gem.obj));
		drm_gpuva_unlink(va);
		drm_gem_gpuva_unlock(va->gem.obj);

		if (can_release_gem_obj)
			pvr_vm_gpuva_mapping_fini(va);
		kfree(va);
	}

	drm_gpuva_manager_destroy(&vm_ctx->gpuva_mgr);
	pvr_mmu_context_destroy(vm_ctx->mmu_ctx);
	mutex_destroy(&vm_ctx->lock);

	kfree(vm_ctx);
}

/**
 * pvr_vm_context_lookup() - Look up VM context from handle
 * @pvr_file: Pointer to pvr_file structure.
 * @handle: Object handle.
 *
 * Takes reference on VM context object. Call pvr_vm_context_put() to release.
 *
 * Returns:
 *  * The requested object on success, or
 *  * %NULL on failure (object does not exist in list, or is not a VM context)
 */
struct pvr_vm_context *
pvr_vm_context_lookup(struct pvr_file *pvr_file, u32 handle)
{
	struct pvr_vm_context *vm_ctx;

	xa_lock(&pvr_file->vm_ctx_handles);
	vm_ctx = xa_load(&pvr_file->vm_ctx_handles, handle);
	if (vm_ctx)
		kref_get(&vm_ctx->ref_count);

	xa_unlock(&pvr_file->vm_ctx_handles);

	return vm_ctx;
}

/**
 * pvr_vm_context_put() - Release a reference on a VM context
 * @vm_ctx: Target VM context.
 *
 * Returns:
 *  * %true if the VM context was destroyed, or
 *  * %false if there are any references still remaining.
 */
bool
pvr_vm_context_put(struct pvr_vm_context *vm_ctx)
{
	WARN_ON(!vm_ctx);

	if (vm_ctx)
		return kref_put(&vm_ctx->ref_count, pvr_vm_context_release);

	return true;
}

/**
 * pvr_destroy_vm_contexts_for_file: Destroy any VM contexts associated with the
 * given file.
 * @pvr_file: Pointer to pvr_file structure.
 *
 * Removes all vm_contexts associated with @pvr_file from the device VM context
 * list and drops initial references. vm_contexts will then be destroyed once
 * all outstanding references are dropped.
 */
void pvr_destroy_vm_contexts_for_file(struct pvr_file *pvr_file)
{
	struct pvr_vm_context *vm_ctx;
	unsigned long handle;

	xa_for_each(&pvr_file->vm_ctx_handles, handle, vm_ctx) {
		/* vm_ctx is not used here because that would create a race with xa_erase */
		pvr_vm_context_put(xa_erase(&pvr_file->vm_ctx_handles, handle));
	}
}

/**
 * pvr_vm_map() - Map a section of physical memory into a section of device-virtual memory.
 * @vm_ctx: Target VM context.
 * @pvr_obj: Target PowerVR memory object.
 * @pvr_obj_offset: Offset into @pvr_obj to map from.
 * @device_addr: Virtual device address at the start of the requested mapping.
 * @size: Size of the requested mapping.
 *
 * No handle is returned to represent the mapping. Instead, callers should
 * remember @device_addr and use that as a handle.
 *
 * Return:
 *  * 0 on success,
 *  * -%EINVAL if @device_addr is not a valid page-aligned device-virtual
 *    address; the region specified by @pvr_obj_offset and @size does not fall
 *    entirely within @pvr_obj, or any part of the specified region of @pvr_obj
 *    is not device-virtual page-aligned,
 *  * Any error encountered while performing internal operations required to
 *    destroy the mapping (returned from pvr_vm_gpuva_map or
 *    pvr_vm_gpuva_remap).
 */
int
pvr_vm_map(struct pvr_vm_context *vm_ctx,
	   struct pvr_gem_object *pvr_obj, u64 pvr_obj_offset,
	   u64 device_addr, u64 size)
{
	const size_t pvr_obj_size = pvr_gem_object_size(pvr_obj);
	struct pvr_vm_gpuva_op_ctx op_ctx = { .vm_ctx = vm_ctx };
	int err;

	if (!pvr_device_addr_and_size_are_valid(device_addr, size) ||
	    pvr_obj_offset & ~PAGE_MASK || size & ~PAGE_MASK ||
	    pvr_obj_offset + size > pvr_obj_size ||
	    pvr_obj_offset > pvr_obj_size) {
		return -EINVAL;
	}

	op_ctx.pa = drm_gpuva_prealloc_create(&vm_ctx->gpuva_mgr);
	if (!op_ctx.pa)
		return -ENOMEM;

	op_ctx.new_va = kzalloc(sizeof(*op_ctx.new_va), GFP_KERNEL);
	op_ctx.prev_va = kzalloc(sizeof(*op_ctx.prev_va), GFP_KERNEL);
	op_ctx.next_va = kzalloc(sizeof(*op_ctx.next_va), GFP_KERNEL);
	if (!op_ctx.new_va || !op_ctx.prev_va || !op_ctx.next_va) {
		err = -ENOMEM;
		goto out;
	}

	mutex_lock(&vm_ctx->lock);
	err = drm_gpuva_sm_map(&vm_ctx->gpuva_mgr, &op_ctx, device_addr, size,
			       gem_from_pvr_gem(pvr_obj), pvr_obj_offset);
	mutex_unlock(&vm_ctx->lock);

out:
	kfree(op_ctx.next_va);
	kfree(op_ctx.prev_va);
	kfree(op_ctx.new_va);
	drm_gpuva_prealloc_destroy(op_ctx.pa);
	return err;
}

/**
 * pvr_vm_unmap() - Unmap an already mapped section of device-virtual memory.
 * @vm_ctx: Target VM context.
 * @device_addr: Virtual device address at the start of the target mapping.
 * @size: Size of the target mapping.
 *
 * Return:
 *  * 0 on success,
 *  * -%EINVAL if @device_addr is not a valid page-aligned device-virtual
 *    address,
 *  * Any error encountered while performing internal operations required to
 *    destroy the mapping (returned from pvr_vm_gpuva_unmap or
 *    pvr_vm_gpuva_remap).
 */
int
pvr_vm_unmap(struct pvr_vm_context *vm_ctx, u64 device_addr, u64 size)
{
	struct pvr_vm_gpuva_op_ctx op_ctx = { .vm_ctx = vm_ctx };
	int err;

	if (!pvr_device_addr_and_size_are_valid(device_addr, size))
		return -EINVAL;

	op_ctx.prev_va = kzalloc(sizeof(*op_ctx.prev_va), GFP_KERNEL);
	op_ctx.next_va = kzalloc(sizeof(*op_ctx.next_va), GFP_KERNEL);
	if (!op_ctx.prev_va || !op_ctx.next_va) {
		err = -ENOMEM;
		goto out;
	}

	mutex_lock(&vm_ctx->lock);
	err = drm_gpuva_sm_unmap(&vm_ctx->gpuva_mgr, &op_ctx, device_addr, size);
	mutex_unlock(&vm_ctx->lock);

out:
	kfree(op_ctx.next_va);
	kfree(op_ctx.prev_va);
	return err;
}

/*
 * Static data areas are determined by firmware.
 *
 * When adding a new static data area you will also need to update the reserved_size field for the
 * heap in pvr_heaps[].
 */
static const struct drm_pvr_static_data_area static_data_areas[] = {
	{
		.area_usage = DRM_PVR_STATIC_DATA_AREA_FENCE,
		.location_heap_id = DRM_PVR_HEAP_GENERAL,
		.offset = 0,
		.size = 128,
	},
	{
		.area_usage = DRM_PVR_STATIC_DATA_AREA_YUV_CSC,
		.location_heap_id = DRM_PVR_HEAP_GENERAL,
		.offset = 128,
		.size = 1024,
	},
	{
		.area_usage = DRM_PVR_STATIC_DATA_AREA_VDM_SYNC,
		.location_heap_id = DRM_PVR_HEAP_PDS_CODE_DATA,
		.offset = 0,
		.size = 128,
	},
	{
		.area_usage = DRM_PVR_STATIC_DATA_AREA_EOT,
		.location_heap_id = DRM_PVR_HEAP_PDS_CODE_DATA,
		.offset = 128,
		.size = 128,
	},
	{
		.area_usage = DRM_PVR_STATIC_DATA_AREA_VDM_SYNC,
		.location_heap_id = DRM_PVR_HEAP_USC_CODE,
		.offset = 0,
		.size = 128,
	},
};

#define GET_RESERVED_SIZE(last_offset, last_size) round_up((last_offset) + (last_size), PAGE_SIZE)

/*
 * The values given to GET_RESERVED_SIZE() are taken from the last entry in the corresponding
 * static data area for each heap.
 */
static const struct drm_pvr_heap pvr_heaps[] = {
	[DRM_PVR_HEAP_GENERAL] = {
		.base = ROGUE_GENERAL_HEAP_BASE,
		.size = ROGUE_GENERAL_HEAP_SIZE,
		.flags = 0,
		.page_size_log2 = PVR_DEVICE_PAGE_SHIFT,
	},
	[DRM_PVR_HEAP_PDS_CODE_DATA] = {
		.base = ROGUE_PDSCODEDATA_HEAP_BASE,
		.size = ROGUE_PDSCODEDATA_HEAP_SIZE,
		.flags = 0,
		.page_size_log2 = PVR_DEVICE_PAGE_SHIFT,
	},
	[DRM_PVR_HEAP_USC_CODE] = {
		.base = ROGUE_USCCODE_HEAP_BASE,
		.size = ROGUE_USCCODE_HEAP_SIZE,
		.flags = 0,
		.page_size_log2 = PVR_DEVICE_PAGE_SHIFT,
	},
	[DRM_PVR_HEAP_RGNHDR] = {
		.base = ROGUE_RGNHDR_HEAP_BASE,
		.size = ROGUE_RGNHDR_HEAP_SIZE,
		.flags = 0,
		.page_size_log2 = PVR_DEVICE_PAGE_SHIFT,
	},
	[DRM_PVR_HEAP_VIS_TEST] = {
		.base = ROGUE_VISTEST_HEAP_BASE,
		.size = ROGUE_VISTEST_HEAP_SIZE,
		.flags = 0,
		.page_size_log2 = PVR_DEVICE_PAGE_SHIFT,
	},
	[DRM_PVR_HEAP_TRANSFER_FRAG] = {
		.base = ROGUE_TRANSFER_FRAG_HEAP_BASE,
		.size = ROGUE_TRANSFER_FRAG_HEAP_SIZE,
		.flags = 0,
		.page_size_log2 = PVR_DEVICE_PAGE_SHIFT,
	},
};

int
pvr_static_data_areas_get(const struct pvr_device *pvr_dev,
			  struct drm_pvr_ioctl_dev_query_args *args)
{
	struct drm_pvr_dev_query_static_data_areas query = {0};
	int err;

	if (!args->pointer) {
		args->size = sizeof(struct drm_pvr_dev_query_static_data_areas);
		return 0;
	}

	err = PVR_UOBJ_GET(query, args->size, args->pointer);
	if (err < 0)
		return err;

	if (!query.static_data_areas.array) {
		query.static_data_areas.count = ARRAY_SIZE(static_data_areas);
		query.static_data_areas.stride = sizeof(struct drm_pvr_static_data_area);
		goto copy_out;
	}

	if (query.static_data_areas.count > ARRAY_SIZE(static_data_areas))
		query.static_data_areas.count = ARRAY_SIZE(static_data_areas);

	err = PVR_UOBJ_SET_ARRAY(&query.static_data_areas, static_data_areas);
	if (err < 0)
		return err;

copy_out:
	err = PVR_UOBJ_SET(args->pointer, args->size, query);
	if (err < 0)
		return err;

	args->size = sizeof(query);
	return 0;
}

int
pvr_heap_info_get(const struct pvr_device *pvr_dev,
		  struct drm_pvr_ioctl_dev_query_args *args)
{
	struct drm_pvr_dev_query_heap_info query = {0};
	u64 dest;
	int err;

	if (!args->pointer) {
		args->size = sizeof(struct drm_pvr_dev_query_heap_info);
		return 0;
	}

	err = PVR_UOBJ_GET(query, args->size, args->pointer);
	if (err < 0)
		return err;

	if (!query.heaps.array) {
		query.heaps.count = ARRAY_SIZE(pvr_heaps);
		query.heaps.stride = sizeof(struct drm_pvr_heap);
		goto copy_out;
	}

	if (query.heaps.count > ARRAY_SIZE(pvr_heaps))
		query.heaps.count = ARRAY_SIZE(pvr_heaps);

	/* Region header heap is only present if BRN63142 is present. */
	dest = query.heaps.array;
	for (size_t i = 0; i < query.heaps.count; i++) {
		struct drm_pvr_heap heap = pvr_heaps[i];

		if (i == DRM_PVR_HEAP_RGNHDR && !PVR_HAS_QUIRK(pvr_dev, 63142))
			heap.size = 0;

		err = PVR_UOBJ_SET(dest, query.heaps.stride, heap);
		if (err < 0)
			return err;

		dest += query.heaps.stride;
	}

copy_out:
	err = PVR_UOBJ_SET(args->pointer, args->size, query);
	if (err < 0)
		return err;

	args->size = sizeof(query);
	return 0;
}

/**
 * pvr_heap_contains_range() - Determine if a given heap contains the specified
 *                             device-virtual address range.
 * @pvr_heap: Target heap.
 * @start: Inclusive start of the target range.
 * @end: Inclusive end of the target range.
 *
 * It is an error to call this function with values of @start and @end that do
 * not satisfy the condition @start <= @end.
 */
static __always_inline bool
pvr_heap_contains_range(const struct drm_pvr_heap *pvr_heap, u64 start, u64 end)
{
	return pvr_heap->base <= start && end < pvr_heap->base + pvr_heap->size;
}

/**
 * pvr_find_heap_containing() - Find a heap which contains the specified
 *                              device-virtual address range.
 * @pvr_dev: Target PowerVR device.
 * @start: Start of the target range.
 * @size: Size of the target range.
 *
 * Return:
 *  * A pointer to a constant instance of struct drm_pvr_heap representing the
 *    heap containing the entire range specified by @start and @size on
 *    success, or
 *  * %NULL if no such heap exists.
 */
const struct drm_pvr_heap *
pvr_find_heap_containing(struct pvr_device *pvr_dev, u64 start, u64 size)
{
	u64 end;

	if (check_add_overflow(start, size - 1, &end))
		return NULL;

	/*
	 * There are no guarantees about the order of address ranges in
	 * &pvr_heaps, so iterate over the entire array for a heap whose
	 * range completely encompasses the given range.
	 */
	for (u32 heap_id = 0; heap_id < ARRAY_SIZE(pvr_heaps); heap_id++) {
		/* Filter heaps that present only with an associated quirk */
		if (heap_id == DRM_PVR_HEAP_RGNHDR &&
		    !PVR_HAS_QUIRK(pvr_dev, 63142)) {
			continue;
		}

		if (pvr_heap_contains_range(&pvr_heaps[heap_id], start, end))
			return &pvr_heaps[heap_id];
	}

	return NULL;
}

/**
 * pvr_vm_find_gem_object() - Look up a buffer object from a given
 *                            device-virtual address.
 * @vm_ctx: [IN] Target VM context.
 * @device_addr: [IN] Virtual device address at the start of the required
 *               object.
 * @mapped_offset_out: [OUT] Pointer to location to write offset of the start
 *                     of the mapped region within the buffer object. May be
 *                     %NULL if this information is not required.
 * @mapped_size_out: [OUT] Pointer to location to write size of the mapped
 *                   region. May be %NULL if this information is not required.
 *
 * If successful, a reference will be taken on the buffer object. The caller
 * must drop the reference with pvr_gem_object_put().
 *
 * Return:
 *  * The PowerVR buffer object mapped at @device_addr if one exists, or
 *  * %NULL otherwise.
 */
struct pvr_gem_object *
pvr_vm_find_gem_object(struct pvr_vm_context *vm_ctx, u64 device_addr,
		       u64 *mapped_offset_out, u64 *mapped_size_out)
{
	struct pvr_gem_object *pvr_obj;
	struct drm_gpuva *va;

	mutex_lock(&vm_ctx->lock);

	va = drm_gpuva_find_first(&vm_ctx->gpuva_mgr, device_addr, 1);
	if (!va)
		goto err_unlock;

	pvr_obj = gem_to_pvr_gem(va->gem.obj);
	pvr_gem_object_get(pvr_obj);

	if (mapped_offset_out)
		*mapped_offset_out = va->gem.offset;
	if (mapped_size_out)
		*mapped_size_out = va->va.range;

	mutex_unlock(&vm_ctx->lock);

	return pvr_obj;

err_unlock:
	mutex_unlock(&vm_ctx->lock);

	return NULL;
}

/**
 * pvr_vm_get_fw_mem_context: Get object representing firmware memory context
 * @vm_ctx: Target VM context.
 *
 * Returns:
 *  * FW object representing firmware memory context, or
 *  * %NULL if this VM context does not have a firmware memory context.
 */
struct pvr_fw_object *
pvr_vm_get_fw_mem_context(struct pvr_vm_context *vm_ctx)
{
	return vm_ctx->fw_mem_ctx_obj;
}
