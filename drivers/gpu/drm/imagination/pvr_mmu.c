// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#include "pvr_mmu.h"

#include "pvr_ccb.h"
#include "pvr_device.h"
#include "pvr_fw.h"
#include "pvr_gem.h"
#include "pvr_rogue_fwif.h"
#include "pvr_rogue_mmu_defs.h"

#include <drm/drm_drv.h>
#include <linux/bitops.h>
#include <linux/dma-mapping.h>
#include <linux/kmemleak.h>
#include <linux/minmax.h>
#include <linux/sizes.h>

#define PVR_SHIFT_FROM_SIZE(size_) (__builtin_ctzll(size_))
#define PVR_MASK_FROM_SIZE(size_) (~((size_) - U64_C(1)))

/*
 * The value of the device page size (%PVR_DEVICE_PAGE_SIZE) is currently
 * pegged to the host page size (%PAGE_SIZE). This chunk of macro goodness both
 * ensures that the selected host page size corresponds to a valid device page
 * size and sets up values needed by the MMU code below.
 */
#if (PVR_DEVICE_PAGE_SIZE == SZ_4K)
# define ROGUE_MMUCTRL_PAGE_SIZE_X ROGUE_MMUCTRL_PAGE_SIZE_4KB
# define ROGUE_MMUCTRL_PAGE_X_RANGE_SHIFT ROGUE_MMUCTRL_PAGE_4KB_RANGE_SHIFT
# define ROGUE_MMUCTRL_PAGE_X_RANGE_CLRMSK ROGUE_MMUCTRL_PAGE_4KB_RANGE_CLRMSK
#elif (PVR_DEVICE_PAGE_SIZE == SZ_16K)
# define ROGUE_MMUCTRL_PAGE_SIZE_X ROGUE_MMUCTRL_PAGE_SIZE_16KB
# define ROGUE_MMUCTRL_PAGE_X_RANGE_SHIFT ROGUE_MMUCTRL_PAGE_16KB_RANGE_SHIFT
# define ROGUE_MMUCTRL_PAGE_X_RANGE_CLRMSK ROGUE_MMUCTRL_PAGE_16KB_RANGE_CLRMSK
#elif (PVR_DEVICE_PAGE_SIZE == SZ_64K)
# define ROGUE_MMUCTRL_PAGE_SIZE_X ROGUE_MMUCTRL_PAGE_SIZE_64KB
# define ROGUE_MMUCTRL_PAGE_X_RANGE_SHIFT ROGUE_MMUCTRL_PAGE_64KB_RANGE_SHIFT
# define ROGUE_MMUCTRL_PAGE_X_RANGE_CLRMSK ROGUE_MMUCTRL_PAGE_64KB_RANGE_CLRMSK
#elif (PVR_DEVICE_PAGE_SIZE == SZ_256K)
# define ROGUE_MMUCTRL_PAGE_SIZE_X ROGUE_MMUCTRL_PAGE_SIZE_256KB
# define ROGUE_MMUCTRL_PAGE_X_RANGE_SHIFT ROGUE_MMUCTRL_PAGE_256KB_RANGE_SHIFT
# define ROGUE_MMUCTRL_PAGE_X_RANGE_CLRMSK ROGUE_MMUCTRL_PAGE_256KB_RANGE_CLRMSK
#elif (PVR_DEVICE_PAGE_SIZE == SZ_1M)
# define ROGUE_MMUCTRL_PAGE_SIZE_X ROGUE_MMUCTRL_PAGE_SIZE_1MB
# define ROGUE_MMUCTRL_PAGE_X_RANGE_SHIFT ROGUE_MMUCTRL_PAGE_1MB_RANGE_SHIFT
# define ROGUE_MMUCTRL_PAGE_X_RANGE_CLRMSK ROGUE_MMUCTRL_PAGE_1MB_RANGE_CLRMSK
#elif (PVR_DEVICE_PAGE_SIZE == SZ_2M)
# define ROGUE_MMUCTRL_PAGE_SIZE_X ROGUE_MMUCTRL_PAGE_SIZE_2MB
# define ROGUE_MMUCTRL_PAGE_X_RANGE_SHIFT ROGUE_MMUCTRL_PAGE_2MB_RANGE_SHIFT
# define ROGUE_MMUCTRL_PAGE_X_RANGE_CLRMSK ROGUE_MMUCTRL_PAGE_2MB_RANGE_CLRMSK
#else
# error Unsupported device page size PVR_DEVICE_PAGE_SIZE
#endif

#define ROGUE_MMUCTRL_ENTRIES_PT_VALUE_X   \
	(ROGUE_MMUCTRL_ENTRIES_PT_VALUE >> \
	 (PVR_DEVICE_PAGE_SHIFT - PVR_SHIFT_FROM_SIZE(SZ_4K)))

/**
 * pvr_mmu_flush() - Request flush of all MMU caches.
 * @pvr_dev: Target PowerVR device.
 *
 * This function must be called following any possible change to the MMU page
 * tables.
 *
 * Returns:
 *  * 0 on success, or
 *  * Any error encountered while submitting the flush command via the KCCB.
 */
int
pvr_mmu_flush(struct pvr_device *pvr_dev)
{
	struct rogue_fwif_kccb_cmd cmd_mmu_cache;
	struct rogue_fwif_mmucachedata *cmd_mmu_cache_data =
		&cmd_mmu_cache.cmd_data.mmu_cache_data;
	u32 slot;
	int idx;
	int err;

	if (!drm_dev_enter(from_pvr_device(pvr_dev), &idx))
		return -EIO;

	/* Can't flush MMU if the firmware hasn't booted yet. */
	if (!pvr_dev->fw_dev.booted) {
		err = 0;
		goto err_drm_dev_exit;
	}

	cmd_mmu_cache.cmd_type = ROGUE_FWIF_KCCB_CMD_MMUCACHE;
	/* Request a complete MMU flush, across all pagetable levels, TLBs and contexts. */
	cmd_mmu_cache_data->cache_flags = ROGUE_FWIF_MMUCACHEDATA_FLAGS_PT |
					  ROGUE_FWIF_MMUCACHEDATA_FLAGS_PD |
					  ROGUE_FWIF_MMUCACHEDATA_FLAGS_PC |
					  ROGUE_FWIF_MMUCACHEDATA_FLAGS_TLB |
					  ROGUE_FWIF_MMUCACHEDATA_FLAGS_INTERRUPT;
	pvr_fw_object_get_fw_addr(pvr_dev->fw_dev.mem.mmucache_sync_obj,
				  &cmd_mmu_cache_data->mmu_cache_sync_fw_addr);
	cmd_mmu_cache_data->mmu_cache_sync_update_value = 0;

	err = pvr_kccb_send_cmd(pvr_dev, &cmd_mmu_cache, &slot);
	if (err)
		goto err_drm_dev_exit;

	err = pvr_kccb_wait_for_completion(pvr_dev, slot, HZ, NULL);

err_drm_dev_exit:
	drm_dev_exit(idx);

	return err;
}

/**
 * DOC: PowerVR Virtual Memory Handling
 */
/**
 * DOC: PowerVR Virtual Memory Handling (constants)
 *
 * .. c:macro:: PVR_IDX_INVALID
 *
 *    Default value for a u16-based index.
 *
 *    This value cannot be zero, since zero is a valid index value.
 */
#define PVR_IDX_INVALID ((u16)(-1))

/**
 * DOC: MMU backing pages
 */
/**
 * DOC: MMU backing pages (constants)
 *
 * .. c:macro:: PVR_MMU_BACKING_PAGE_SIZE
 *
 *    Page size of a PowerVR device's integrated MMU. The CPU page size must be
 *    at least as large as this value for the current implementation; this is
 *    checked at compile-time.
 */
#define PVR_MMU_BACKING_PAGE_SIZE SZ_4K
static_assert(PAGE_SIZE >= PVR_MMU_BACKING_PAGE_SIZE);

/**
 * struct pvr_mmu_backing_page - Represents a single page used to back a page
 *                              table of any level.
 * @dma_addr: DMA address of this page.
 * @host_ptr: CPU address of this page.
 * @pvr_dev: The PowerVR device to which this page is associated. **For
 *           internal use only.**
 */
struct pvr_mmu_backing_page {
	dma_addr_t dma_addr;
	void *host_ptr;
/* private: internal use only */
	struct page *raw_page;
	struct pvr_device *pvr_dev;
};

/**
 * pvr_mmu_backing_page_init() - Initialize a MMU backing page.
 * @page: Target backing page.
 * @pvr_dev: Target PowerVR device.
 *
 * This function performs three distinct operations:
 *
 * 1. Allocate a single page,
 * 2. Map the page to the CPU, and
 * 3. Map the page to DMA-space.
 *
 * It is expected that @page be zeroed (e.g. from kzalloc()) before calling
 * this function.
 *
 * Return:
 *  * 0 on success, or
 *  * -%ENOMEM if allocation of the backing page or mapping of the backing
 *    page to DMA fails.
 */
static int
pvr_mmu_backing_page_init(struct pvr_mmu_backing_page *page,
			  struct pvr_device *pvr_dev)
{
	struct device *dev = from_pvr_device(pvr_dev)->dev;

	struct page *raw_page;
	int err;

	dma_addr_t dma_addr;
	void *host_ptr;

	raw_page = alloc_page(__GFP_ZERO | GFP_KERNEL);
	if (!raw_page) {
		err = -ENOMEM;
		goto err_out;
	}

	host_ptr = vmap(&raw_page, 1, VM_MAP, pgprot_writecombine(PAGE_KERNEL));
	if (!host_ptr) {
		err = -ENOMEM;
		goto err_free_page;
	}

	dma_addr = dma_map_page(dev, raw_page, 0, PVR_MMU_BACKING_PAGE_SIZE,
				DMA_TO_DEVICE);
	if (dma_mapping_error(dev, dma_addr)) {
		err = -ENOMEM;
		goto err_unmap_page;
	}

	page->dma_addr = dma_addr;
	page->host_ptr = host_ptr;
	page->pvr_dev = pvr_dev;
	page->raw_page = raw_page;
	kmemleak_alloc(page->host_ptr, PAGE_SIZE, 1, GFP_KERNEL);

	return 0;

err_unmap_page:
	vunmap(host_ptr);

err_free_page:
	__free_page(raw_page);

err_out:
	return err;
}

/**
 * pvr_mmu_backing_page_fini() - Teardown a MMU backing page.
 * @page: Target backing page.
 *
 * This function performs the mirror operations to pvr_mmu_backing_page_init(),
 * in reverse order:
 *
 * 1. Unmap the page from DMA-space,
 * 2. Unmap the page from the CPU, and
 * 3. Free the page.
 *
 * It also zeros @page.
 *
 * It is a no-op to call this function a second (or further) time on any @page.
 */
static void
pvr_mmu_backing_page_fini(struct pvr_mmu_backing_page *page)
{
	struct device *dev = from_pvr_device(page->pvr_dev)->dev;

	/* Do nothing if no allocation is present. */
	if (!page->pvr_dev)
		return;

	dma_unmap_page(dev, page->dma_addr, PVR_MMU_BACKING_PAGE_SIZE,
		       DMA_TO_DEVICE);

	kmemleak_free(page->host_ptr);
	vunmap(page->host_ptr);

	__free_page(page->raw_page);

	memset(page, 0, sizeof(*page));
}

/**
 * pvr_mmu_backing_page_sync() - Flush a MMU backing page from the CPU to the
 *                              device.
 * @page: Target backing page.
 *
 * .. caution::
 *
 *    **This is potentially an expensive function call.** Only call
 *    pvr_mmu_backing_page_sync() once you're sure you have no more changes to
 *    make to the backing page in the immediate future.
 */
static void
pvr_mmu_backing_page_sync(struct pvr_mmu_backing_page *page)
{
	struct device *dev;

	/*
	 * Do nothing if no allocation is present. This may be the case if
	 * we are unmapping pages.
	 */
	if (!page->pvr_dev)
		return;

	dev = from_pvr_device(page->pvr_dev)->dev;

	dma_sync_single_for_device(dev, page->dma_addr,
				   PVR_MMU_BACKING_PAGE_SIZE, DMA_TO_DEVICE);
}

/**
 * DOC: Raw page tables
 */

#define PVR_PAGE_TABLE_TYPEOF_ENTRY(level_) \
	typeof_member(struct pvr_page_table_l##level_##_entry_raw, val)

#define PVR_PAGE_TABLE_FIELD_GET(level_, name_, field_, entry_)           \
	(((entry_).val &                                           \
	  ~ROGUE_MMUCTRL_##name_##_DATA_##field_##_CLRMSK) >> \
	 ROGUE_MMUCTRL_##name_##_DATA_##field_##_SHIFT)

#define PVR_PAGE_TABLE_FIELD_PREP(level_, name_, field_, val_)            \
	((((PVR_PAGE_TABLE_TYPEOF_ENTRY(level_))(val_))            \
	  << ROGUE_MMUCTRL_##name_##_DATA_##field_##_SHIFT) & \
	 ~ROGUE_MMUCTRL_##name_##_DATA_##field_##_CLRMSK)

/**
 * struct pvr_page_table_l2_entry_raw - A single entry in a level 2 page table.
 * @val: The raw value of this entry.
 *
 * This type is a structure for type-checking purposes. At compile-time, its
 * size is checked against %ROGUE_MMUCTRL_ENTRY_SIZE_PC_VALUE.
 *
 * The value stored in this structure can be decoded using the following bitmap:
 *
 * .. flat-table::
 *    :widths: 1 5
 *    :stub-columns: 1
 *
 *    * - 31..4
 *      - **Level 1 Page Table Base Address:** Bits 39..12 of the L1
 *        page table base address, which is 4KiB aligned.
 *
 *    * - 3..2
 *      - *(reserved)*
 *
 *    * - 1
 *      - **Pending:** When valid bit is not set, indicates that a valid
 *        entry is pending and the MMU should wait for the driver to map
 *        the entry. This is used to support page demand mapping of
 *        memory.
 *
 *    * - 0
 *      - **Valid:** Indicates that the entry contains a valid L1 page
 *        table. If the valid bit is not set, then an attempted use of
 *        the page would result in a page fault.
 */
struct pvr_page_table_l2_entry_raw {
	u32 val;
} __packed;
static_assert(sizeof(struct pvr_page_table_l2_entry_raw) * 8 ==
	      ROGUE_MMUCTRL_ENTRY_SIZE_PC_VALUE);

static __always_inline bool
pvr_page_table_l2_entry_raw_is_valid(struct pvr_page_table_l2_entry_raw entry)
{
	return PVR_PAGE_TABLE_FIELD_GET(2, PC, VALID, entry);
}

/**
 * pvr_page_table_l2_entry_raw_set() - Write a valid entry into a raw level 2
 *                                     page table.
 * @entry: Target raw level 2 page table entry.
 * @child_table_dma_addr: DMA address of the level 1 page table to be
 *                        associated with @entry.
 *
 * When calling this function, @child_table_dma_addr must be a valid DMA
 * address and a multiple of %ROGUE_MMUCTRL_PC_DATA_PD_BASE_ALIGNSIZE.
 */
static __always_inline void
pvr_page_table_l2_entry_raw_set(struct pvr_page_table_l2_entry_raw *entry,
				dma_addr_t child_table_dma_addr)
{
	child_table_dma_addr >>= ROGUE_MMUCTRL_PC_DATA_PD_BASE_ALIGNSHIFT;

	entry->val =
		PVR_PAGE_TABLE_FIELD_PREP(2, PC, VALID, true) |
		PVR_PAGE_TABLE_FIELD_PREP(2, PC, ENTRY_PENDING, false) |
		PVR_PAGE_TABLE_FIELD_PREP(2, PC, PD_BASE, child_table_dma_addr);
}

static __always_inline void
pvr_page_table_l2_entry_raw_clear(struct pvr_page_table_l2_entry_raw *entry)
{
	entry->val = 0;
}

/**
 * struct pvr_page_table_l1_entry_raw - A single entry in a level 1 page table.
 * @val: The raw value of this entry.
 *
 * This type is a structure for type-checking purposes. At compile-time, its
 * size is checked against %ROGUE_MMUCTRL_ENTRY_SIZE_PD_VALUE.
 *
 * The value stored in this structure can be decoded using the following bitmap:
 *
 * .. flat-table::
 *    :widths: 1 5
 *    :stub-columns: 1
 *
 *    * - 63..41
 *      - *(reserved)*
 *
 *    * - 40
 *      - **Pending:** When valid bit is not set, indicates that a valid entry
 *        is pending and the MMU should wait for the driver to map the entry.
 *        This is used to support page demand mapping of memory.
 *
 *    * - 39..5
 *      - **Level 0 Page Table Base Address:** The way this value is
 *        interpreted depends on the page size. Bits not specified in the
 *        table below (e.g. bits 11..5 for page size 4KiB) should be
 *        considered reserved.
 *
 *        This table shows the bits used in an L1 page table entry to
 *        represent the Physical Table Base Address for a given Page Size.
 *        Since each L1 page table entry covers 2MiB of address space, the
 *        maximum page size is 2MiB.
 *
 *        .. flat-table::
 *           :widths: 1 1 1 1
 *           :header-rows: 1
 *           :stub-columns: 1
 *
 *           * - Page size
 *             - L0 page table base address bits
 *             - Number of L0 page table entries
 *             - Size of L0 page table
 *
 *           * - 4KiB
 *             - 39..12
 *             - 512
 *             - 4KiB
 *
 *           * - 16KiB
 *             - 39..10
 *             - 128
 *             - 1KiB
 *
 *           * - 64KiB
 *             - 39..8
 *             - 32
 *             - 256B
 *
 *           * - 256KiB
 *             - 39..6
 *             - 8
 *             - 64B
 *
 *           * - 1MiB
 *             - 39..5 (4 = '0')
 *             - 2
 *             - 16B
 *
 *           * - 2MiB
 *             - 39..5 (4..3 = '00')
 *             - 1
 *             - 8B
 *
 *    * - 4
 *      - *(reserved)*
 *
 *    * - 3..1
 *      - **Page Size:** Sets the page size, from 4KiB to 2MiB.
 *
 *    * - 0
 *      - **Valid:** Indicates that the entry contains a valid L0 page table.
 *        If the valid bit is not set, then an attempted use of the page would
 *        result in a page fault.
 */
struct pvr_page_table_l1_entry_raw {
	u64 val;
} __packed;
static_assert(sizeof(struct pvr_page_table_l1_entry_raw) * 8 ==
	      ROGUE_MMUCTRL_ENTRY_SIZE_PD_VALUE);

static __always_inline bool
pvr_page_table_l1_entry_raw_is_valid(struct pvr_page_table_l1_entry_raw entry)
{
	return PVR_PAGE_TABLE_FIELD_GET(1, PD, VALID, entry);
}

/**
 * pvr_page_table_l1_entry_raw_set() - Write a valid entry into a raw level 1
 *                                     page table.
 * @entry: Target raw level 1 page table entry.
 * @child_table_dma_addr: DMA address of the level 0 page table to be
 *                        associated with @entry.
 *
 * When calling this function, @child_table_dma_addr must be a valid DMA
 * address and a multiple of 4 KiB.
 */
static void
pvr_page_table_l1_entry_raw_set(struct pvr_page_table_l1_entry_raw *entry,
				dma_addr_t child_table_dma_addr)
{
	entry->val = PVR_PAGE_TABLE_FIELD_PREP(1, PD, VALID, true) |
		     PVR_PAGE_TABLE_FIELD_PREP(1, PD, ENTRY_PENDING, false) |
		     PVR_PAGE_TABLE_FIELD_PREP(1, PD, PAGE_SIZE,
					       ROGUE_MMUCTRL_PAGE_SIZE_X) |
		     /*
		      * The use of a 4K-specific macro here is correct. It is
		      * a future optimization to allocate sub-host-page-sized
		      * blocks for individual tables, so the condition that any
		      * page table address is aligned to the size of the
		      * largest (a 4KB) table currently holds.
		      */
		     (child_table_dma_addr &
		      ~ROGUE_MMUCTRL_PT_BASE_4KB_RANGE_CLRMSK);
}

static __always_inline void
pvr_page_table_l1_entry_raw_clear(struct pvr_page_table_l1_entry_raw *entry)
{
	entry->val = 0;
}

/**
 * struct pvr_page_table_l0_entry_raw - A single entry in a level 0 page table.
 * @val: The raw value of this entry.
 *
 * This type is a structure for type-checking purposes. At compile-time, its
 * size is checked against %ROGUE_MMUCTRL_ENTRY_SIZE_PT_VALUE.
 *
 * The value stored in this structure can be decoded using the following bitmap:
 *
 * .. flat-table::
 *    :widths: 1 5
 *    :stub-columns: 1
 *
 *    * - 63
 *      - *(reserved)*
 *
 *    * - 62
 *      - **PM/FW Protect:** Indicates a protected region which only the
 *        Parameter Manager (PM) or firmware processor can write to.
 *
 *    * - 61..40
 *      - **VP Page (High):** Virtual-physical page used for Parameter Manager
 *        (PM) memory. This field is only used if the additional level of PB
 *        virtualization is enabled. The VP Page field is needed by the PM in
 *        order to correctly reconstitute the free lists after render
 *        completion. This (High) field holds bits 39..18 of the value; the
 *        Low field holds bits 17..12. Bits 11..0 are always zero because the
 *        value is always aligned to the 4KiB page size.
 *
 *    * - 39..12
 *      - **Physical Page Address:** The way this value is interpreted depends
 *        on the page size. Bits not specified in the table below (e.g. bits
 *        20..12 for page size 2MiB) should be considered reserved.
 *
 *        This table shows the bits used in an L0 page table entry to represent
 *        the Physical Page Address for a given page size (as defined in the
 *        associated L1 page table entry).
 *
 *        .. flat-table::
 *           :widths: 1 1
 *           :header-rows: 1
 *           :stub-columns: 1
 *
 *           * - Page size
 *             - Physical address bits
 *
 *           * - 4KiB
 *             - 39..12
 *
 *           * - 16KiB
 *             - 39..14
 *
 *           * - 64KiB
 *             - 39..16
 *
 *           * - 256KiB
 *             - 39..18
 *
 *           * - 1MiB
 *             - 39..20
 *
 *           * - 2MiB
 *             - 39..21
 *
 *    * - 11..6
 *      - **VP Page (Low):** Continuation of VP Page (High).
 *
 *    * - 5
 *      - **Pending:** When valid bit is not set, indicates that a valid entry
 *        is pending and the MMU should wait for the driver to map the entry.
 *        This is used to support page demand mapping of memory.
 *
 *    * - 4
 *      - **PM Src:** Set on Parameter Manager (PM) allocated page table
 *        entries when indicated by the PM. Note that this bit will only be set
 *        by the PM, not by the device driver.
 *
 *    * - 3
 *      - **SLC Bypass Control:** Specifies requests to this page should bypass
 *        the System Level Cache (SLC), if enabled in SLC configuration.
 *
 *    * - 2
 *      - **Cache Coherency:** Indicates that the page is coherent (i.e. it
 *        does not require a cache flush between operations on the CPU and the
 *        device).
 *
 *    * - 1
 *      - **Read Only:** If set, this bit indicates that the page is read only.
 *        An attempted write to this page would result in a write-protection
 *        fault.
 *
 *    * - 0
 *      - **Valid:** Indicates that the entry contains a valid page. If the
 *        valid bit is not set, then an attempted use of the page would result
 *        in a page fault.
 */
struct pvr_page_table_l0_entry_raw {
	u64 val;
} __packed;
static_assert(sizeof(struct pvr_page_table_l0_entry_raw) * 8 ==
	      ROGUE_MMUCTRL_ENTRY_SIZE_PT_VALUE);

/**
 * struct pvr_page_flags_raw - The configurable flags from a single entry in a
 *                             level 0 page table.
 * @val: The raw value of these flags. Since these are a strict subset of
 *       &struct pvr_page_table_l0_entry_raw; use that type for our member here.
 *
 * The flags stored in this type are: PM/FW Protect; SLC Bypass Control; Cache
 * Coherency, and Read Only (bits 62, 3, 2 and 1 respectively).
 *
 * This type should never be instantiated directly; instead use
 * pvr_page_flags_raw_create() to ensure only valid bits of @val are set.
 */
struct pvr_page_flags_raw {
	struct pvr_page_table_l0_entry_raw val;
} __packed;
static_assert(sizeof(struct pvr_page_flags_raw) ==
	      sizeof(struct pvr_page_table_l0_entry_raw));

static __always_inline bool
pvr_page_table_l0_entry_raw_is_valid(struct pvr_page_table_l0_entry_raw entry)
{
	return PVR_PAGE_TABLE_FIELD_GET(0, PT, VALID, entry);
}

/**
 * pvr_page_table_l0_entry_raw_set() - Write a valid entry into a raw level 0
 *                                     page table.
 * @entry: Target raw level 0 page table entry.
 * @dma_addr: DMA address of the physical page to be associated with @entry.
 * @flags: Options to be set on @entry.
 *
 * When calling this function, @child_table_dma_addr must be a valid DMA
 * address and a multiple of %PVR_DEVICE_PAGE_SIZE.
 *
 * The @flags parameter is directly assigned into @entry. It is the callers
 * responsibility to ensure that only bits specified in
 * &struct pvr_page_flags_raw are set in @flags.
 */
static void
pvr_page_table_l0_entry_raw_set(struct pvr_page_table_l0_entry_raw *entry,
				dma_addr_t dma_addr,
				struct pvr_page_flags_raw flags)
{
	entry->val = PVR_PAGE_TABLE_FIELD_PREP(0, PT, VALID, true) |
		     PVR_PAGE_TABLE_FIELD_PREP(0, PT, ENTRY_PENDING, false) |
		     (dma_addr & ~ROGUE_MMUCTRL_PAGE_X_RANGE_CLRMSK) |
		     flags.val.val;
}

static __always_inline void
pvr_page_table_l0_entry_raw_clear(struct pvr_page_table_l0_entry_raw *entry)
{
	entry->val = 0;
}

/**
 * pvr_page_flags_raw_create() - Initialize the flag bits of a raw level 0 page
 *                               table entry.
 * @read_only: This page is read-only (see: Read Only).
 * @cache_coherent: This page does not require cache flushes (see: Cache
 *                  Coherency).
 * @slc_bypass: This page bypasses the device cache (see: SLC Bypass Control).
 * @pm_fw_protect: This page is only for use by the firmware or Parameter
 *                 Manager (see PM/FW Protect).
 *
 * For more details on the use of these four options, see their respective
 * entries in the table under &struct pvr_page_table_l0_entry_raw.
 *
 * Return:
 * A new &struct pvr_page_flags_raw instance which can be passed directly to
 * pvr_page_table_l0_entry_raw_set() or pvr_page_table_l0_insert().
 */
static struct pvr_page_flags_raw
pvr_page_flags_raw_create(bool read_only, bool cache_coherent, bool slc_bypass,
			  bool pm_fw_protect)
{
	struct pvr_page_flags_raw flags;

	flags.val.val =
		PVR_PAGE_TABLE_FIELD_PREP(0, PT, READ_ONLY, read_only) |
		PVR_PAGE_TABLE_FIELD_PREP(0, PT, CC, cache_coherent) |
		PVR_PAGE_TABLE_FIELD_PREP(0, PT, SLC_BYPASS_CTRL, slc_bypass) |
		PVR_PAGE_TABLE_FIELD_PREP(0, PT, PM_META_PROTECT, pm_fw_protect);

	return flags;
}

/**
 * struct pvr_page_table_l2_raw - The raw data of a level 2 page table.
 *
 * This type is a structure for type-checking purposes. At compile-time, its
 * size is checked against %PVR_MMU_BACKING_PAGE_SIZE.
 */
struct pvr_page_table_l2_raw {
	/** @entries: The raw values of this table. */
	struct pvr_page_table_l2_entry_raw
		entries[ROGUE_MMUCTRL_ENTRIES_PC_VALUE];
} __packed;
static_assert(sizeof(struct pvr_page_table_l2_raw) == PVR_MMU_BACKING_PAGE_SIZE);

/**
 * struct pvr_page_table_l1_raw - The raw data of a level 1 page table.
 *
 * This type is a structure for type-checking purposes. At compile-time, its
 * size is checked against %PVR_MMU_BACKING_PAGE_SIZE.
 */
struct pvr_page_table_l1_raw {
	/** @entries: The raw values of this table. */
	struct pvr_page_table_l1_entry_raw
		entries[ROGUE_MMUCTRL_ENTRIES_PD_VALUE];
} __packed;
static_assert(sizeof(struct pvr_page_table_l1_raw) == PVR_MMU_BACKING_PAGE_SIZE);

/**
 * struct pvr_page_table_l0_raw - The raw data of a level 0 page table.
 *
 * This type is a structure for type-checking purposes. At compile-time, its
 * size is checked against %PVR_MMU_BACKING_PAGE_SIZE.
 *
 * .. caution::
 *
 *    The size of level 0 page tables is variable depending on the page size
 *    specified in the associated level 1 page table entry. Since the device
 *    page size in use is pegged to the host page size, it cannot vary at
 *    runtime. This structure is therefore only defined to contain the required
 *    number of entries for the current device page size. **You should never
 *    read or write beyond the last supported entry.**
 */
struct pvr_page_table_l0_raw {
	/** @entries: The raw values of this table. */
	struct pvr_page_table_l0_entry_raw
		entries[ROGUE_MMUCTRL_ENTRIES_PT_VALUE_X];
} __packed;
static_assert(sizeof(struct pvr_page_table_l0_raw) <= PVR_MMU_BACKING_PAGE_SIZE);

/**
 * DOC: Mirror page tables
 */

/*
 * We pre-declare these types because they cross-depend on pointers to each
 * other.
 */
struct pvr_page_table_l2;
struct pvr_page_table_l1;
struct pvr_page_table_l0;

/**
 * struct pvr_page_table_ptr - A reference to a single physical page as indexed
 *                             by the page table structure.
 * @pvr_dev: The PowerVR device associated with the VM context the
 *           pointer is traversing.
 * @l1_free_list: List of level 1 page tables to free when the pointer is destroyed.
 * @l0_free_list: List of level 0 page tables to free when the pointer is destroyed.
 * @l2_table: A cached handle to the level 2 page table the pointer is
 *            currently traversing.
 * @l1_table: A cached handle to the level 1 page table the pointer is
 *            currently traversing.
 * @l0_table: A cached handle to the level 0 page table the pointer is
 *            currently traversing.
 * @l2_idx: Index into the level 2 page table the pointer is currently
 *          referencing.
 * @l1_idx: Index into the level 1 page table the pointer is currently
 *          referencing.
 * @l0_idx: Index into the level 0 page table the pointer is currently
 *          referencing.
 * @sync_level_required: The maximum level of the page table tree structure
 *                       which has (possibly) been modified since it was last
 *                       flushed to the device.
 *
 *                       This field should only be set with
 *                       pvr_page_table_ptr_require_sync() or indirectly by
 *                       pvr_page_table_ptr_sync_partial().
 */
struct pvr_page_table_ptr {
	struct pvr_device *pvr_dev;
	struct pvr_page_table_l1 *l1_free_list;
	struct pvr_page_table_l0 *l0_free_list;
	struct pvr_page_table_l2 *l2_table;
	struct pvr_page_table_l1 *l1_table;
	struct pvr_page_table_l0 *l0_table;
	u16 l2_idx;
	u16 l1_idx;
	u16 l0_idx;
	s8 sync_level_required;
};

/**
 * struct pvr_page_table_l2 - A wrapped level 2 page table.
 *
 * To access the raw part of this table, use pvr_page_table_l2_get_raw().
 * Alternatively to access a raw entry directly, use
 * pvr_page_table_l2_get_entry_raw().
 *
 * A level 2 page table forms the root of the page table tree structure, so
 * this type has no &parent or &parent_idx members.
 */
struct pvr_page_table_l2 {
	/**
	 * @entries: The children of this node in the page table tree
	 * structure. These are also mirror tables. The indexing of this array
	 * is identical to that of the raw equivalent
	 * (&pvr_page_table_l1_raw.entries).
	 */
	struct pvr_page_table_l1 *entries[ROGUE_MMUCTRL_ENTRIES_PC_VALUE];

	/**
	 * @backing_page: A handle to the memory which holds the raw
	 * equivalent of this table. **For internal use only.**
	 */
	struct pvr_mmu_backing_page backing_page;

	/**
	 * @entry_count: The current number of valid entries (that we know of)
	 * in this table. This value is essentially a refcount - the table is
	 * destroyed when this value is decremented to zero by
	 * pvr_page_table_l2_remove().
	 */
	u16 entry_count;
};

/**
 * pvr_page_table_l2_init() - Initialize a level 2 page table.
 * @table: Target level 2 page table.
 * @pvr_dev: Target PowerVR device
 *
 * It is expected that @table be zeroed (e.g. from kzalloc()) before calling
 * this function.
 *
 * Return:
 *  * 0 on success, or
 *  * Any error encountered while intializing &table->backing_page using
 *    pvr_mmu_backing_page_init().
 */
static __always_inline int
pvr_page_table_l2_init(struct pvr_page_table_l2 *table,
		       struct pvr_device *pvr_dev)
{
	return pvr_mmu_backing_page_init(&table->backing_page, pvr_dev);
}

/**
 * pvr_page_table_l2_fini() - Teardown a level 2 page table.
 * @table: Target level 2 page table.
 *
 * It is an error to attempt to use @table after calling this function.
 */
static __always_inline void
pvr_page_table_l2_fini(struct pvr_page_table_l2 *table)
{
	pvr_mmu_backing_page_fini(&table->backing_page);
}

/**
 * pvr_page_table_l2_sync() - Flush a level 2 page table from the CPU to the
 *                            device.
 * @table: Target level 2 page table.
 *
 * This is just a thin wrapper around pvr_mmu_backing_page_sync(), so the
 * warning there applies here too: **Only call pvr_page_table_l2_sync() once
 * you're sure you have no more changes to make to** @table **in the immediate
 * future.**
 *
 * If child level 1 page tables of @table also need to be flushed, this should
 * be done first using pvr_page_table_l1_sync() *before* calling this function.
 */
static __always_inline void
pvr_page_table_l2_sync(struct pvr_page_table_l2 *table)
{
	pvr_mmu_backing_page_sync(&table->backing_page);
}

/**
 * pvr_page_table_l2_get_raw() - Access the raw equivalent of a mirror level 2
 *                               page table.
 * @table: Target level 2 page table.
 *
 * Essentially returns the CPU address of the raw equivalent of @table, cast to
 * a &struct pvr_page_table_l2_raw pointer.
 *
 * You probably want to call pvr_page_table_l2_get_entry_raw() instead.
 *
 * Return:
 * The raw equivalent of @table.
 */
static __always_inline struct pvr_page_table_l2_raw *
pvr_page_table_l2_get_raw(struct pvr_page_table_l2 *table)
{
	return table->backing_page.host_ptr;
}

/**
 * pvr_page_table_l2_get_entry_raw() - Access an entry from the raw equivalent
 *                                     of a mirror level 2 page table.
 * @table: Target level 2 page table.
 * @idx: Index of the entry to access.
 *
 * Technically this function returns a pointer to a slot in a raw level 2 page
 * table, since the returned "entry" is not guaranteed to be valid. The caller
 * must verify the validity of the entry at the returned address (perhaps using
 * pvr_page_table_l2_entry_raw_is_valid()) before reading or overwriting it.
 *
 * The value of @idx is not checked here; it is the callers responsibility to
 * ensure @idx refers to a valid index within @table before dereferencing the
 * returned pointer.
 *
 * Return:
 * A pointer to the requested raw level 2 page table entry.
 */
static __always_inline struct pvr_page_table_l2_entry_raw *
pvr_page_table_l2_get_entry_raw(struct pvr_page_table_l2 *table, u16 idx)
{
	return &pvr_page_table_l2_get_raw(table)->entries[idx];
}

/**
 * pvr_page_table_l2_entry_is_valid() - Check if a level 2 page table entry is
 *                                      marked as valid.
 * @table: Target level 2 page table.
 * @idx: Index of the entry to check.
 *
 * The value of @idx is not checked here; it is the callers responsibility to
 * ensure @idx refers to a valid index within @table before calling this
 * function.
 */
static __always_inline bool
pvr_page_table_l2_entry_is_valid(struct pvr_page_table_l2 *table, u16 idx)
{
	struct pvr_page_table_l2_entry_raw entry_raw =
		*pvr_page_table_l2_get_entry_raw(table, idx);

	return pvr_page_table_l2_entry_raw_is_valid(entry_raw);
}

/**
 * struct pvr_page_table_l1 - A wrapped level 1 page table.
 *
 * To access the raw part of this table, use pvr_page_table_l1_get_raw().
 * Alternatively to access a raw entry directly, use
 * pvr_page_table_l1_get_entry_raw().
 */
struct pvr_page_table_l1 {
	/**
	 * @entries: The children of this node in the page table tree
	 * structure. These are also mirror tables. The indexing of this array
	 * is identical to that of the raw equivalent
	 * (&pvr_page_table_l0_raw.entries).
	 */
	struct pvr_page_table_l0 *entries[ROGUE_MMUCTRL_ENTRIES_PD_VALUE];

	/**
	 * @backing_page: A handle to the memory which holds the raw
	 * equivalent of this table. **For internal use only.**
	 */
	struct pvr_mmu_backing_page backing_page;

	union {
		/**
		 * @parent: The parent of this node in the page table tree structure.
		 *
		 * This is also a mirror table.
		 *
		 * Only valid when the L1 page table is active. When the L1 page table
		 * has been removed and queued for destruction, the next_free field
		 * should be used instead.
		 */
		struct pvr_page_table_l2 *parent;

		/**
		 * @next_free: Pointer to the next L1 page table to free.
		 *
		 * Used to form a linked list of L1 page tables queued for destruction.
		 * Only valid when the page table has been removed and queued for
		 * destruction.
		 */
		struct pvr_page_table_l1 *next_free;
	};

	/**
	 * @parent_idx: The index of the entry in the parent table (see
	 * @parent) which corresponds to this table.
	 */
	u16 parent_idx;

	/**
	 * @entry_count: The current number of valid entries (that we know of)
	 * in this table. This value is essentially a refcount - the table is
	 * destroyed when this value is decremented to zero by
	 * pvr_page_table_l1_remove().
	 */
	u16 entry_count;
};

/**
 * pvr_page_table_l1_init() - Initialize a level 1 page table.
 * @table: Target level 1 page table.
 * @pvr_dev: Target PowerVR device
 *
 * When this function returns successfully, @table is still not considered
 * valid. It must be inserted into the page table tree structure with
 * pvr_page_table_l2_insert() before it is ready for use.
 *
 * It is expected that @table be zeroed (e.g. from kzalloc()) before calling
 * this function.
 *
 * Return:
 *  * 0 on success, or
 *  * Any error encountered while intializing &table->backing_page using
 *    pvr_mmu_backing_page_init().
 */
static __always_inline int
pvr_page_table_l1_init(struct pvr_page_table_l1 *table,
		       struct pvr_device *pvr_dev)
{
	table->parent_idx = PVR_IDX_INVALID;

	return pvr_mmu_backing_page_init(&table->backing_page, pvr_dev);
}

/**
 * pvr_page_table_l1_fini() - Teardown a level 1 page table.
 * @table: Target level 1 page table.
 *
 * It is an error to attempt to use @table after calling this function, even
 * indirectly. This includes calling pvr_page_table_l2_remove(), which must
 * be called *before* pvr_page_table_l1_fini().
 */
static __always_inline void
pvr_page_table_l1_fini(struct pvr_page_table_l1 *table)
{
	pvr_mmu_backing_page_fini(&table->backing_page);
}

/**
 * pvr_page_table_l1_sync() - Flush a level 1 page table from the CPU to the
 *                            device.
 * @table: Target level 1 page table.
 *
 * This is just a thin wrapper around pvr_mmu_backing_page_sync(), so the
 * warning there applies here too: **Only call pvr_page_table_l1_sync() once
 * you're sure you have no more changes to make to** @table **in the immediate
 * future.**
 *
 * If child level 0 page tables of @table also need to be flushed, this should
 * be done first using pvr_page_table_l0_sync() *before* calling this function.
 */
static __always_inline void
pvr_page_table_l1_sync(struct pvr_page_table_l1 *table)
{
	pvr_mmu_backing_page_sync(&table->backing_page);
}

/**
 * pvr_page_table_l1_get_raw() - Access the raw equivalent of a mirror level 1
 *                               page table.
 * @table: Target level 1 page table.
 *
 * Essentially returns the CPU address of the raw equivalent of @table, cast to
 * a &struct pvr_page_table_l1_raw pointer.
 *
 * You probably want to call pvr_page_table_l1_get_entry_raw() instead.
 *
 * Return:
 * The raw equivalent of @table.
 */
static __always_inline struct pvr_page_table_l1_raw *
pvr_page_table_l1_get_raw(struct pvr_page_table_l1 *table)
{
	return table->backing_page.host_ptr;
}

/**
 * pvr_page_table_l1_get_entry_raw() - Access an entry from the raw equivalent
 *                                     of a mirror level 1 page table.
 * @table: Target level 1 page table.
 * @idx: Index of the entry to access.
 *
 * Technically this function returns a pointer to a slot in a raw level 1 page
 * table, since the returned "entry" is not guaranteed to be valid. The caller
 * must verify the validity of the entry at the returned address (perhaps using
 * pvr_page_table_l1_entry_raw_is_valid()) before reading or overwriting it.
 *
 * The value of @idx is not checked here; it is the callers responsibility to
 * ensure @idx refers to a valid index within @table before dereferencing the
 * returned pointer.
 *
 * Return:
 * A pointer to the requested raw level 1 page table entry.
 */
static __always_inline struct pvr_page_table_l1_entry_raw *
pvr_page_table_l1_get_entry_raw(struct pvr_page_table_l1 *table, u16 idx)
{
	return &pvr_page_table_l1_get_raw(table)->entries[idx];
}

/**
 * pvr_page_table_l1_entry_is_valid() - Check if a level 1 page table entry is
 *                                      marked as valid.
 * @table: Target level 1 page table.
 * @idx: Index of the entry to check.
 *
 * The value of @idx is not checked here; it is the callers responsibility to
 * ensure @idx refers to a valid index within @table before calling this
 * function.
 */
static __always_inline bool
pvr_page_table_l1_entry_is_valid(struct pvr_page_table_l1 *table, u16 idx)
{
	struct pvr_page_table_l1_entry_raw entry_raw =
		*pvr_page_table_l1_get_entry_raw(table, idx);

	return pvr_page_table_l1_entry_raw_is_valid(entry_raw);
}

/**
 * struct pvr_page_table_l0 - A wrapped level 0 page table.
 *
 * To access the raw part of this table, use pvr_page_table_l0_get_raw().
 * Alternatively to access a raw entry directly, use
 * pvr_page_table_l0_get_entry_raw().
 *
 * There is no mirror representation of an individual page, so this type has no
 * &entries member.
 */
struct pvr_page_table_l0 {
	/**
	 * @backing_page: A handle to the memory which holds the raw
	 * equivalent of this table. **For internal use only.**
	 */
	struct pvr_mmu_backing_page backing_page;

	union {
		/**
		 * @parent: The parent of this node in the page table tree structure.
		 *
		 * This is also a mirror table.
		 *
		 * Only valid when the L0 page table is active. When the L0 page table
		 * has been removed and queued for destruction, the next_free field
		 * should be used instead.
		 */
		struct pvr_page_table_l1 *parent;

		/**
		 * @next_free: Pointer to the next L0 page table to free.
		 *
		 * Used to form a linked list of L0 page tables queued for destruction.
		 *
		 * Only valid when the page table has been removed and queued for
		 * destruction.
		 */
		struct pvr_page_table_l0 *next_free;
	};

	/**
	 * @parent_idx: The index of the entry in the parent table (see
	 * @parent) which corresponds to this table.
	 */
	u16 parent_idx;

	/**
	 * @entry_count: The current number of valid entries (that we know of)
	 * in this table. This value is essentially a refcount - the table is
	 * destroyed when this value is decremented to zero by
	 * pvr_page_table_l0_remove().
	 */
	u16 entry_count;
};

/**
 * pvr_page_table_l0_init() - Initialize a level 0 page table.
 * @table: Target level 0 page table.
 * @pvr_dev: Target PowerVR device
 *
 * When this function returns successfully, @table is still not considered
 * valid. It must be inserted into the page table tree structure with
 * pvr_page_table_l1_insert() before it is ready for use.
 *
 * It is expected that @table be zeroed (e.g. from kzalloc()) before calling
 * this function.
 *
 * Return:
 *  * 0 on success, or
 *  * Any error encountered while intializing &table->backing_page using
 *    pvr_mmu_backing_page_init().
 */
static __always_inline int
pvr_page_table_l0_init(struct pvr_page_table_l0 *table,
		       struct pvr_device *pvr_dev)
{
	table->parent_idx = PVR_IDX_INVALID;

	return pvr_mmu_backing_page_init(&table->backing_page, pvr_dev);
}

/**
 * pvr_page_table_l0_fini() - Teardown a level 0 page table.
 * @table: Target level 0 page table.
 *
 * It is an error to attempt to use @table after calling this function, even
 * indirectly. This includes calling pvr_page_table_l1_remove(), which must
 * be called *before* pvr_page_table_l0_fini().
 */
static __always_inline void
pvr_page_table_l0_fini(struct pvr_page_table_l0 *table)
{
	pvr_mmu_backing_page_fini(&table->backing_page);
}

/**
 * pvr_page_table_l0_sync() - Flush a level 0 page table from the CPU to the
 *                            device.
 * @table: Target level 0 page table.
 *
 * This is just a thin wrapper around pvr_mmu_backing_page_sync(), so the
 * warning there applies here too: **Only call pvr_page_table_l0_sync() once
 * you're sure you have no more changes to make to** @table **in the immediate
 * future.**
 *
 * If child pages of @table also need to be flushed, this should be done first
 * using a DMA sync function (e.g. dma_sync_sg_for_device()) *before* calling
 * this function.
 */
static __always_inline void
pvr_page_table_l0_sync(struct pvr_page_table_l0 *table)
{
	pvr_mmu_backing_page_sync(&table->backing_page);
}

/**
 * pvr_page_table_l0_get_raw() - Access the raw equivalent of a mirror level 0
 *                               page table.
 * @table: Target level 0 page table.
 *
 * Essentially returns the CPU address of the raw equivalent of @table, cast to
 * a &struct pvr_page_table_l0_raw pointer.
 *
 * You probably want to call pvr_page_table_l0_get_entry_raw() instead.
 *
 * Return:
 * The raw equivalent of @table.
 */
static __always_inline struct pvr_page_table_l0_raw *
pvr_page_table_l0_get_raw(struct pvr_page_table_l0 *table)
{
	return table->backing_page.host_ptr;
}

/**
 * pvr_page_table_l0_get_entry_raw() - Access an entry from the raw equivalent
 *                                     of a mirror level 0 page table.
 * @table: Target level 0 page table.
 * @idx: Index of the entry to access.
 *
 * Technically this function returns a pointer to a slot in a raw level 0 page
 * table, since the returned "entry" is not guaranteed to be valid. The caller
 * must verify the validity of the entry at the returned address (perhaps using
 * pvr_page_table_l0_entry_raw_is_valid()) before reading or overwriting it.
 *
 * The value of @idx is not checked here; it is the callers responsibility to
 * ensure @idx refers to a valid index within @table before dereferencing the
 * returned pointer. This is espcially important for level 0 page tables, which
 * can have a variable number of entries.
 *
 * Return:
 * A pointer to the requested raw level 0 page table entry.
 */
static __always_inline struct pvr_page_table_l0_entry_raw *
pvr_page_table_l0_get_entry_raw(struct pvr_page_table_l0 *table, u16 idx)
{
	return &pvr_page_table_l0_get_raw(table)->entries[idx];
}

/**
 * pvr_page_table_l0_entry_is_valid() - Check if a level 0 page table entry is
 *                                      marked as valid.
 * @table: Target level 0 page table.
 * @idx: Index of the entry to check.
 *
 * The value of @idx is not checked here; it is the callers responsibility to
 * ensure @idx refers to a valid index within @table before calling this
 * function.
 */
static __always_inline bool
pvr_page_table_l0_entry_is_valid(struct pvr_page_table_l0 *table, u16 idx)
{
	struct pvr_page_table_l0_entry_raw entry_raw =
		*pvr_page_table_l0_get_entry_raw(table, idx);

	return pvr_page_table_l0_entry_raw_is_valid(entry_raw);
}

/**
 * pvr_page_table_l2_insert() - Insert an entry referring to a level 1 page
 *                              table into a level 2 page table.
 * @ptr: Page table pointer pointing to the entry to insert the L1 page table into.
 * @child_table: Target level 1 page table to be referenced by the new entry.
 *
 * The values of @ptr are not checked here; it is the callers responsibility to
 * ensure @ptr points to a valid L2 entry.
 *
 * This function is unchecked. Do not call it unless you're absolutely sure
 * there is not already a valid entry at @ptr.
 */
static void
pvr_page_table_l2_insert(struct pvr_page_table_ptr *ptr,
			 struct pvr_page_table_l1 *child_table)
{
	struct pvr_page_table_l2_entry_raw *entry_raw =
		pvr_page_table_l2_get_entry_raw(ptr->l2_table, ptr->l2_idx);

	pvr_page_table_l2_entry_raw_set(entry_raw,
					child_table->backing_page.dma_addr);

	child_table->parent = ptr->l2_table;
	child_table->parent_idx = ptr->l2_idx;
	ptr->l2_table->entries[ptr->l2_idx] = child_table;
	++ptr->l2_table->entry_count;
	ptr->l1_table = child_table;
}

/**
 * pvr_page_table_l2_remove() - Remove a level 1 page table from a level 2 page
 *                              table.
 * @ptr: Page table pointer pointing to the L2 entry to remove.
 *
 * The values of @ptr are not checked here; it is the callers responsibility to
 * ensure @ptr points to a valid L2 entry before calling this function.
 *
 * This function is unchecked. Do not call it unless you're absolutely sure
 * there is a valid entry pointed by @ptr. It is **not** a no-op to call this
 * function twice, and subsequent calls **will** place @table into an invalid
 * state.
 */
static void
pvr_page_table_l2_remove(struct pvr_page_table_ptr *ptr)
{
	struct pvr_page_table_l2_entry_raw *entry_raw =
		pvr_page_table_l2_get_entry_raw(ptr->l2_table, ptr->l1_table->parent_idx);

	WARN_ON(ptr->l1_table->parent != ptr->l2_table);

	pvr_page_table_l2_entry_raw_clear(entry_raw);

	ptr->l2_table->entries[ptr->l1_table->parent_idx] = NULL;
	ptr->l1_table->parent_idx = PVR_IDX_INVALID;
	ptr->l1_table->next_free = ptr->l1_free_list;
	ptr->l1_free_list = ptr->l1_table;
	ptr->l1_table = NULL;

	--ptr->l2_table->entry_count;
}

/**
 * pvr_page_table_l1_insert() - Insert an entry referring to a level 0 page
 *                              table into a level 1 page table.
 * @ptr: Page table pointer pointing to the entry to insert the L0 page table into.
 * @child_table: L0 page table to insert.
 *
 * The value of @ptr is not checked here; it is the callers responsibility to
 * ensure @ptr points to valid L1 entry before calling this function.
 *
 * This function is unchecked. Do not call it unless you're absolutely sure
 * there is not already a valid entry at @ptr.
 */
static void
pvr_page_table_l1_insert(struct pvr_page_table_ptr *ptr,
			 struct pvr_page_table_l0 *child_table)
{
	struct pvr_page_table_l1_entry_raw *entry_raw =
		pvr_page_table_l1_get_entry_raw(ptr->l1_table, ptr->l1_idx);

	pvr_page_table_l1_entry_raw_set(entry_raw,
					child_table->backing_page.dma_addr);

	child_table->parent = ptr->l1_table;
	child_table->parent_idx = ptr->l1_idx;
	ptr->l1_table->entries[ptr->l1_idx] = child_table;
	++ptr->l1_table->entry_count;
	ptr->l0_table = child_table;
}

/**
 * pvr_page_table_l1_remove() - Remove a level 0 page table from a level 1 page
 *                              table.
 * @ptr: Page table pointer pointing to the L1 entry to remove.
 *
 * If this function results in the L1 table becoming empty, it will be removed
 * from its parent level 2 page table and destroyed.
 *
 * The values of @ptr are not checked here; it is the callers responsibility to
 * ensure @ptr points to a valid L1 entry before calling this function.
 *
 * This function is unchecked. Do not call it unless you're absolutely sure
 * there is a valid entry pointed by @ptr. It is **not** a no-op to call this
 * function twice, and subsequent calls **will** place @table into an invalid
 * state.
 */
static void
pvr_page_table_l1_remove(struct pvr_page_table_ptr *ptr)
{
	struct pvr_page_table_l1_entry_raw *entry_raw =
		pvr_page_table_l1_get_entry_raw(ptr->l0_table->parent,
						ptr->l0_table->parent_idx);

	WARN_ON(ptr->l0_table->parent != ptr->l1_table);

	pvr_page_table_l1_entry_raw_clear(entry_raw);

	ptr->l1_table->entries[ptr->l0_table->parent_idx] = NULL;
	ptr->l0_table->parent_idx = PVR_IDX_INVALID;
	ptr->l0_table->next_free = ptr->l0_free_list;
	ptr->l0_free_list = ptr->l0_table;
	ptr->l0_table = NULL;

	if (--ptr->l1_table->entry_count == 0) {
		/* Clear the parent L2 page table entry. */
		if (ptr->l1_table->parent_idx != PVR_IDX_INVALID)
			pvr_page_table_l2_remove(ptr);
	}
}

/**
 * pvr_page_table_l0_insert() - Insert an entry referring to a physical page
 *                              into a level 0 page table.
 * @ptr: Page table pointer pointing to the L0 entry to insert.
 * @dma_addr: Target DMA address to be referenced by the new entry.
 * @flags: Page options to be stored in the new entry.
 *
 * The values of @ptr are not checked here; it is the callers responsibility to
 * ensure @ptr points to a valid L0 entry before calling this function.
 *
 * This function is unchecked. Do not call it unless you're absolutely sure
 * there is not already a valid entry in the L0 table @ptr points to.
 */
static void
pvr_page_table_l0_insert(struct pvr_page_table_ptr *ptr,
			 dma_addr_t dma_addr, struct pvr_page_flags_raw flags)
{
	struct pvr_page_table_l0_entry_raw *entry_raw =
		pvr_page_table_l0_get_entry_raw(ptr->l0_table, ptr->l0_idx);

	pvr_page_table_l0_entry_raw_set(entry_raw, dma_addr, flags);

	/*
	 * There is no entry to set here - we don't keep a mirror of
	 * individual pages.
	 */

	++ptr->l0_table->entry_count;
}

/**
 * pvr_page_table_l0_remove() - Remove a physical page from a level 0 page
 *                              table.
 * @ptr: Page table pointer pointing to the L0 entry to remove.
 *
 * If this function results in the L0 table becoming empty, it will be removed
 * from its parent L1 page table and destroyed.
 *
 * The values of @ptr are not checked here; it is the callers responsibility to
 * ensure @ptr points to a valid L0 entry before calling this function.
 *
 * This function is unchecked. Do not call it unless you're absolutely sure
 * there is a valid entry pointed by @ptr. It is **not** a no-op to call this
 * function twice, and subsequent calls **will** place @table into an invalid
 * state.
 */
static void
pvr_page_table_l0_remove(struct pvr_page_table_ptr *ptr)
{
	struct pvr_page_table_l0_entry_raw *entry_raw =
		pvr_page_table_l0_get_entry_raw(ptr->l0_table, ptr->l0_idx);

	pvr_page_table_l0_entry_raw_clear(entry_raw);

	/*
	 * There is no entry to clear here - we don't keep a mirror of
	 * individual pages.
	 */

	if (--ptr->l0_table->entry_count == 0) {
		/* Clear the parent L1 page table entry. */
		if (ptr->l0_table->parent_idx != PVR_IDX_INVALID)
			pvr_page_table_l1_remove(ptr);
	}
}

/**
 * DOC: Page table index utilities
 */

/**
 * pvr_page_table_l2_idx() - Calculate the level 2 page table index for a
 *                           device-virtual address.
 * @device_addr: Target device-virtual address.
 *
 * This function does not perform any bounds checking - it is the caller's
 * responsibility to ensure that @device_addr is valid before interpreting
 * the result.
 *
 * Return:
 * The index into a level 2 page table corresponding to @device_addr.
 */
static __always_inline u16
pvr_page_table_l2_idx(u64 device_addr)
{
	return (device_addr & ~ROGUE_MMUCTRL_VADDR_PC_INDEX_CLRMSK) >>
	       ROGUE_MMUCTRL_VADDR_PC_INDEX_SHIFT;
}

/**
 * pvr_page_table_l1_idx() - Calculate the level 1 page table index for a
 *                           device-virtual address.
 * @device_addr: Target device-virtual address.
 *
 * This function does not perform any bounds checking - it is the caller's
 * responsibility to ensure that @device_addr is valid before interpreting
 * the result.
 *
 * Return:
 * The index into a level 1 page table corresponding to @device_addr.
 */
static __always_inline u16
pvr_page_table_l1_idx(u64 device_addr)
{
	return (device_addr & ~ROGUE_MMUCTRL_VADDR_PD_INDEX_CLRMSK) >>
	       ROGUE_MMUCTRL_VADDR_PD_INDEX_SHIFT;
}

/**
 * pvr_page_table_l0_idx() - Calculate the level 0 page table index for a
 *                           device-virtual address.
 * @device_addr: Target device-virtual address.
 *
 * This function does not perform any bounds checking - it is the caller's
 * responsibility to ensure that @device_addr is valid before interpreting
 * the result.
 *
 * Return:
 * The index into a level 0 page table corresponding to @device_addr.
 */
static __always_inline u16
pvr_page_table_l0_idx(u64 device_addr)
{
	return (device_addr & ~ROGUE_MMUCTRL_VADDR_PT_INDEX_CLRMSK) >>
	       ROGUE_MMUCTRL_PAGE_X_RANGE_SHIFT;
}

/**
 * DOC: High-level page table operations
 */

/**
 * pvr_page_table_l1_create_unchecked() - Create a level 1 page table and
 *                                        insert it into a level 2 page table.
 * @ptr: Page table pointer pointing to the entry to insert the L1 page table into.
 *
 * This function is unchecked. By using it, the caller is asserting that @ptr
 * points to a valid L2 slot, and that this slot does not contain a valid entry.
 *
 * Return:
 *  * 0 on success,
 *  * -%ENOMEM if allocation of a &struct pvr_page_table_l1 fails, or
 *  * Any error encountered while initializing the new level 1 page table with
 *    pvr_page_table_l1_init().
 */
static int
pvr_page_table_l1_create_unchecked(struct pvr_page_table_ptr *ptr)
{
	struct pvr_page_table_l1 *table;
	int err;

	table = kzalloc(sizeof(*table), GFP_KERNEL);
	if (!table)
		return -ENOMEM;

	err = pvr_page_table_l1_init(table, ptr->pvr_dev);
	if (err)
		goto err_free_table;

	pvr_page_table_l2_insert(ptr, table);
	return 0;

err_free_table:
	kfree(table);
	return err;
}

/**
 * pvr_page_table_l1_get_or_create() - Retrieves (optionally creating if
 *                                     necessary) a level 1 page table from the
 *                                     specified level 2 page table entry.
 * @ptr: [IN] Page table pointer.
 * @should_create: [IN] Specifies whether new page tables should be created
 *                 when empty page table entries are encountered during
 *                 traversal.
 * @did_create: [OUT] Optional pointer to a flag which is set when
 *              @should_create is %true and new page table entries are created.
 *              In any other case, the value will not be modified.
 *
 * Return:
 *  * 0 on success, or
 *
 *    If @should_create is %false:
 *     * -%ENXIO if a level 1 page table would have been created.
 *
 *    If @should_create is %true:
 *     * Any error encountered while creating the level 1 page table with
 *       pvr_page_table_l1_create_unchecked() if one needs to be created.
 */
static int
pvr_page_table_l1_get_or_create(struct pvr_page_table_ptr *ptr,
				bool should_create, bool *did_create)
{
	int err;

	if (pvr_page_table_l2_entry_is_valid(ptr->l2_table, ptr->l2_idx)) {
		ptr->l1_table = ptr->l2_table->entries[ptr->l2_idx];
		return 0;
	}

	if (!should_create)
		return -ENXIO;

	/* Safe, because we just verified the entry does not exist yet. */
	err = pvr_page_table_l1_create_unchecked(ptr);
	if (!err && did_create)
		*did_create = true;

	return err;
}

/**
 * pvr_page_table_l0_create_unchecked() - Create a level 0 page table and
 *                                        insert it into a level 1 page table.
 * @ptr: Page table pointer pointing to the L1 entry to insert the L0 page table into.
 *
 * This function is unchecked. By using it, the caller is asserting that @ptr
 * points to a valid L1 slot, and that slot does not contain a valid entry.
 *
 * Return:
 *  * 0 on success,
 *  * -%ENOMEM if allocation of a &struct pvr_page_table_l0 fails, or
 *  * Any error encountered while initializing the new level 0 page table with
 *    pvr_page_table_l1_init().
 */
static int
pvr_page_table_l0_create_unchecked(struct pvr_page_table_ptr *ptr)
{
	struct pvr_page_table_l0 *table;
	int err;

	table = kzalloc(sizeof(*table), GFP_KERNEL);
	if (!table)
		return -ENOMEM;

	err = pvr_page_table_l0_init(table, ptr->pvr_dev);
	if (err)
		goto err_free_table;

	pvr_page_table_l1_insert(ptr, table);
	return 0;

err_free_table:
	kfree(table);
	return err;
}

/**
 * pvr_page_table_l0_get_or_create() - Retrieves (optionally creating if
 *                                     necessary) a level 0 page table from the
 *                                     specified level 1 page table entry.
 * @ptr: [IN] Page table pointer.
 * @should_create: [IN] Specifies whether new page tables should be created
 *                 when empty page table entries are encountered during
 *                 traversal.
 * @did_create: [OUT] Optional pointer to a flag which is set when
 *              @should_create is %true and new page table entries are created.
 *              In any other case, the value will not be modified.
 *
 * Return:
 *  * 0 on success,
 *  * -%ENXIO if @should_create is %false and a level 0 page table would have
 *    been created, or
 *  * Any error returned by pvr_page_table_l1_create_unchecked() if
 *    @should_create is %true and a new level 0 page table needs to be created.
 */
static int
pvr_page_table_l0_get_or_create(struct pvr_page_table_ptr *ptr,
				bool should_create, bool *did_create)
{
	int err;

	if (pvr_page_table_l1_entry_is_valid(ptr->l1_table, ptr->l1_idx)) {
		ptr->l0_table = ptr->l1_table->entries[ptr->l1_idx];
		return 0;
	}

	if (!should_create)
		return -ENXIO;

	/* Safe, because we just verified the entry does not exist yet. */
	err = pvr_page_table_l0_create_unchecked(ptr);
	if (!err && did_create)
		*did_create = true;

	return err;
}

/**
 * DOC: MMU context
 */

struct pvr_mmu_context {
	struct pvr_device *pvr_dev;
	struct pvr_page_table_l2 page_table_l2;
};

struct pvr_mmu_context *pvr_mmu_context_create(struct pvr_device *pvr_dev)
{
	struct pvr_mmu_context *ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	int err;

	if (!ctx)
		return ERR_PTR(-ENOMEM);

	err = pvr_page_table_l2_init(&ctx->page_table_l2, pvr_dev);
	if (err)
		return ERR_PTR(err);

	ctx->pvr_dev = pvr_dev;

	return ctx;
}

void pvr_mmu_context_destroy(struct pvr_mmu_context *ctx)
{
	pvr_page_table_l2_fini(&ctx->page_table_l2);
	kfree(ctx);
}

/**
 * pvr_mmu_get_root_table_dma_addr() - Get the DMA address of the root of the
 * page table structure behind a VM context.
 * @root: Target MMU page table root.
 */
dma_addr_t pvr_mmu_get_root_table_dma_addr(struct pvr_mmu_context *ctx)
{
	return ctx->page_table_l2.backing_page.dma_addr;
}

/**
 * DOC: Page table pointer
 */
/**
 * DOC: Page table pointer (constants)
 *
 * .. c:macro:: PVR_PAGE_TABLE_PTR_IN_SYNC
 *
 *    Negative value to indicate that a page table pointer is fully in sync
 *    when assigned to &pvr_page_table_ptr->sync_level_required.
 */
#define PVR_PAGE_TABLE_PTR_IN_SYNC ((s8)(-1))

/**
 * pvr_page_table_ptr_require_sync() - Mark a page table pointer as requiring a
 *                                     sync operation for the referenced page
 *                                     tables up to a specified level.
 * @ptr: Target page table pointer.
 * @level: Maximum page table level for which a sync is required.
 */
static __always_inline void
pvr_page_table_ptr_require_sync(struct pvr_page_table_ptr *ptr, s8 level)
{
	if (ptr->sync_level_required < level)
		ptr->sync_level_required = level;
}

/**
 * pvr_page_table_ptr_sync_manual() - Trigger a sync of some or all of the
 *                                    page tables referenced by a page table
 *                                    pointer.
 * @ptr: Target page table pointer.
 * @level: Maximum page table level to sync.
 *
 * Do not call this function directly. Instead use
 * pvr_page_table_ptr_sync_partial() which is checked against the current
 * value of &ptr->sync_level_required as set by
 * pvr_page_table_ptr_require_sync().
 */
static void
pvr_page_table_ptr_sync_manual(struct pvr_page_table_ptr *ptr, s8 level)
{
	/*
	 * We sync the page table levels in ascending order (starting from the
	 * leaf node) to ensure consistency.
	 */

	if (level < 0)
		return;

	if (ptr->l0_table)
		pvr_page_table_l0_sync(ptr->l0_table);

	if (level < 1)
		return;

	if (ptr->l1_table)
		pvr_page_table_l1_sync(ptr->l1_table);

	if (level < 2)
		return;

	pvr_page_table_l2_sync(ptr->l2_table);
}

/**
 * pvr_page_table_ptr_sync_partial() - Trigger a sync of some or all of the
 *                                     page tables referenced by a page table
 *                                     pointer.
 * @ptr: Target page table pointer.
 * @level: Requested page table level to sync up to (inclusive).
 *
 * If @level is greater than the maximum level recorded by @ptr as requiring
 * a sync operation, only the previously recorded maximum will be used.
 *
 * Additionally, if @level is greater than or equal to the maximum level
 * recorded by @ptr as requiring a sync operation, that maximum level will be
 * reset as a full sync will be performed. This is equivalent to calling
 * pvr_page_table_ptr_sync().
 */
static void
pvr_page_table_ptr_sync_partial(struct pvr_page_table_ptr *ptr, s8 level)
{
	/*
	 * If the requested sync level is greater than or equal to the
	 * currently required sync level, we do two things:
	 *  * Don't waste time syncing levels we haven't previously marked as
	 *    requiring a sync, and
	 *  * Reset the required sync level since we are about to sync
	 *    everything that was previously marked as requiring a sync.
	 */
	if (level >= ptr->sync_level_required) {
		level = ptr->sync_level_required;
		ptr->sync_level_required = PVR_PAGE_TABLE_PTR_IN_SYNC;
	}

	pvr_page_table_ptr_sync_manual(ptr, level);
}

/**
 * pvr_page_table_ptr_sync() - Trigger a sync of every page table referenced by
 *                             a page table pointer.
 * @ptr: Target page table pointer.
 *
 * The maximum level marked internally as requiring a sync will be reset so
 * that subsequent calls to this function will be no-ops unless @ptr is
 * otherwise updated.
 */
static __always_inline void
pvr_page_table_ptr_sync(struct pvr_page_table_ptr *ptr)
{
	pvr_page_table_ptr_sync_manual(ptr, ptr->sync_level_required);

	ptr->sync_level_required = PVR_PAGE_TABLE_PTR_IN_SYNC;
}

/**
 * pvr_page_table_ptr_load_tables() - Load pointers to tables in each level of
 *                                    the page table tree structure needed to
 *                                    reference the physical page referenced by
 *                                    a page table pointer.
 * @ptr: Target page table pointer.
 * @should_create: Specifies whether new page tables should be created when
 *                 empty page table entries are encountered during traversal.
 * @load_level_required: Maximum page table level to load.
 *
 * If @should_create is %true, this function may modify the stored required
 * sync level of @ptr as new page tables are created and inserted into their
 * respective parents.
 *
 * Since there is only one root page table, it is technically incorrect to call
 * this function with a value of @load_level_required greater than or equal to
 * the root level number. However, this is not explicitly disallowed here.
 *
 * Return:
 *  * 0 on success,
 *  * Any error returned by pvr_page_table_l1_get_or_create() if
 *    @load_level_required >= 1 except -%ENXIO, or
 *  * Any error returned by pvr_page_table_l0_get_or_create() if
 *    @load_level_required >= 0 except -%ENXIO.
 */
static int
pvr_page_table_ptr_load_tables(struct pvr_page_table_ptr *ptr,
			       bool should_create, s8 load_level_required)
{
	bool did_create_l1 = false;
	bool did_create_l0 = false;
	int err;

	/* Clear tables we're about to fetch in case of error states. */
	if (load_level_required >= 1)
		ptr->l1_table = NULL;

	if (load_level_required >= 0)
		ptr->l0_table = NULL;

	/* Get or create L1 page table. */
	if (load_level_required >= 1) {
		err = pvr_page_table_l1_get_or_create(ptr, should_create, &did_create_l1);
		if (err) {
			/*
			 * If @should_create is %false and no L1 page table was
			 * found, return early but without an error. Since
			 * pvr_page_table_l1_get_or_create() can only return
			 * -%ENXIO if @should_create is %false, there is no
			 * need to check it here.
			 */
			if (err == -ENXIO)
				err = 0;

			goto err_out;
		}
	}

	/* Get or create L0 page table. */
	if (load_level_required >= 0) {
		err = pvr_page_table_l0_get_or_create(ptr, should_create, &did_create_l0);
		if (err) {
			/*
			 * If @should_create is %false and no L0 page table was
			 * found, return early but without an error. Since
			 * pvr_page_table_l0_get_or_create() can only return
			 * -%ENXIO if @should_create is %false, there is no
			 * need to check it here.
			 */
			if (err == -ENXIO)
				err = 0;

			/*
			 * At this point, an L1 page table could have been
			 * created but is now empty due to the failed attempt
			 * at creating an L0 page table. In this instance, we
			 * must remove the empty L1 page table ourselves as
			 * pvr_page_table_l1_remove() is never called as part
			 * of the error path in
			 * pvr_page_table_l0_get_or_create().
			 */
			if (did_create_l1) {
				pvr_page_table_l2_remove(ptr);
				pvr_page_table_ptr_require_sync(ptr, 2);
			}

			goto err_out;
		}
	}

	if (did_create_l1)
		pvr_page_table_ptr_require_sync(ptr, 2);
	else if (did_create_l0)
		pvr_page_table_ptr_require_sync(ptr, 1);

	return 0;

err_out:
	return err;
}

/**
 * pvr_page_table_ptr_set() - Reassign a page table pointer, syncing any
 *                            page tables previously assigned to it which are
 *                            no longer relevant.
 * @ptr: Target page table pointer.
 * @device_addr: New pointer target.
 * @should_create: Specify whether new page tables should be created when
 *                 empty page table entries are encountered during traversal.
 *
 * This function performs a full sync on the pointer, regardless of which
 * levels are modified.
 *
 * Return:
 *  * 0 on success, or
 *  * Any error returned by pvr_page_table_ptr_load_tables().
 */
static int
pvr_page_table_ptr_set(struct pvr_page_table_ptr *ptr, u64 device_addr,
		       bool should_create)
{
	pvr_page_table_ptr_sync(ptr);

	ptr->l2_idx = pvr_page_table_l2_idx(device_addr);
	ptr->l1_idx = pvr_page_table_l1_idx(device_addr);
	ptr->l0_idx = pvr_page_table_l0_idx(device_addr);

	return pvr_page_table_ptr_load_tables(ptr, should_create, 1);
}

/**
 * pvr_page_table_ptr_init() - Initialize a page table pointer.
 * @ptr: Target page table pointer.
 * @root_table: Root of the target page table tree structure.
 * @device_addr: Pointer target.
 * @should_create: Specify whether new page tables should be created when
 *                 empty page table entries are encountered during traversal.
 *
 * This function zeroes @ptr; it must not be a valid page table pointer when it
 * is called.
 *
 * Return:
 *  * 0 on success, or
 *  * Any error returned by pvr_page_table_ptr_set().
 */
static int
pvr_page_table_ptr_init(struct pvr_page_table_ptr *ptr,
			struct pvr_mmu_context *ctx, u64 device_addr,
			bool should_create)
{
	memset(ptr, 0, sizeof(*ptr));

	ptr->pvr_dev = ctx->pvr_dev;
	ptr->l2_table = &ctx->page_table_l2;
	ptr->sync_level_required = PVR_PAGE_TABLE_PTR_IN_SYNC;

	return pvr_page_table_ptr_set(ptr, device_addr, should_create);
}

/**
 * pvr_page_table_ptr_fini() - Teardown a page table pointer.
 * @ptr: Target page table pointer.
 */
static void
pvr_page_table_ptr_fini(struct pvr_page_table_ptr *ptr)
{
	bool flush_caches = ptr->sync_level_required != PVR_PAGE_TABLE_PTR_IN_SYNC;

	if (WARN_ON(!flush_caches && (ptr->l0_free_list || ptr->l1_free_list)))
		flush_caches = true;

	pvr_page_table_ptr_sync(ptr);

	if (flush_caches)
		WARN_ON(pvr_mmu_flush(ptr->pvr_dev));

	while (ptr->l0_free_list) {
		struct pvr_page_table_l0 *l0 = ptr->l0_free_list;

		ptr->l0_free_list = l0->next_free;
		pvr_page_table_l0_fini(l0);
		kfree(l0);
	}

	while (ptr->l1_free_list) {
		struct pvr_page_table_l1 *l1 = ptr->l1_free_list;

		ptr->l1_free_list = l1->next_free;
		pvr_page_table_l1_fini(l1);
		kfree(l1);
	}
}

/**
 * pvr_page_table_ptr_next_page() - Advance a page table pointer.
 * @ptr: Target page table pointer.
 * @should_create: Specify whether new page tables should be created when
 *                 empty page table entries are encountered during traversal.
 *
 * If @should_create is %false, it is the caller's responsibility to verify that
 * the state of the table references in @ptr is valid on return. If -%ENXIO is
 * returned, at least one of the table references is invalid. It should be
 * noted that @ptr as a whole will be left in a valid state if -%ENXIO is
 * returned, unlike other error codes. The caller should check which references
 * are invalid by comparing them to %NULL. Only &@ptr->l2_table is guaranteed
 * to be valid, since it represents the root of the page table tree structure.
 *
 * Return:
 *  * 0 on success,
 *  * -%EPERM if the operation would wrap at the top of the page table
 *    hierarchy,
 *  * -%ENXIO if @should_create is %false and a page table of any level would
 *    have otherwise been created, or
 *  * Any error returned while attempting to create missing page tables if
 *    @should_create is %true.
 */
static int
pvr_page_table_ptr_next_page(struct pvr_page_table_ptr *ptr, bool should_create)
{
	s8 load_level_required = PVR_PAGE_TABLE_PTR_IN_SYNC;

	if (++ptr->l0_idx != ROGUE_MMUCTRL_ENTRIES_PT_VALUE_X)
		goto load_tables;

	ptr->l0_idx = 0;
	load_level_required = 0;

	if (++ptr->l1_idx != ROGUE_MMUCTRL_ENTRIES_PD_VALUE)
		goto load_tables;

	ptr->l1_idx = 0;
	load_level_required = 1;

	if (++ptr->l2_idx != ROGUE_MMUCTRL_ENTRIES_PC_VALUE)
		goto load_tables;

	/*
	 * If the pattern continued, we would set &ptr->l2_idx to zero here.
	 * However, that would wrap the top layer of the page table hierarchy
	 * which is not a valid operation. Instead, we warn and return an
	 * error.
	 */
	WARN(true,
	     "%s(%p) attempted to loop the top of the page table hierarchy",
	     __func__, ptr);
	return -EPERM;

	/* If indices have wrapped, we need to load new tables. */
load_tables:
	/* First, flush tables which will be unloaded. */
	pvr_page_table_ptr_sync_partial(ptr, load_level_required);

	/* Then load tables from the required level down. */
	return pvr_page_table_ptr_load_tables(ptr, should_create,
					      load_level_required);
}

/**
 * pvr_page_table_ptr_copy() - Duplicate a page table pointer.
 * @dst: [OUT] New page table pointer.
 * @src: [IN] Original page table pointer.
 *
 * The pointer at @dst will be marked as "synced" so that any sync operations
 * required on @src are not duplicated.
 */
static void
pvr_page_table_ptr_copy(struct pvr_page_table_ptr *dst,
			const struct pvr_page_table_ptr *src)
{
	memcpy(dst, src, sizeof(*dst));

	/*
	 * Nothing currently in the clone requires a sync later on, since the
	 * original will handle it either when advancing or during teardown.
	 */
	dst->sync_level_required = PVR_PAGE_TABLE_PTR_IN_SYNC;

	/* The clone starts with an empty list of L1/L0 tables to free. */
	dst->l1_free_list = NULL;
	dst->l0_free_list = NULL;
}

/**
 * DOC: Single page operations
 */

/**
 * pvr_page_create() - Create a device-virtual memory page and insert it into
 * a level 0 page table.
 * @ptr: Page table pointer to the device-virtual address of the target page.
 * @dma_addr: DMA address of the physical page backing the created page.
 * @flags: Page options saved on the level 0 page table entry for reading by
 *         the device.
 *
 * Return:
 *  * 0 on success, or
 *  * -%EEXIST if the requested page already exists.
 */
static int
pvr_page_create(struct pvr_page_table_ptr *ptr, dma_addr_t dma_addr,
		struct pvr_page_flags_raw flags)
{
	/* Do not create a new page if one already exists. */
	if (pvr_page_table_l0_entry_is_valid(ptr->l0_table, ptr->l0_idx))
		return -EEXIST;

	pvr_page_table_l0_insert(ptr, dma_addr, flags);

	pvr_page_table_ptr_require_sync(ptr, 0);

	return 0;
}

/**
 * pvr_page_destroy() - Destroy a device page after removing it from its
 *                      parent level 0 page table.
 * @ptr: Page table pointer to the device-virtual address of the target page.
 */
static void
pvr_page_destroy(struct pvr_page_table_ptr *ptr)
{
	/* Do nothing if the page does not exist. */
	if (!pvr_page_table_l0_entry_is_valid(ptr->l0_table, ptr->l0_idx))
		return;

	/* Clear the parent L0 page table entry. */
	pvr_page_table_l0_remove(ptr);

	pvr_page_table_ptr_require_sync(ptr, 0);
}

/**
 * pvr_page_table_ptr_unmap() - Unmap pages from a memory context starting
 * from the entry addressed by a page table pointer.
 * @ptr: Page table pointer to the first page to unmap.
 * @nr_pages: Number of pages to unmap.
 *
 * Return:
 *  * 0 on success, or
 *  * Any error encountered while advancing @ptr with
 *    pvr_page_table_ptr_next_page() (except -%ENXIO).
 */
static int
pvr_page_table_ptr_unmap(struct pvr_page_table_ptr *ptr, u64 nr_pages)
{
	int err;

	if (nr_pages == 0)
		return 0;

	/*
	 * Destroy first page outside loop, as it doesn't require a pointer
	 * increment beforehand. If the L0 page table reference in @ptr is
	 * %NULL, there cannot be a mapped page at @ptr (so skip ahead).
	 */
	if (ptr->l0_table)
		pvr_page_destroy(ptr);

	for (u64 page = 1; page < nr_pages; ++page) {
		err = pvr_page_table_ptr_next_page(ptr, false);
		/*
		 * If the page table tree structure at @ptr is incomplete,
		 * skip ahead. We don't care about unmapping pages that
		 * cannot exist.
		 *
		 * FIXME: This could be made more efficient by jumping ahead
		 * using pvr_page_table_ptr_set().
		 */
		if (err == -ENXIO)
			continue;

		if (err)
			goto err_out;

		pvr_page_destroy(ptr);
	}

	return 0;

err_out:
	return err;
}

/**
 * pvr_mmu_unmap() - Unmap pages from a memory context.
 * @ctx: Target MMU context.
 * @device_addr: First device-virtual address to unmap.
 * @nr_pages: Number of pages to unmap.
 *
 * The total amount of device-virtual memory unmapped is
 * @nr_pages * %PVR_DEVICE_PAGE_SIZE.
 */
int pvr_mmu_unmap(struct pvr_mmu_context *ctx, u64 device_addr, u64 nr_pages)
{
	struct pvr_page_table_ptr ptr;
	int err = pvr_page_table_ptr_init(&ptr, ctx, device_addr, false);

	if (err && err != -ENXIO)
		goto err_out;

	err = pvr_page_table_ptr_unmap(&ptr, nr_pages);
	if (err)
		goto err_ptr_fini;

	err = 0;
	goto err_ptr_fini;

err_ptr_fini:
	pvr_page_table_ptr_fini(&ptr);

err_out:
	return err;
}

/**
 * pvr_mmu_map_sgl() - Map part of a scatter-gather table entry to
 * device-virtual memory.
 * @sgl: Target scatter-gather table entry.
 * @ptr: Page table pointer which points to the first page that should be
 * mapped to. This will point to the last page mapped to on return.
 * @offset: Offset into @sgl to map from. Must result in a starting address
 * from @sgl which is CPU page-aligned.
 * @size: Size of the memory to be mapped in bytes. Must be a non-zero multiple
 * of the device page size.
 * @page_flags: Page options to be applied to every device-virtual memory page
 * in the created mapping.
 *
 * Return:
 *  * 0 on success,
 *  * -%EINVAL if the range specified by @offset and @size is not completely
 *    within @sgl, or
 *  * Any error encountered while creating a page with pvr_page_create(), or
 *  * Any error encountered while advancing @ptr with
 *    pvr_page_table_ptr_next_page().
 */
static int
pvr_mmu_map_sgl(struct scatterlist *sgl, struct pvr_page_table_ptr *ptr,
		u64 offset, u64 size, struct pvr_page_flags_raw page_flags)
{
	const unsigned int pages = size >> PVR_DEVICE_PAGE_SHIFT;
	dma_addr_t dma_addr = sg_dma_address(sgl) + offset;
	const unsigned int dma_len = sg_dma_len(sgl);
	struct pvr_page_table_ptr ptr_copy;
	unsigned int page;
	int err;

	if (size > dma_len || offset > dma_len - size)
		return -EINVAL;

	/*
	 * Before progressing, save a copy of the start pointer so we can use
	 * it again if we enter an error state and have to destroy pages.
	 */
	pvr_page_table_ptr_copy(&ptr_copy, ptr);

	/*
	 * Create first page outside loop, as it doesn't require a pointer
	 * increment beforehand.
	 */
	err = pvr_page_create(ptr, dma_addr, page_flags);
	if (err)
		goto err_fini_ptr_copy;

	for (page = 1; page < pages; ++page) {
		err = pvr_page_table_ptr_next_page(ptr, true);
		if (err)
			goto err_destroy_pages;

		dma_addr += PVR_DEVICE_PAGE_SIZE;

		err = pvr_page_create(ptr, dma_addr, page_flags);
		if (err)
			goto err_destroy_pages;
	}

	err = 0;
	goto err_fini_ptr_copy;

err_destroy_pages:
	err = pvr_page_table_ptr_unmap(&ptr_copy, page);

err_fini_ptr_copy:
	pvr_page_table_ptr_fini(&ptr_copy);

	return err;
}

/**
 * pvr_mmu_map() - Map an object's virtual memory to physical memory.
 * @ctx: Target MMU context.
 * @sgt: Scatter gather table of pinned pages.
 * @sgt_offset: Offset into associate pvr_gem_object to map from. Must result
 * in a starting address which is CPU page-aligned.
 * @size: Size of memory to be mapped in bytes. Must be a non-zero multiple
 * of the device page size.
 * @flags: Flags from pvr_gem_object associated with the mapping.
 * @device_addr: Virtual device address to map to. Must be device page-aligned.
 *
 * Return:
 *  * 0 on success, or
 *  * ...
 */
int pvr_mmu_map(struct pvr_mmu_context *ctx, struct sg_table *sgt,
		u64 sgt_offset, u64 size, u64 flags, u64 device_addr)
{
	struct pvr_page_table_ptr ptr, ptr_copy;
	struct pvr_page_flags_raw flags_raw;
	struct scatterlist *sgl;
	u64 mapped_size = 0;
	unsigned int count;
	int err;

	if (!size)
		return 0;

	err = pvr_page_table_ptr_init(&ptr, ctx, device_addr, true);
	if (err)
		return -EINVAL;

	pvr_page_table_ptr_copy(&ptr_copy, &ptr);

	flags_raw = pvr_page_flags_raw_create(false, false,
					      flags & DRM_PVR_BO_DEVICE_BYPASS_CACHE,
					      flags & DRM_PVR_BO_DEVICE_PM_FW_PROTECT);

	/* Map scatter gather table */
	for_each_sgtable_dma_sg(sgt, sgl, count) {
		const size_t sgl_len = sg_dma_len(sgl);
		u64 sgl_offset, map_sgl_len;

		if (sgl_len <= sgt_offset) {
			sgt_offset -= sgl_len;
			continue;
		}

		sgl_offset = sgt_offset;
		map_sgl_len = min_t(u64, sgl_len - sgl_offset, size - mapped_size);

		err = pvr_mmu_map_sgl(sgl, &ptr, sgl_offset, map_sgl_len,
				      flags_raw);
		if (err)
			break;

		/*
		 * Flag the L0 page table as requiring a flush when the page
		 * table pointer is destroyed.
		 */
		pvr_page_table_ptr_require_sync(&ptr, 0);

		sgt_offset = 0;
		mapped_size += map_sgl_len;

		if (mapped_size >= size)
			break;

		err = pvr_page_table_ptr_next_page(&ptr, true);
		if (err)
			break;
	}

	if (err && mapped_size) {
		pvr_page_table_ptr_unmap(&ptr_copy,
					 mapped_size >> PVR_DEVICE_PAGE_SHIFT);
	}

	pvr_page_table_ptr_fini(&ptr_copy);
	pvr_page_table_ptr_fini(&ptr);

	return err;
}
