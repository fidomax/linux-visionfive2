/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#ifndef PVR_CCB_H
#define PVR_CCB_H

#include "pvr_rogue_fwif.h"

#include <linux/mutex.h>
#include <linux/types.h>

/* Forward declaration from pvr_device.h. */
struct pvr_device;

/* Forward declaration from pvr_gem.h. */
struct pvr_fw_object;

struct pvr_ccb {
	/** @ctrl_obj: FW object representing CCB control structure. */
	struct pvr_fw_object *ctrl_obj;
	/** @ccb_obj: FW object representing CCB. */
	struct pvr_fw_object *ccb_obj;

	/** @ctrl_fw_addr: FW virtual address of CCB control structure. */
	u32 ctrl_fw_addr;
	/** @ccb_fw_addr: FW virtual address of CCB. */
	u32 ccb_fw_addr;

	/** @lock: Mutex protecting @ctrl and @ccb. */
	struct mutex lock;
	/**
	 * @ctrl: Kernel mapping of CCB control structure. @lock must be held
	 *        when accessing.
	 */
	struct rogue_fwif_ccb_ctl *ctrl;
	/** @ccb: Kernel mapping of CCB. @lock must be held when accessing. */
	void *ccb;
};

int pvr_kccb_init(struct pvr_device *pvr_dev);
void pvr_kccb_fini(struct pvr_device *pvr_dev);
int pvr_fwccb_init(struct pvr_device *pvr_dev);
void pvr_ccb_fini(struct pvr_ccb *ccb);

struct dma_fence *pvr_kccb_fence_alloc(void);
void pvr_kccb_fence_put(struct dma_fence *fence);
struct dma_fence *
pvr_kccb_reserve_slot(struct pvr_device *pvr_dev, struct dma_fence *f);
void pvr_kccb_release_slot(struct pvr_device *pvr_dev);
int pvr_kccb_send_cmd(struct pvr_device *pvr_dev,
		      struct rogue_fwif_kccb_cmd *cmd, u32 *kccb_slot);
int pvr_kccb_send_cmd_powered(struct pvr_device *pvr_dev,
			      struct rogue_fwif_kccb_cmd *cmd,
			      u32 *kccb_slot);
int pvr_kccb_send_cmd_reserved_powered(struct pvr_device *pvr_dev,
				       struct rogue_fwif_kccb_cmd *cmd,
				       u32 *kccb_slot);
int pvr_kccb_wait_for_completion(struct pvr_device *pvr_dev, u32 slot_nr, u32 timeout,
				 u32 *rtn_out);
bool pvr_kccb_is_idle(struct pvr_device *pvr_dev);

#endif /* PVR_CCB_H */
