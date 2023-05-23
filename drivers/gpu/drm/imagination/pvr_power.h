/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#ifndef PVR_POWER_H
#define PVR_POWER_H

#include "pvr_device.h"

#include <linux/mutex.h>
#include <linux/pm_runtime.h>

int pvr_power_init(struct pvr_device *pvr_dev);
void pvr_power_fini(struct pvr_device *pvr_dev);
bool pvr_power_is_idle(struct pvr_device *pvr_dev);

int pvr_power_request_idle(struct pvr_device *pvr_dev);
int pvr_power_request_pwr_off(struct pvr_device *pvr_dev);

int pvr_power_device_suspend(struct device *dev);
int pvr_power_device_resume(struct device *dev);
int pvr_power_device_idle(struct device *dev);

int pvr_power_reset(struct pvr_device *pvr_dev);

static __always_inline int
pvr_power_get(struct pvr_device *pvr_dev)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);

	return pm_runtime_resume_and_get(drm_dev->dev);
}

static __always_inline int
pvr_power_put(struct pvr_device *pvr_dev)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);

	return pm_runtime_put(drm_dev->dev);
}

#endif /* PVR_POWER_H */
