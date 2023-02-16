/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#ifndef __PVR_DRV_H__
#define __PVR_DRV_H__

#include <uapi/drm/pvr_drm.h>

#define PVR_DRIVER_NAME "powervr"
#define PVR_DRIVER_DESC "Imagination PowerVR Graphics"
#define PVR_DRIVER_DATE "20220211"

/*
 * Driver interface version:
 *  - 1.0: Initial interface
 */
#define PVR_DRIVER_MAJOR 1
#define PVR_DRIVER_MINOR 0
#define PVR_DRIVER_PATCHLEVEL 0

void *pvr_get_obj_array(struct drm_pvr_obj_array *in, u32 min_stride, u32 obj_size);

#define DRM_PVR_OBJ_ARRAY_GETTER(__obj_name, __last_mandatory_field) \
static __always_inline struct drm_pvr_ ## __obj_name * \
pvr_get_ ## __obj_name ## _array(struct drm_pvr_obj_array *in) \
{ \
	u32 min_stride = offsetof(struct drm_pvr_ ## __obj_name, __last_mandatory_field) + \
			 sizeof(((struct drm_pvr_ ## __obj_name *)0)->__last_mandatory_field); \
	return pvr_get_obj_array(in, min_stride, sizeof(struct drm_pvr_ ## __obj_name)); \
}

DRM_PVR_OBJ_ARRAY_GETTER(sync_op, value);

#endif /* __PVR_DRV_H__ */
