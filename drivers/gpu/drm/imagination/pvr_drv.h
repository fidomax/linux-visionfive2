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

int pvr_get_uobj(u64 usr_ptr, u32 usr_size, u32 min_size, u32 obj_size, void *out);
int pvr_set_uobj(u64 usr_ptr, u32 usr_size, u32 min_size, u32 obj_size, const void *in);
void *pvr_get_uobj_array(struct drm_pvr_obj_array *in, u32 min_stride, u32 obj_size);
int pvr_set_uobj_array(const struct drm_pvr_obj_array *out, u32 min_stride, u32 obj_size,
		       const void *in);

#define DRM_PVR_OBJ_TYPE(__obj_name, __last_mandatory_field) \
static __always_inline int \
pvr_get_ ## __obj_name(u64 usr_ptr, u32 usr_size, struct drm_pvr_ ## __obj_name *out) \
{ \
	u32 min_size = offsetof(struct drm_pvr_ ## __obj_name, __last_mandatory_field) + \
		       sizeof(out->__last_mandatory_field); \
	return pvr_get_uobj(usr_ptr, usr_size, min_size, sizeof(*out), out); \
} \
static __always_inline int \
pvr_set_ ## __obj_name(u64 usr_ptr, u32 usr_size, const struct drm_pvr_ ## __obj_name *in) \
{ \
	u32 min_size = offsetof(struct drm_pvr_ ## __obj_name, __last_mandatory_field) + \
		       sizeof(in->__last_mandatory_field); \
	return pvr_set_uobj(usr_ptr, usr_size, min_size, sizeof(*in), in); \
} \
static __always_inline struct drm_pvr_ ## __obj_name * \
pvr_get_ ## __obj_name ## _array(struct drm_pvr_obj_array *in) \
{ \
	u32 min_stride = offsetof(struct drm_pvr_ ## __obj_name, __last_mandatory_field) + \
			 sizeof(((struct drm_pvr_ ## __obj_name *)0)->__last_mandatory_field); \
	return pvr_get_uobj_array(in, min_stride, sizeof(struct drm_pvr_ ## __obj_name)); \
} \
static __always_inline int \
pvr_set_ ## __obj_name ## _array(const struct drm_pvr_obj_array *out, \
				 const struct drm_pvr_ ## __obj_name *in) \
{ \
	u32 min_stride = offsetof(struct drm_pvr_ ## __obj_name, __last_mandatory_field) + \
			 sizeof(((struct drm_pvr_ ## __obj_name *)0)->__last_mandatory_field); \
	return pvr_set_uobj_array(out, min_stride, sizeof(struct drm_pvr_ ## __obj_name), in); \
}

DRM_PVR_OBJ_TYPE(sync_op, value);
DRM_PVR_OBJ_TYPE(job, hwrt);

#endif /* __PVR_DRV_H__ */
