/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_VIMC_H
#define CCAMERA_VIMC_H
#include <ccamera/ccamera.h>
#ifdef __cplusplus
extern "C" {
#endif

enum cc_vimc_control_flags { CC_VIMC_BRIGHTNESS=1U, CC_VIMC_CONTRAST=2U, CC_VIMC_SATURATION=4U };
typedef struct cc_vimc_controls {
    uint32_t flags;
    double brightness; /* [-1,1], default 0. */
    double contrast, saturation; /* [0,2], default 1. */
} cc_vimc_controls;
struct cc_request;
/* Convert normalized VIMC image controls to native V4L2 scalar values, using
 * upstream rounding/clamping. Inputs must be finite when their flag is set;
 * values outside the documented intervals saturate. No libm is required.
 * *count is capacity on input and required count on success/-ENOSPC. NULL
 * controls queries the count. Rejected input leaves outputs unchanged. */
CC_API int cc_vimc_controls_to_native(const cc_vimc_controls *settings,cc_control *controls,size_t *count);
/* Immediate sensor write on a VIMC camera, before adoption into a graph.
 * Success replaces settings with the native driver's acknowledged values in
 * normalized units. Calls must be serialized with capture; unchanged on error. */
CC_API int cc_camera_set_vimc_controls(cc_camera *camera,cc_vimc_controls *settings);
/* Copied request commands, applied once before queuing its source buffer.
 * NULL clears them; omitted controls retain previous values. VIMC graphs only.
 * Completed results retain this request's acknowledged normalized snapshot.
 * Reuse repeats the commands until cleared. No frame-sync subscription is used. */
CC_API int cc_request_set_vimc_controls(struct cc_request *request,const cc_vimc_controls *settings);

#ifdef __cplusplus
}
#endif
#endif
