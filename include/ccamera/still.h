/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_STILL_H
#define CCAMERA_STILL_H
#include <ccamera/capture.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct cc_still_options {
    uint32_t fourcc; /* V4L2 format; 0 uses YUV420 on Pi or the current native format. */
    uint32_t width, height; /* Both 0 selects full sensor size on Pi or the current native size. */
    enum cc_vc4_profile profile; /* AUTO selects the standard Pi profile for the sensor. */
} cc_still_options;

typedef struct cc_still_configuration {
    cc_format format; /* Actual image format, dimensions and plane layout. */
    enum cc_capture_stream stream; /* Which frame in cc_capture_result holds this image. */
    enum cc_orientation orientation; /* Applied Pi orientation (EXIF); 0 on native video. */
    uint32_t buffer_count; /* Available common request slots. */
} cc_still_configuration;

/* Open a selected camera and return a stopped, ready-to-queue capture graph.
 * Unicam uses VC4 and CFE uses PiSP, with calibrated AE/AWB, sensor delays
 * and startup filtering. V4L2/UVC/VIMC deliver native video frames and use
 * device controls. All image storage is internal and mapped for CPU access.
 * The driver may adjust the requested format/size; use the returned layout.
 * NULL options selects defaults. Unsupported Pi sensors and incompatible
 * profiles return an error. On error *capture is NULL and configuration is unchanged.
 * Close with cc_capture_close. Requests and encoding belong to the caller. */
CC_API int cc_still_open(const cc_camera_info *camera, const cc_still_options *options,
                         cc_capture **capture, cc_still_configuration *configuration);
/* Resolve a discovery ID and verify the opened camera's identity. */
CC_API int cc_still_open_by_id(const char *id, const cc_still_options *options,
                               cc_capture **capture, cc_still_configuration *configuration);

#ifdef __cplusplus
}
#endif
#endif
