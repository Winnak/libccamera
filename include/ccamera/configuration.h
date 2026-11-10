/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_CONFIGURATION_H
#define CCAMERA_CONFIGURATION_H
#include <ccamera/capture.h>
#ifdef __cplusplus
extern "C" {
#endif

#define CC_CONFIGURATION_MAX_STREAMS 3U
enum cc_stream_role { CC_ROLE_RAW=0, CC_ROLE_STILL_CAPTURE, CC_ROLE_VIDEO_RECORDING, CC_ROLE_VIEWFINDER };
enum cc_configuration_status { CC_CONFIGURATION_VALID=0, CC_CONFIGURATION_ADJUSTED=1 };
typedef struct cc_stream_configuration {
    enum cc_stream_role role; /* Generation hint; validation classifies the edited format. */
    cc_format format;
    uint32_t buffer_count;
    uint32_t sensor_bit_depth; /* Raw sensor preference; 0 infers from format (compressed: 12). */
    enum cc_capture_stream stream; /* Derived result port; application order is preserved. */
} cc_stream_configuration;
typedef struct cc_pipeline_configuration {
    size_t num_streams;
    cc_stream_configuration streams[CC_CONFIGURATION_MAX_STREAMS];
    enum cc_orientation orientation; /* 0 preserves live flips; otherwise EXIF 1..8. */
    cc_sensor_format sensor; /* Optional exact size/depth; code's Bayer order follows orientation. All-zero: auto. */
    cc_sensor_format sensor_format; /* Derived actual sensor mode. */
    cc_format source_format; /* Derived capture source layout, including FE packing. */
    enum cc_orientation applied_orientation; /* Derived; 0 if live flips cannot be read in preserve mode. */
    uint32_t buffer_count; /* Derived recommendation for a common buffer pool. */
} cc_pipeline_configuration;

/* Use opened devices before cc_capture_create adopts them. Ownership stays
 * with the caller. Exactly one camera/frontend and at most one vc4/backend;
 * frontend+vc4 is unsupported. Pi processed roles require a supplied processor.
 * No device discovery, buffer allocation, IPA selection or encoding is implicit.
 * Nonnegative results are VALID or ADJUSTED; negative results are errno values.
 * All caller outputs stay unchanged on error. Serialize calls on these devices. */
CC_API int cc_pipeline_generate_configuration(const cc_capture_devices *devices,
    const enum cc_stream_role *roles, size_t count, cc_pipeline_configuration *configuration);
/* Read-only TRY negotiation. May validate while buffers/jobs exist. Empty
 * configurations are invalid; generation with zero roles returns an empty one.
 * UVC/VIMC/generic video cap the stream count to one; Pi accepts one raw and
 * two processed streams. Unknown pixel formats are adjusted where a native
 * fallback exists. Derived fields are regenerated. */
CC_API int cc_pipeline_validate_configuration(const cc_capture_devices *devices,
    cc_pipeline_configuration *configuration);
/* Validate the whole plan and check every device is unallocated before the
 * first active write. Applies source, orientation and processor configuration;
 * callers then allocate using the existing native APIs and adopt the devices.
 * A failure after active writes may have changed hardware and invalidates all
 * participating configurations; reconfigure the entire plan before allocating.
 * Outputs and ownership stay unchanged on failure. */
CC_API int cc_pipeline_configure(const cc_capture_devices *devices,
    cc_pipeline_configuration *configuration);

#ifdef __cplusplus
}
#endif
#endif
