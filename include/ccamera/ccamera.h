/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_CCAMERA_H
#define CCAMERA_CCAMERA_H

#include <stddef.h>
#include <stdint.h>

#if defined(__GNUC__) || defined(__clang__)
#define CC_API __attribute__((visibility("default")))
#else
#define CC_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define CC_MAX_PLANES 8
#define CC_PATH_MAX 256
#define CC_CAMERA_ID_MAX 1024
#define CC_FOURCC(a, b, c, d) ((uint32_t)(a) | ((uint32_t)(b) << 8) | \
                              ((uint32_t)(c) << 16) | ((uint32_t)(d) << 24))

typedef struct cc_camera cc_camera;

enum cc_pipeline {
    CC_PIPELINE_V4L2 = 0,
    CC_PIPELINE_RPI_UNICAM = 1,
    CC_PIPELINE_RPI_CFE = 2,
    CC_PIPELINE_VIMC = 3,
    CC_PIPELINE_UVC = 4
};

/* IDs describe a firmware/topology location, not a device serial number.
 * USB IDs remain stable on the same controller/port/interface; moving a camera
 * to another port changes its ID. NONE means no reliable identity is available. */
enum cc_camera_id_type {
    CC_CAMERA_ID_NONE = 0, CC_CAMERA_ID_FIRMWARE = 1, CC_CAMERA_ID_PLATFORM = 2,
    CC_CAMERA_ID_USB = 3, CC_CAMERA_ID_VIDEO = 4
};

typedef struct cc_camera_info {
    enum cc_pipeline pipeline;
    char name[128];
    char driver[32];
    char video_path[CC_PATH_MAX];
    char media_path[CC_PATH_MAX];
    uint32_t sensor_entity;
    enum cc_camera_id_type id_type;
    char id[CC_CAMERA_ID_MAX]; /* Opaque, NUL-terminated; empty when unavailable. */
} cc_camera_info;

typedef struct cc_plane_format {
    uint32_t stride;
    uint32_t size;
} cc_plane_format;

/* All format/field/colour values use the Linux V4L2 definitions. No conversion
 * or encoding occurs. Planes describe memory planes, not colour components. */
typedef struct cc_format {
    uint32_t fourcc, width, height, field;
    uint32_t colorspace, ycbcr_enc, quantization, xfer_func;
    uint32_t num_planes;
    cc_plane_format planes[CC_MAX_PLANES];
} cc_format;

typedef struct cc_format_info {
    uint32_t fourcc, flags;
    char description[32];
} cc_format_info;

typedef struct cc_size_range {
    uint32_t min_width, max_width, width_step;
    uint32_t min_height, max_height, height_step;
} cc_size_range;

/* Enumerate mbus codes and their sizes independently, using code_index and
 * size_index. CSI formats use MEDIA_BUS_FMT_* codes from Linux headers. */
typedef struct cc_sensor_mode {
    uint32_t mbus_code;
    cc_size_range sizes;
} cc_sensor_mode;

typedef struct cc_sensor_format {
    uint32_t mbus_code, width, height;
} cc_sensor_format;

typedef struct cc_sensor_mode_request {
    const uint32_t *mbus_codes; /* Accepted codes, in tie-breaking preference order. */
    size_t num_mbus_codes;
    uint32_t width, height; /* Minimum sensor output dimensions, both positive. */
    uint32_t max_width, max_height; /* Both zero for no upper bound. */
} cc_sensor_mode_request;

typedef struct cc_rpi_sensor_mode_request {
    const uint32_t *mbus_codes; /* Accepted native Bayer/mono codes; order has no effect. */
    size_t num_mbus_codes;
    uint32_t width, height; /* Desired dimensions, both positive; smaller modes may win. */
    uint32_t bit_depth; /* Desired sample depth: 8, 10, 12, 14 or 16. */
} cc_rpi_sensor_mode_request;

typedef struct cc_fraction { uint32_t numerator, denominator; } cc_fraction;
enum cc_frame_interval_type { CC_INTERVAL_DISCRETE=1, CC_INTERVAL_CONTINUOUS=2, CC_INTERVAL_STEPWISE=3 };
typedef struct cc_frame_interval_range {
    enum cc_frame_interval_type type;
    cc_fraction minimum, maximum, step; /* Seconds per frame, not frames/second. */
} cc_frame_interval_range;

typedef struct cc_plane {
    void *data;
    size_t length;
    size_t bytes_used;
    size_t data_offset;
} cc_plane;

typedef struct cc_frame {
    uint32_t index, sequence, flags, field, num_planes;
    uint64_t timestamp_ns;
    cc_plane planes[CC_MAX_PLANES];
} cc_frame;

typedef struct cc_dmabuf_plane {
    int fd;
    uint32_t length;
} cc_dmabuf_plane;

typedef struct cc_rectangle {
    int32_t x, y;
    uint32_t width, height;
} cc_rectangle;

/* EXIF tag 274 values. Rotate clockwise first, then optionally mirror
 * horizontally. The 90/270 names follow libcamera's image-memory convention. */
enum cc_orientation {
    CC_ORIENTATION_ROTATE_0 = 1, CC_ORIENTATION_ROTATE_0_MIRROR = 2,
    CC_ORIENTATION_ROTATE_180 = 3, CC_ORIENTATION_ROTATE_180_MIRROR = 4,
    CC_ORIENTATION_ROTATE_90_MIRROR = 5, CC_ORIENTATION_ROTATE_270 = 6,
    CC_ORIENTATION_ROTATE_270_MIRROR = 7, CC_ORIENTATION_ROTATE_90 = 8
};
/* Apply horizontal/vertical flips first, then transpose. Composition applies
 * its first argument followed by its second, as in libcamera's operator*. */
enum cc_transform {
    CC_TRANSFORM_IDENTITY = 0, CC_TRANSFORM_HFLIP = 1, CC_TRANSFORM_VFLIP = 2,
    CC_TRANSFORM_ROTATE_180 = 3, CC_TRANSFORM_TRANSPOSE = 4,
    CC_TRANSFORM_ROTATE_270 = 5, CC_TRANSFORM_ROTATE_90 = 6,
    CC_TRANSFORM_ROTATE_180_TRANSPOSE = 7
};
/* Pure orientation algebra. Rotation accepts signed multiples of 90 degrees,
 * normalized modulo 360. Invalid inputs return -EINVAL without changing output. */
CC_API int cc_orientation_from_rotation(int32_t angle, enum cc_orientation *orientation);
CC_API int cc_transform_compose(enum cc_transform first, enum cc_transform second, enum cc_transform *result);
CC_API int cc_transform_inverse(enum cc_transform transform, enum cc_transform *inverse);
CC_API int cc_orientation_transform(enum cc_orientation desired, enum cc_orientation base, enum cc_transform *transform);
CC_API int cc_orientation_apply(enum cc_orientation base, enum cc_transform transform, enum cc_orientation *orientation);

enum cc_camera_location {
    CC_CAMERA_LOCATION_FRONT = 0, CC_CAMERA_LOCATION_BACK = 1, CC_CAMERA_LOCATION_EXTERNAL = 2
};
enum cc_camera_cfa {
    CC_CAMERA_CFA_RGGB = 0, CC_CAMERA_CFA_GRBG = 1, CC_CAMERA_CFA_GBRG = 2,
    CC_CAMERA_CFA_BGGR = 3, CC_CAMERA_CFA_MONO = 4
};
enum cc_camera_property_flags {
    CC_CAMERA_PROPERTY_MODEL = 1U, CC_CAMERA_PROPERTY_LOCATION = 2U,
    CC_CAMERA_PROPERTY_ROTATION = 4U, CC_CAMERA_PROPERTY_PIXEL_ARRAY = 8U,
    CC_CAMERA_PROPERTY_ACTIVE_AREA = 16U, CC_CAMERA_PROPERTY_UNIT_CELL = 32U,
    CC_CAMERA_PROPERTY_CFA = 64U
};
typedef struct cc_camera_properties {
    uint32_t flags; /* Only flagged fields are available; all others are zero. */
    char model[128]; /* ASCII model, without the I2C address for recognized entity naming. */
    enum cc_camera_location location;
    int32_t rotation; /* Native mounting rotation in degrees, per V4L2; before image transforms. */
    uint32_t pixel_array_width, pixel_array_height;
    cc_rectangle active_area; /* Sensor coordinates; one active area is currently supported. */
    uint32_t unit_cell_width_nm, unit_cell_height_nm;
    enum cc_camera_cfa cfa; /* Native advertised pattern, before runtime flips. */
} cc_camera_properties;

typedef struct cc_control_info {
    uint32_t id, type, flags;
    char name[32];
    int64_t minimum, maximum, default_value;
    uint64_t step;
    uint32_t elem_size, elems, nr_of_dims, dims[4];
} cc_control_info;

typedef struct cc_control {
    uint32_t id;
    int64_t value;
    void *data;             /* For string/compound/array controls. */
    uint32_t size;          /* Payload capacity; updated by get_controls. */
} cc_control;

typedef struct cc_control_menu {
    uint32_t id, index, type; /* Native MENU or INTEGER_MENU type. */
    char name[32]; /* MENU label, NUL-terminated. Empty for INTEGER_MENU. */
    int64_t value; /* INTEGER_MENU value; zero for MENU. */
} cc_control_menu;

/* All fallible functions return 0 or a negative errno unless stated otherwise.
 * Enumeration ends with -ENOENT. Calls on one camera must be serialized by the
 * caller. There are no background threads, global managers or event loops. */
CC_API const char *cc_version(void);
CC_API int cc_enumerate(cc_camera_info **cameras, size_t *count);
CC_API void cc_camera_list_free(cc_camera_info *cameras);
/* Fresh discovery followed by an exact ID match. Device/media node numbers
 * can change. -ENOENT if absent, -ENOTUNIQ for an ambiguous ID. Output is NULL
 * on error. No identity is synthesized from /dev names or enumeration order. */
CC_API int cc_camera_open_by_id(const char *id, cc_camera **camera);
/* A nonempty info->id is revalidated against opened hardware before returning
 * the handle. A mismatch returns -ESTALE; unavailable verification fails rather
 * than opening another camera. Empty IDs retain path-based opening. */
CC_API int cc_camera_open(const cc_camera_info *info, cc_camera **camera);
/* Open a self-contained capture node. MC-centric nodes require cc_camera_open
 * with a discovered pipeline and are rejected by this convenience function. */
CC_API int cc_camera_open_video(const char *path, cc_camera **camera);
CC_API void cc_camera_close(cc_camera *camera);
CC_API const cc_camera_info *cc_camera_get_info(const cc_camera *camera);
/* Read-only normalized properties, available before configuration and during
 * capture. Sensors use control defaults, static cell sizes and CROP_BOUNDS /
 * CROP_DEFAULT, falling back to the largest advertised mode for geometry.
 * VIMC uses its largest sensor mode. UVC uses its media model (or video card),
 * largest video mode and USB removable attribute (fixed => Front, else External).
 * Sensor rotation defaults to zero; missing sensor location stays unavailable.
 * Unsupported optional queries leave fields unflagged. Unexpected driver
 * failures, malformed replies and excessive enumeration fail the entire query,
 * leaving *properties unchanged. No flips or graph changes are applied.
 * Serialize with other operations on this camera. */
CC_API int cc_camera_get_properties(cc_camera *camera, cc_camera_properties *properties);
/* Read-only upstream sensor policy: both writable HFLIP and VFLIP controls
 * are required. Unsupported flips or a required transpose adjust *orientation
 * to mounting orientation and select identity. Missing/invalid mounting rotation
 * defaults to Rotate0. Self-contained video uses Rotate0/identity. Outputs are
 * unchanged on error. No controls, routes, formats or buffers are written. */
CC_API int cc_camera_try_orientation(cc_camera *camera, enum cc_orientation *orientation, enum cc_transform *transform);
/* Live mounting orientation composed with readable sensor flip values.
 * Video without a sensor reports Rotate0. No format configuration is required. */
CC_API int cc_camera_get_orientation(cc_camera *camera, enum cc_orientation *orientation);
/* Borrowed nonblocking fd for poll/epoll; never close it or issue ioctls on it. */
CC_API int cc_camera_fd(const cc_camera *camera);

CC_API int cc_camera_enum_format(cc_camera *camera, uint32_t index, cc_format_info *format);
CC_API int cc_camera_enum_size(cc_camera *camera, uint32_t fourcc, uint32_t index, cc_size_range *size);
CC_API int cc_camera_enum_sensor_mode(cc_camera *camera, uint32_t code_index,
                                      uint32_t size_index, cc_sensor_mode *mode);
/* Read-only generic libcamera sensor selection policy: among accepted codes,
 * prefer the closest aspect ratio, then the smallest area, then code order.
 * Each advertised range contributes its maximum size; intermediate sizes are
 * not synthesized. Both dimensions must fit the request's minimum/maximum.
 * This is distinct from the RPi pipeline's weighted size/bit-depth heuristic.
 * No format, routing, links, controls or buffers are changed. Native negotiation
 * is still required. Outputs stay unchanged on error: -ENOENT if no mode fits,
 * -ENOTSUP without a sensor, -EPROTO for malformed replies, -E2BIG above 256
 * requested/advertised codes or 4096 size ranges across accepted codes.
 * Calls must be serialized with other operations on this camera. */
CC_API int cc_camera_select_sensor_mode(cc_camera *camera, const cc_sensor_mode_request *request,
                                        cc_sensor_format *format);
/* Read-only Raspberry Pi libcamera policy: weighted width/height/aspect error
 * plus 500 per bit of sample-depth difference. Oversize dimension/aspect errors
 * cost one eighth as much as undersize errors. Exact score ties prefer the
 * greater numeric MBUS code, then larger area, then width (upstream sort order).
 * Accepts native 8/10/12/14/16-bit Bayer and mono MBUS codes; unsupported request
 * codes return -ENOTSUP. Range maxima, enumeration bounds, serialization and
 * error/output guarantees are the same as cc_camera_select_sensor_mode.
 * No frame-rate filtering or graph configuration is performed. */
CC_API int cc_camera_select_rpi_sensor_mode(cc_camera *camera, const cc_rpi_sensor_mode_request *request,
                                            cc_sensor_format *format);
/* Capture-node intervals use FOURCC and output geometry. Sensor intervals use
 * MBUS codes and sensor geometry on the discovered image pad/stream. Native
 * unsupported ioctls return -ENOTSUP; enumeration ends with -ENOENT. */
CC_API int cc_camera_enum_frame_interval(cc_camera *camera,uint32_t fourcc,uint32_t width,uint32_t height,
                                        uint32_t index,cc_frame_interval_range *range);
CC_API int cc_camera_enum_sensor_frame_interval(cc_camera *camera,uint32_t mbus_code,uint32_t width,uint32_t height,
                                               uint32_t index,cc_fraction *interval);
/* Active native interval. CSI/VIMC use the sensor ioctl; self-contained video
 * uses G/S_PARM. Native support is optional: no synthesized blanking fallback.
 * Set before allocating buffers, after configuring the format. The driver may
 * adjust the request; success returns its actual rational interval. A video
 * request {0,0} restores the driver's default; sensor requests must be positive.
 * Outputs stay unchanged on error; a failed set may still have changed hardware.
 * Calls must be serialized with all other operations on the device. */
CC_API int cc_camera_get_frame_interval(cc_camera *camera,cc_fraction *interval);
CC_API int cc_camera_set_frame_interval(cc_camera *camera,cc_fraction *interval);
CC_API int cc_camera_get_format(cc_camera *camera, cc_format *format);
/* try_format leaves active hardware unchanged. configure returns the actual
 * negotiated format, and requires buffers to be freed first. Media pipelines
 * set their sensor, processing pads and links before the capture node. VIMC
 * adjusts unsupported formats/sizes to its version-dependent RGB limits. */
CC_API int cc_camera_try_format(cc_camera *camera, cc_format *format);
/* TRY negotiation plus orientation/flip prediction, without writing controls
 * or active formats. Bayer phase is predicted from live layout-changing flip
 * controls; configure_oriented still verifies the actual applied result.
 * Caller outputs stay unchanged on error. */
CC_API int cc_camera_try_format_oriented(cc_camera *camera, cc_format *format, enum cc_orientation *orientation);
CC_API int cc_camera_configure(cc_camera *camera, cc_format *format);
/* Negotiate orientation, apply sensor flips, then configure the complete raw
 * path. Bayer order can change; sample depth and packing are preserved. Native
 * format adjustment and actual orientation are returned only on full success.
 * Call before allocating image/embedded buffers. Unsupported orientation is
 * adjusted using try_orientation's policy; writable individual flips are reset
 * on fallback. Fixed read-only flips are reflected in the actual result.
 * A failed call can change hardware; reconfigure before capture. Outputs stay
 * unchanged on error. Legacy configure leaves native flip settings alone. */
CC_API int cc_camera_configure_oriented(cc_camera *camera, cc_format *format, enum cc_orientation *orientation);

/* MMAP allocation. count is in/out; a driver may change the requested count.
 * Buffers belong to the camera until free_buffers or close. */
CC_API int cc_camera_allocate_buffers(cc_camera *camera, uint32_t *count);
/* Alternative to MMAP: reserve DMABUF slots, then bind each slot to externally
 * allocated planes. Imports duplicate caller fds with CLOEXEC and remain bound
 * until replaced/free/close. Imported frame planes have data == NULL; the caller
 * owns CPU mapping and DMA-BUF CPU cache synchronization, if needed. */
CC_API int cc_camera_prepare_imports(cc_camera *camera, uint32_t *count);
CC_API int cc_camera_import_buffer(cc_camera *camera, uint32_t index,
                                   const cc_dmabuf_plane *planes, uint32_t num_planes);
CC_API int cc_camera_free_buffers(cc_camera *camera);
/* Inspect idle/dequeued buffer mappings. A queued buffer is inaccessible. */
CC_API int cc_camera_buffer(cc_camera *camera, uint32_t index, cc_frame *buffer);
/* Returns a new CLOEXEC DMA-BUF fd (export or duplicated import) or negative errno. Caller owns it and must
 * close all exported fds before freeing/reallocating capture buffers. */
CC_API int cc_camera_export_plane(cc_camera *camera, uint32_t index, uint32_t plane);
CC_API int cc_camera_queue(cc_camera *camera, uint32_t index);
CC_API int cc_camera_start(cc_camera *camera);
/* Queue at least one buffer before start. dequeue returns a view whose data is
 * valid until requeue/free/close. flags retain V4L2 error and timestamp flags.
 * timeout_ms: 0 = nonblocking (-EAGAIN), -1 = indefinite, >0 = bounded wait
 * (-ETIMEDOUT). Corrupt driver metadata returns -EPROTO after reclaiming the
 * indexed buffer; stop and restart capture on this error. */
CC_API int cc_camera_dequeue(cc_camera *camera, cc_frame *frame, int timeout_ms);
/* stop returns all queued buffers to the caller, retaining allocations. */
CC_API int cc_camera_stop(cc_camera *camera);

/* Controls use native V4L2 IDs, units and types. On CSI/VIMC cameras these refer to
 * the sensor. Changes apply immediately here; optional frame-start delay
 * scheduling is available through the capture graph in ccamera/capture.h.
 * error_index is optional and identifies a failed item. */
CC_API int cc_camera_query_control(cc_camera *camera, uint32_t id, cc_control_info *info);
/* Menu indices can have holes: unsupported/out-of-range indices return -ENOENT.
 * Non-menu controls return -ENOTSUP. Output stays unchanged on error. */
CC_API int cc_camera_query_menu(cc_camera *camera, uint32_t id, uint32_t index, cc_control_menu *entry);
/* Native lens codes, distinct from sensor controls. RPi lens association uses
 * media v2 ancillary links and opens/locks only the sensor's associated lens.
 * Self-contained V4L2 cameras use their capture node. No lens: -ENOENT/ENOTSUP.
 * Set returns the driver's acknowledged code through the in/out argument. */
CC_API int cc_camera_query_lens(cc_camera *camera, cc_control_info *info);
CC_API int cc_camera_get_lens_position(cc_camera *camera, int32_t *position);
CC_API int cc_camera_set_lens_position(cc_camera *camera, int32_t *position);
CC_API int cc_camera_next_control(cc_camera *camera, uint32_t after_id, cc_control_info *info);
CC_API int cc_camera_get_controls(cc_camera *camera, cc_control *controls, size_t count, size_t *error_index);
CC_API int cc_camera_set_controls(cc_camera *camera, const cc_control *controls, size_t count, size_t *error_index);

#ifdef __cplusplus
}
#endif
#endif
