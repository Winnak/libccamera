/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_EMBEDDED_H
#define CCAMERA_EMBEDDED_H
#include <ccamera/sensor.h>
#include <ccamera/af.h>
#include <ccamera/cfe.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CC_SMIA_MAX_REGISTERS 64U
typedef struct cc_embedded_configuration {
    cc_format format; /* Payload geometry; legacy SENS stride is reported in bytes. */
    uint32_t bits_per_pixel; /* Received packing, independent of processed output. */
} cc_embedded_configuration;

/* Enable the discovered Pi sensor embedded stream after image configuration,
 * before buffer allocation. NULL disables it. Owns the associated metadata node
 * and receiver routing. Reconfiguring the image disables embedded capture.
 * Success returns its payload geometry/packing. Errors may require image
 * reconfiguration after partial native changes; output is unchanged on error.
 * Source allocation reserves an extra metadata buffer and may reduce returned
 * usable image slots. Image dequeue includes timestamp pairing: stale metadata
 * is recycled, a future packet is retained and a missing packet is reported as
 * absent. With valid timestamps, image and metadata sequence counters need not
 * coincide. When both timestamps are zero, matching uses native sequences with
 * wrap handling; mixed clock availability or incompatible clock types fail. */
CC_API int cc_camera_configure_embedded(cc_camera *camera, cc_embedded_configuration *configuration);
CC_API int cc_cfe_configure_embedded(cc_cfe *cfe, cc_embedded_configuration *configuration);
CC_API int cc_camera_get_embedded_configuration(cc_camera *camera, cc_embedded_configuration *configuration);
CC_API int cc_cfe_get_embedded_configuration(cc_cfe *cfe, cc_embedded_configuration *configuration);
/* Borrowed paired view, indexed by the completed image slot. -ENODATA means no
 * packet matched that image. Views expire on image requeue/release/stop/free.
 * Export returns a caller-owned CLOEXEC fd. fd() returns the metadata poll fd;
 * native graphs select the outstanding image/metadata direction automatically. */
CC_API int cc_camera_embedded_frame(cc_camera *camera, uint32_t image_index, cc_frame *frame);
CC_API int cc_camera_export_embedded_plane(cc_camera *camera, uint32_t image_index, uint32_t plane);
CC_API int cc_camera_embedded_fd(const cc_camera *camera);

typedef struct cc_smia_layout {
    uint32_t bits_per_pixel; /* 8, 10, 12 or 14; describes received byte packing. */
    uint32_t line_length_bytes; /* 0 searches for the next line-start marker. */
    uint32_t num_lines; /* 0 uses the supplied buffer length as the limit. */
} cc_smia_layout;
typedef struct cc_smia_register {
    uint16_t address;
    uint8_t value;
} cc_smia_register;

/* Stateless SMIA tagged-register reader. Addresses must be unique; values are
 * filled only after every requested register is found. Each call revalidates
 * tags/packing, without caching offsets from earlier frames. Repeated reports
 * update a value without counting the same requested address twice. Bounds apply to
 * every byte, including dummy packing bytes and transitions between lines.
 * Missing registers return -ENODATA, malformed/truncated tags -EBADMSG.
 * Input is borrowed for the call. All outputs remain unchanged on error. */
CC_API int cc_smia_parse(const cc_smia_layout *layout, const void *data, size_t size,
                         cc_smia_register *registers, size_t count);

#define CC_SENSOR_STATUS_TEMPERATURE 1U
typedef struct cc_sensor_embedded_status {
    uint32_t flags;
    uint32_t exposure_lines, analogue_gain_code;
    uint32_t line_length_pck, frame_length_lines;
    double analogue_gain;
    int32_t temperature_c; /* Present with TEMPERATURE; reference clamp [-20,80]. */
} cc_sensor_embedded_status;

/* Sony IMX477/519/708 register reports. Unlike delayed-control snapshots these
 * values come from this buffer. Exposure/frame length are the unshifted native
 * register values: the long-exposure shift is not present in embedded data.
 * Callers must retain modelled long-exposure values when that mode is active.
 * IMX219's upstream helper disables embedded reports because of unreliable
 * values; this typed helper returns -ENOTSUP for that sensor. */
CC_API int cc_sensor_parse_embedded_status(const cc_sensor_model *model,
                                           const cc_smia_layout *layout,
                                           const void *data, size_t size,
                                           cc_sensor_embedded_status *status);

#define CC_SENSOR_PDAF_ROWS 12U
#define CC_SENSOR_PDAF_COLS 16U
#define CC_SENSOR_PDAF_REGIONS (CC_SENSOR_PDAF_ROWS * CC_SENSOR_PDAF_COLS)
typedef struct cc_sensor_pdaf {
    uint32_t rows, cols;
    cc_af_pdaf regions[CC_SENSOR_PDAF_REGIONS];
} cc_sensor_pdaf;

/* IMX708 third embedded scanline, in 10/12/14-bit CSI-2 packing. Pass the start
 * and available length of that scanline, after any capture data offset. The
 * required length is 194 * (bits_per_pixel / 2) bytes. Signed phase is Q4 and
 * zero-confidence phase is cleared. rows/cols/regions can be borrowed directly
 * by cc_af_prepare through a cc_af_pdaf_grid. */
CC_API int cc_sensor_parse_pdaf(const cc_sensor_model *model, uint32_t bits_per_pixel,
                               const void *data, size_t size, cc_sensor_pdaf *pdaf);

#define CC_SENSOR_HDR_BINS 128U
typedef struct cc_sensor_hdr_statistics {
    uint32_t histogram[CC_SENSOR_HDR_BINS]; /* Linear histogram, pixel quads. */
    uint64_t counted; /* Pixels used in the linear/logarithmic mean. */
    uint32_t average; /* Linear raw mean, scaled by 65536. No HDR headroom gain. */
} cc_sensor_hdr_statistics;

/* IMX708 fourth embedded scanline, where present. Verifies both histogram
 * markers and linear/logarithmic overlap. The required length is 144 *
 * (bits_per_pixel / 2) bytes; all accumulation is 64-bit. Empty histograms
 * return -ENODATA. This is a decoder, not an HDR exposure/fusion policy. */
CC_API int cc_sensor_parse_hdr_statistics(const cc_sensor_model *model, uint32_t bits_per_pixel,
                                         const void *data, size_t size,
                                         cc_sensor_hdr_statistics *statistics);

#ifdef __cplusplus
}
#endif
#endif
