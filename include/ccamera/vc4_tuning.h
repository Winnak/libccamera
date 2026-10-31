/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_VC4_TUNING_H
#define CCAMERA_VC4_TUNING_H
#include <ccamera/linux/bcm2835-isp.h>
#include <ccamera/sensor.h>
#ifdef __cplusplus
extern "C" {
#endif
#define CC_VC4_LSC_SAMPLES 192U
typedef struct cc_sensor_geometry {
    cc_rectangle active, crop;
    uint32_t width, height, hflip, vflip;
} cc_sensor_geometry;
/* Live ACTIVE geometry; no guessed crop or flip defaults. */
CC_API int cc_camera_get_sensor_geometry(cc_camera *camera, cc_sensor_geometry *geometry);
typedef struct cc_vc4_tuning {
    struct bcm2835_isp_black_level black;
    struct bcm2835_isp_gamma gamma;
    struct bcm2835_isp_custom_ccm ccm;
    struct bcm2835_isp_geq geq;
    struct bcm2835_isp_denoise denoise;
    struct bcm2835_isp_cdn cdn;
    struct bcm2835_isp_sharpen sharpen;
    struct bcm2835_isp_dpc dpc;
    /* Centre-sampled, mode-adjusted R/G/B gains, row-major 16x12.
     * Backend converts to corner-sampled U4.10 and duplicates green. */
    double lens_shading[3][CC_VC4_LSC_SAMPLES];
    uint32_t lens_shading_enabled;
    double colour_temperature;
} cc_vc4_tuning;
/* Official standard-colour OV5647 calibration, not the NoIR variant.
 * Native black/gamma values are 16-bit-scale quantities, including 10-bit input.
 * No device I/O; output stays unchanged on error. */
CC_API int cc_vc4_ov5647_build_tuning(const cc_sensor_geometry *geometry, double temperature,
                                      double analogue_gain, cc_vc4_tuning *tuning);
CC_API int cc_vc4_validate_tuning(const cc_vc4_tuning *tuning);
CC_API const char *cc_vc4_ov5647_profile_id(void);
/* Quantize automatic shutter to whole lighting half-cycles, then native lines.
 * Below one half-cycle only native-line quantization applies. 0 disables. */
CC_API int cc_sensor_flicker_exposure(const cc_sensor_model *model, const cc_sensor_timing *timing,
                                      uint64_t wanted_ns, uint32_t mains_hz, uint64_t *actual_ns);
#ifdef __cplusplus
}
#endif
#endif
