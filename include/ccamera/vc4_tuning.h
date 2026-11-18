/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_VC4_TUNING_H
#define CCAMERA_VC4_TUNING_H
#include <ccamera/linux/bcm2835-isp.h>
#include <ccamera/sensor.h>
#ifdef __cplusplus
extern "C" {
#endif
#define CC_VC4_LSC_SAMPLES 192U
/* Optional measured flat-field correction. Gains are R/G/B, row-major 16x12,
 * sampled at cell centres across the full unflipped sensor active area.
 * Each gain must be in [1, 7.99]. Zero enabled selects the profile's table. */
typedef struct cc_rpi_lens_shading {
    uint32_t enabled;
    double gains[3][CC_VC4_LSC_SAMPLES];
} cc_rpi_lens_shading;
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
/* AUTO selects the standard colour module for the sensor. Sensor identity
 * cannot establish the lens or IR filter: select other variants explicitly. */
enum cc_vc4_profile {
    CC_VC4_PROFILE_AUTO = 0,
    CC_VC4_PROFILE_OV5647, CC_VC4_PROFILE_OV5647_NOIR,
    CC_VC4_PROFILE_IMX219, CC_VC4_PROFILE_IMX219_NOIR,
    CC_VC4_PROFILE_IMX477, CC_VC4_PROFILE_IMX477_NOIR,
    CC_VC4_PROFILE_IMX708, CC_VC4_PROFILE_IMX708_NOIR,
    CC_VC4_PROFILE_IMX708_WIDE, CC_VC4_PROFILE_IMX708_WIDE_NOIR,
    CC_VC4_PROFILE_IMX477_6MM, CC_VC4_PROFILE_IMX477_16MM /* PiSP only. */
};
enum cc_vc4_profile_flags {
    CC_VC4_PROFILE_NOIR = 1U, CC_VC4_PROFILE_WIDE = 2U,
    CC_VC4_PROFILE_GENERIC_LENS = 4U, /* Factory shading is generic for HQ. */
    CC_VC4_PROFILE_LENS_CAC = 8U /* Factory chromatic aberration correction is present. */
};
typedef struct cc_vc4_profile_info {
    enum cc_vc4_profile profile;
    const char *name, *sensor_model, *calibration_id; /* Static borrowed strings. */
    uint32_t flags;
} cc_vc4_profile_info;
/* AUTO/unknown IDs have no info. Lookup uses exact names (e.g. imx708_wide).
 * These are software profiles, with no physical image-quality validation. */
CC_API const cc_vc4_profile_info *cc_vc4_get_profile_info(enum cc_vc4_profile profile);
CC_API int cc_vc4_lookup_profile(const char *name, enum cc_vc4_profile *profile);
/* Rejects a sensor/override mismatch with -ENOTSUP; outputs unchanged on error. */
CC_API int cc_vc4_select_profile(const cc_sensor_model *sensor, enum cc_vc4_profile override,
                                 enum cc_vc4_profile *profile);
/* No I/O. Requires a concrete profile, live geometry, 2000..13000 K, 1..64x.
 * Outputs unchanged on error. Shading is calibration-only, without adaptive ALSC. */
CC_API int cc_vc4_build_tuning(enum cc_vc4_profile profile, const cc_sensor_geometry *geometry,
                               double temperature, double analogue_gain, cc_vc4_tuning *tuning);
/* As above, with optional full-active-area lens shading in place of the
 * factory shading. The measured grid is resampled for the live crop/flips. */
CC_API int cc_vc4_build_tuning_with_shading(enum cc_vc4_profile profile, const cc_sensor_geometry *geometry,
    double temperature, double analogue_gain, const cc_rpi_lens_shading *shading, cc_vc4_tuning *tuning);
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
