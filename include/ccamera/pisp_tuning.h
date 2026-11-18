/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_PISP_TUNING_H
#define CCAMERA_PISP_TUNING_H
#include <ccamera/vc4_tuning.h>
#include <ccamera/pisp_be.h>
#include <ccamera/linux/pisp/pisp_fe_config.h>
#ifdef __cplusplus
extern "C" {
#endif
/* Module identifiers are shared with the existing VC4 API. Calibration data
 * is pipeline-specific: PiSP never uses the VC4 tables. */
typedef enum cc_vc4_profile cc_rpi_profile;
typedef cc_vc4_profile_info cc_rpi_profile_info;
CC_API const cc_rpi_profile_info *cc_pisp_get_profile_info(cc_rpi_profile profile);
/* Looks up a PiSP profile by exact name. Output unchanged on error. */
CC_API int cc_pisp_lookup_profile(const char *name, cc_rpi_profile *profile);
CC_API int cc_pisp_select_profile(const cc_sensor_model *sensor, cc_rpi_profile override,
    cc_rpi_profile *profile);
/* Static single-exposure photo processing, no history/HDR or device I/O.
 * Live geometry is for the sensor image stream; CFE crop/downscale is excluded.
 * Native blocks include stats-only FE black subtraction and BE black/colour/
 * gamma/shading/denoise/sharpening. Red/blue gains are [0.5,4], gain [1,64],
 * temperature [2000,13000]. Outputs are unchanged on error. The native builders
 * finalize I/O, output CSC, tiles and register dirty bits. */
CC_API int cc_pisp_build_tuning(cc_rpi_profile profile, const cc_sensor_geometry *geometry,
    double temperature, double analogue_gain, double red_gain, double blue_gain,
    struct pisp_fe_config *frontend, struct pisp_be_config *backend);
/* As above, with optional measured shading. The PiSP back end resamples the
 * full-active-area grid to its native resolution. */
CC_API int cc_pisp_build_tuning_with_shading(cc_rpi_profile profile, const cc_sensor_geometry *geometry,
    double temperature, double analogue_gain, double red_gain, double blue_gain,
    const cc_rpi_lens_shading *shading, struct pisp_fe_config *frontend, struct pisp_be_config *backend);
enum cc_native_photo_features { CC_NATIVE_PHOTO_TDN=1U, CC_NATIVE_PHOTO_HDR=2U };
/* Configure native history ports before buffer allocation. Uses lossless,
 * MSB-aligned Bayer16 histories, independent of the main input packing.
 * Existing auxiliary choices are replaced. Output unchanged on failure.
 * HDR brackets use the sensor's image route at two delayed exposures. */
CC_API int cc_pisp_be_configure_native_photo(cc_pisp_be *backend,
    cc_pisp_be_configuration *configuration, uint32_t features);
#ifdef __cplusplus
}
#endif
#endif
