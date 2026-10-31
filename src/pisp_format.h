/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_PISP_FORMAT_H
#define CCAMERA_PISP_FORMAT_H
#include <ccamera/ccamera.h>
#include <ccamera/linux/pisp/pisp_common.h>
#include <stdbool.h>

struct cc_pisp_output_layout {
    struct pisp_image_format_config native;
    uint32_t planes, components, pixel_bytes, xsub, ysub, alignment;
    bool yuv, swap_uv, swap_rb;
};

struct cc_pisp_raw_layout {
    struct pisp_image_format_config native;
    uint32_t bayer_order, compression, pixel_bytes;
};

int cc_pisp_raw_format(uint32_t fourcc, struct cc_pisp_raw_layout *layout);
int cc_pisp_raw_layout(const cc_format *format, struct cc_pisp_raw_layout *layout);

struct cc_cfe_configuration;
struct pisp_fe_output_branch_config;
int cc_cfe_geometry(const struct cc_cfe_configuration *configuration, uint32_t output,
                    struct pisp_fe_output_branch_config *channel, uint32_t *enables);

int cc_pisp_prepare_output(cc_format *format);
int cc_pisp_output_layout(const cc_format *format, struct cc_pisp_output_layout *layout);
#endif
