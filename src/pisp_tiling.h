/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_PISP_TILING_H
#define CCAMERA_PISP_TILING_H
#include <ccamera/pisp_be.h>
#include "pisp_format.h"

int cc_pisp_crop(const cc_format *input, const cc_rectangle *requested, cc_rectangle *crop);
int cc_pisp_resize(struct pisp_be_config *config, const cc_rectangle crops[2],
                    const struct pisp_be_config *tuning, uint32_t branches);
int cc_pisp_tiles(struct pisp_be_tiles_config *parameters, const cc_rectangle crops[2],
                   const struct cc_pisp_output_layout layouts[2], uint32_t branches);
#endif
