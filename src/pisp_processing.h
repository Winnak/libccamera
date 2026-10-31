/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_PISP_PROCESSING_H
#define CCAMERA_PISP_PROCESSING_H
#include <ccamera/pisp_be.h>
#include <stdbool.h>

int cc_pisp_processing(struct pisp_be_config *config, const struct pisp_be_config *tuning,
                        uint32_t width, uint32_t height, uint32_t hardware_version, bool mono);
#endif
