/* SPDX-License-Identifier: LGPL-2.1-or-later AND BSD-2-Clause */
/* Sensor equations adapted from the supplied Raspberry Pi cam_helper sources.
 * Copyright (C) 2019-2024 Raspberry Pi Ltd. See LICENSES/BSD-2-Clause.txt. */
#include <ccamera/sensor.h>
#include "integer_math.h"

#include <errno.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <linux/videodev2.h>
#include <stdbool.h>
#include <string.h>

enum gain_family { NO_GAIN, LINEAR, RECIPROCAL, LOG_015, LOG_005 };
struct cc_sensor_model {
    cc_sensor_properties properties;
    enum gain_family gain_family;
    uint32_t parameter;
    bool fixed_exposure;
};
#include "sensor_models.h"
#include "sensor_gains.h"

int cc_sensor_lookup(const char *name, const cc_sensor_model **out)
{
    if (!out) return -EINVAL;
    *out = NULL;
    if (!name || !name[0]) return -EINVAL;
    size_t length = strcspn(name, " \t");
    const char *canonical = NULL;
    static const char *const aliases[] = {"imx708_wide", "imx708_noir", "imx708_wide_noir"};
    for (size_t i = 0; i < sizeof(aliases) / sizeof(aliases[0]); ++i)
        if (strlen(aliases[i]) == length && !memcmp(name, aliases[i], length)) canonical = "imx708";
    if (canonical) { name = canonical; length = strlen(canonical); }
    for (size_t i = 0; i < sizeof(models) / sizeof(models[0]); ++i)
        if (strlen(models[i].properties.model) == length && !memcmp(name, models[i].properties.model, length)) {
            *out = &models[i]; return 0;
        }
    return -ENOENT;
}
const cc_sensor_properties *cc_sensor_get_properties(const cc_sensor_model *model)
{
    return model ? &model->properties : NULL;
}
static int gain_bounds(const cc_sensor_model *m, uint32_t *low, uint32_t *high)
{
    if (!m) return -EINVAL;
    if (m->gain_family == NO_GAIN) return -ENOTSUP;
    *low = m->gain_family == LINEAR ? m->parameter : 0;
    *high = m->gain_family == LINEAR ? UINT32_MAX : m->gain_family == RECIPROCAL ? m->parameter - 1U : m->parameter;
    return 0;
}
static double decode(const cc_sensor_model *m, uint32_t code)
{
    switch (m->gain_family) {
    case LINEAR: return (double)code / m->parameter;
    case RECIPROCAL: return (double)m->parameter / (m->parameter - code);
    case LOG_015: return gain_015[code];
    case LOG_005: return gain_005[code];
    default: return 0;
    }
}
int cc_sensor_gain(const cc_sensor_model *m, uint32_t code, double *gain)
{
    if (!gain) return -EINVAL;
    uint32_t low, high;
    int ret = gain_bounds(m, &low, &high);
    if (ret < 0) return ret;
    if (code < low || code > high) return -ERANGE;
    *gain = decode(m, code);
    return 0;
}
int cc_sensor_gain_code(const cc_sensor_model *m, double gain, uint32_t *code)
{
    if (!code || !(gain >= 1.0) || gain > DBL_MAX) return -EINVAL;
    uint32_t low, high;
    int ret = gain_bounds(m, &low, &high);
    if (ret < 0) return ret;
    /* Monotone inversion also makes encode(decode(code)) stable at floating
     * point boundaries, without logarithms or unchecked float-to-int casts. */
    while (low < high) {
        uint32_t mid = low + (high - low) / 2U + 1U;
        if (decode(m, mid) <= gain) low = mid;
        else high = mid - 1U;
    }
    *code = low;
    return 0;
}
int cc_sensor_quantize_gain(const cc_sensor_model *m, const cc_control_info *range, double gain,
                            cc_control *control, double *actual)
{
    if (!range || !control || !actual || range->id != V4L2_CID_ANALOGUE_GAIN || !range->step ||
        range->minimum < 0 || range->maximum < range->minimum || range->minimum > UINT32_MAX) return -EINVAL;
    if ((range->type != V4L2_CTRL_TYPE_INTEGER && range->type != V4L2_CTRL_TYPE_INTEGER64) ||
        range->flags & V4L2_CTRL_FLAG_HAS_PAYLOAD || range->elems > 1) return -ENOTSUP;
    uint32_t lo, hi, wanted;
    int ret = gain_bounds(m, &lo, &hi);
    if (ret < 0) return ret;
    ret = cc_sensor_gain_code(m, gain, &wanted);
    if (ret < 0) return ret;
    uint64_t first = (uint64_t)range->minimum, maximum = (uint64_t)range->maximum;
    if (range->type == V4L2_CTRL_TYPE_INTEGER && maximum > INT32_MAX) maximum = INT32_MAX;
    if (maximum > hi) maximum = hi;
    if (first < lo) {
        uint64_t n = ((uint64_t)lo - first) / range->step + (((uint64_t)lo - first) % range->step != 0);
        if (n > (UINT64_MAX - first) / range->step) return -ERANGE;
        first += n * range->step;
    }
    if (first > maximum) return -ERANGE;
    maximum = first + (maximum - first) / range->step * range->step;
    uint64_t value = wanted;
    if (value < first) value = first;
    if (value > maximum) value = maximum;
    value = first + (value - first) / range->step * range->step;
    cc_control result = {.id = V4L2_CID_ANALOGUE_GAIN, .value = (int64_t)value};
    double quantized = decode(m, (uint32_t)value);
    *control = result; *actual = quantized;
    return 0;
}

static int line_period(const cc_sensor_timing *timing, uint64_t *numerator, uint64_t *denominator)
{
    if (!timing || !timing->width || !timing->height || !timing->pixel_rate || timing->pixel_rate > INT64_MAX)
        return -EINVAL;
    *numerator = ((uint64_t)timing->width + timing->hblank) * UINT64_C(1000000000);
    *denominator = timing->pixel_rate;
    return *denominator > *numerator ? -ERANGE : 0; /* Sub-ns lines cannot round-trip in this API. */
}
static int shutter_period(const cc_sensor_model *model, const cc_sensor_timing *timing,
                           uint64_t *numerator, uint64_t *denominator, uint64_t *offset)
{
    if (!model) return -EINVAL;
    if (!(model->properties.features & CC_SENSOR_EXPOSURE)) return -ENOTSUP;
    *offset = model->fixed_exposure ? 14260 : 0;
    if (model->fixed_exposure) { *numerator = 4400000; *denominator = 297; return 0; }
    return line_period(timing, numerator, denominator);
}
int cc_sensor_exposure_lines(const cc_sensor_model *model, const cc_sensor_timing *timing,
                             uint64_t exposure_ns, uint32_t *lines)
{
    if (!lines) return -EINVAL;
    uint64_t n, d, offset, value;
    int ret = shutter_period(model, timing, &n, &d, &offset);
    if (ret < 0) return ret;
    ret = cc_mul_div_u64(exposure_ns > offset ? exposure_ns - offset : 0, d, n, false, &value);
    if (ret < 0) return ret;
    if (value > UINT32_MAX) return -ERANGE;
    if (model->fixed_exposure && !value) value = 1;
    *lines = (uint32_t)value;
    return 0;
}
int cc_sensor_exposure_time(const cc_sensor_model *model, const cc_sensor_timing *timing,
                            uint32_t lines, uint64_t *exposure_ns)
{
    if (!exposure_ns) return -EINVAL;
    uint64_t n, d, offset, value;
    int ret = shutter_period(model, timing, &n, &d, &offset);
    if (ret < 0) return ret;
    if (model->fixed_exposure && !lines) lines = 1;
    ret = cc_mul_div_u64(lines, n, d, true, &value);
    if (ret < 0) return ret;
    if (value > UINT64_MAX - offset) return -ERANGE;
    *exposure_ns = value + offset;
    return 0;
}
int cc_sensor_frame_duration(const cc_sensor_timing *timing, uint64_t *duration_ns)
{
    if (!duration_ns) return -EINVAL;
    uint64_t n, d, value;
    int ret = line_period(timing, &n, &d);
    if (ret < 0) return ret;
    ret = cc_mul_div_u64((uint64_t)timing->height + timing->vblank, n, d, true, &value);
    if (!ret) *duration_ns = value;
    return ret;
}
