/* SPDX-License-Identifier: LGPL-2.1-or-later AND BSD-2-Clause */
/* Native field selection and VC4's 13-bit scale follow the RPi IPA mappings.
 * Copyright (C) 2019-2024 Raspberry Pi Ltd. See LICENSES/BSD-2-Clause.txt. */
#include <ccamera/ipa.h>
#include <ccamera/linux/bcm2835-isp.h>
#include <ccamera/linux/pisp/pisp_fe_statistics.h>
#include <errno.h>
#include <string.h>

static void histogram(const uint32_t *bins, size_t count, cc_ipa_statistics *s)
{
    uint64_t samples = 0;
    double weighted = 0;
    for (size_t i = 0; i < count; ++i) {
        samples += bins[i]; weighted += ((double)i + 0.5) * bins[i];
    }
    if (samples) { s->luminance = weighted / ((double)samples * (double)count); s->flags |= CC_IPA_STATS_LUMA; }
}
static int region(uint64_t r, uint64_t g, uint64_t b, uint32_t count, uint32_t maximum,
                   uint64_t *red, uint64_t *green, uint64_t *blue, uint64_t *samples)
{
    if (!count) return 0;
    uint64_t limit = (uint64_t)count * maximum;
    if (r > limit || g > limit || b > limit || *red > UINT64_MAX - r || *green > UINT64_MAX - g ||
        *blue > UINT64_MAX - b || *samples > UINT64_MAX - count) return -EPROTO;
    *red += r; *green += g; *blue += b; *samples += count;
    return 0;
}
int cc_ipa_parse_statistics(uint32_t fourcc, const void *data, size_t size, cc_ipa_statistics *out)
{
    if (!data || !out) return -EINVAL;
    cc_ipa_statistics s = {0};
    uint64_t r = 0, g = 0, b = 0, samples = 0;
    uint32_t scale;
    if (fourcc == CC_FOURCC('B','S','T','A')) {
        struct bcm2835_isp_stats stats;
        if (size < sizeof(stats)) return -EMSGSIZE;
        memcpy(&stats, data, sizeof(stats));
        if (stats.size && (stats.size < sizeof(stats) || stats.size > size)) return -EPROTO;
        histogram(stats.hist[0].g_hist, NUM_HISTOGRAM_BINS, &s);
        scale = 8192;
        for (size_t i = 0; i < AWB_REGIONS; ++i) {
            const struct bcm2835_isp_stats_region *z = &stats.awb_stats[i];
            int ret = region(z->r_sum, z->g_sum, z->b_sum, z->counted, scale - 1, &r, &g, &b, &samples);
            if (ret < 0) return ret;
        }
    } else if (fourcc == CC_FOURCC('R','P','F','S')) {
        struct pisp_statistics stats;
        if (size < sizeof(stats)) return -EMSGSIZE;
        memcpy(&stats, data, sizeof(stats));
        /* Copy packed members before passing aligned histogram storage. */
        uint32_t bins[PISP_AGC_STATS_NUM_BINS];
        memcpy(bins, stats.agc.histogram, sizeof(bins));
        histogram(bins, PISP_AGC_STATS_NUM_BINS, &s);
        scale = 65536;
        for (size_t i = 0; i < PISP_AWB_STATS_NUM_ZONES; ++i) {
            const struct pisp_awb_statistics_zone *z = &stats.awb.zones[i];
            int ret = region(z->R_sum, z->G_sum, z->B_sum, z->counted, scale - 1, &r, &g, &b, &samples);
            if (ret < 0) return ret;
        }
    } else return -ENOTSUP;
    if (samples) {
        double denominator = (double)samples * scale;
        s.red = (double)r / denominator; s.green = (double)g / denominator; s.blue = (double)b / denominator;
        s.colour_samples = samples; s.flags |= CC_IPA_STATS_COLOUR;
    }
    *out = s;
    return 0;
}
