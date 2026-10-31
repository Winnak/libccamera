/* SPDX-License-Identifier: LGPL-2.1-or-later AND BSD-2-Clause */
/* Copyright (C) 2021-2023 Raspberry Pi Ltd. C17 port of libpisp v1.5.0 tiling
 * stages and native tile finalization. See LICENSES/BSD-2-Clause.txt.
 * Stages live on the stack; no C++ objects, exceptions or dependencies remain. */
#include "pisp_tiling.h"

#include <errno.h>
#include <string.h>

enum kind { INPUT, CONTEXT, SPLIT, CROP, DOWNSCALE, RESAMPLE, OUTPUT };
struct interval { int start, end; };
struct region { struct interval input, output; int crop_start, crop_end; };
struct stage {
    enum kind kind;
    struct stage *up, *down[3];
    int children, input_size, output_size, align_min, align_max, offset, factor, starts;
    bool done, mirrored;
    struct region r;
};
struct axis_tile {
    struct region input, context, crop[2], downscale[2], resample[2], output[2];
};
struct pipeline {
    struct stage stages[13], *input, *context, *crop[3], *downscale[3], *resample[3], *output[3];
    int count, maximum, error;
    bool first;
};

static int length(struct interval r) { return r.end - r.start; }
static bool contains(struct interval outer, struct interval inner)
{
    return outer.start <= inner.start && outer.end >= inner.end;
}
static int floor_div(int64_t n, int d) { return (int)(n / d - (n % d < 0)); }
static int align_up(int n, int alignment) { return -floor_div(-(int64_t)n, alignment) * alignment; }
static int minimum(int a, int b) { return a < b ? a : b; }
static bool complete(const struct stage *s)
{
    if (s->kind == OUTPUT) return s->done;
    for (int i = 0; i < s->children; ++i) if (!complete(s->down[i])) return false;
    return true;
}
static bool inactive(const struct stage *s)
{
    return s->kind == CROP ? !length(s->r.output) : s->up && inactive(s->up);
}
static bool crop_valid(const struct stage *s) { return s->r.output.end >= 16 && length(s->r.output) >= 16; }

static struct stage *stage(struct pipeline *p, enum kind kind, struct stage *up, int input, int output)
{
    struct stage *s = &p->stages[p->count++];
    *s = (struct stage) { .kind = kind, .up = up, .input_size = input, .output_size = output };
    if (up) up->down[up->children++] = s;
    return s;
}

static void start_up(struct pipeline *p, struct stage *s, int start)
{
    if (p->error) return;
    s->r.output.start = start;
    int input = start;
    switch (s->kind) {
    case INPUT:
        s->r.input.start = floor_div(start, s->align_min) * s->align_min;
        return;
    case CONTEXT:
        input = start < 16 ? 0 : (start - 16) / 2 * 2;
        break;
    case SPLIT: {
        if (!s->starts || start < s->r.input.start) s->r.input.start = start;
        ++s->starts;
        int active = 0;
        for (int i = 0; i < s->children; ++i) active += !complete(s->down[i]);
        if (s->starts == active) { s->starts = 0; start_up(p, s->up, s->r.input.start); }
        return;
    }
    case CROP:
        input += s->offset;
        if (input < 0) { p->error = -ERANGE; return; }
        break;
    case DOWNSCALE: case RESAMPLE:
        input = floor_div((int64_t)start * s->factor, 4096) - (s->kind == RESAMPLE ? 2 : 0);
        if (p->first && input < 0) input = 0;
        break;
    case OUTPUT: break;
    }
    s->r.input.start = input;
    start_up(p, s->up, input);
}

static void end_up(struct stage *s, int end)
{
    int input = end;
    switch (s->kind) {
    case INPUT: input = minimum(align_up(end, s->align_min), s->input_size); break;
    case CONTEXT: input = minimum(align_up(end + 16, 2), s->input_size); break;
    case CROP: input += s->offset; break;
    case DOWNSCALE: input = (int)(((int64_t)end * s->factor + 4095) / 4096); break;
    case RESAMPLE: input = floor_div((int64_t)(end - 1) * s->factor, 4096) + 6; break;
    case SPLIT: return;
    case OUTPUT: break;
    }
    if (s->kind == RESAMPLE || s->kind == DOWNSCALE) input = minimum(input, s->input_size);
    s->r.output.end = end;
    s->r.input.end = input;
    if (s->kind == CROP && !crop_valid(s)) s->r = (struct region) {0};
}

static int aligned_end(const struct stage *s, int end, int alignment)
{
    if (s->mirrored) return s->output_size - align_up(s->output_size - end, alignment);
    return end < s->output_size ? floor_div(end, alignment) * alignment : end;
}

static int end_down(struct pipeline *p, struct stage *s, int input_end)
{
    if (p->error) return 0;
    int output_end = input_end;
    if (s->kind == SPLIT) {
        s->r.input.end = 0;
        for (int i = 0; i < s->children; ++i) {
            if (complete(s->down[i])) continue;
            int end = end_down(p, s->down[i], input_end);
            if (end > s->r.input.end) s->r.input.end = end;
        }
        if (s->r.input.end <= s->r.input.start) { p->error = -ERANGE; return 0; }
        for (int i = 0; i < s->children; ++i)
            if (!complete(s->down[i])) end_down(p, s->down[i], s->r.input.end);
        return s->r.input.end;
    }
    if (s->kind == INPUT) {
        input_end = input_end >= s->input_size ? s->input_size : input_end / s->align_min * s->align_min;
        output_end = input_end;
    } else if (s->kind == CONTEXT) {
        if (input_end < s->input_size) output_end = input_end / 2 * 2 - 16;
    } else if (s->kind == CROP) {
        output_end = minimum(input_end - s->offset, s->output_size);
        s->r.output.end = output_end;
        if (!crop_valid(s)) { s->r = (struct region) {0}; return 0; }
    } else if (s->kind == DOWNSCALE || s->kind == RESAMPLE) {
        if (s->kind == DOWNSCALE) output_end = (int)((int64_t)input_end * 4096 / s->factor);
        else {
            int last = input_end - 1 - (input_end < s->input_size ? 5 : 0);
            output_end = (int)(((int64_t)last * 4096 + 4095) / s->factor) + 1;
        }
        output_end = minimum(output_end, s->output_size);
        output_end = minimum(output_end, s->r.output.start + p->maximum);
    } else if (s->kind == OUTPUT) {
        int aligned = aligned_end(s, input_end, s->align_max);
        if (aligned >= s->r.output.start + s->align_max) output_end = aligned;
        else {
            aligned = aligned_end(s, input_end, s->align_min);
            if (aligned > s->r.output.start || s->r.input.start < s->input_size) output_end = aligned;
        }
    }
    s->r.input.end = input_end;
    s->r.output.end = output_end;
    end_up(s, s->kind == OUTPUT ? output_end : end_down(p, s->down[0], output_end));
    if ((s->kind == RESAMPLE || s->kind == DOWNSCALE) && s->r.output.end < s->output_size &&
        s->r.input.end > s->input_size - 16) {
        int retry = s->input_size - 16;
        if (retry >= input_end) { p->error = -ERANGE; return 0; }
        return end_down(p, s, retry);
    }
    return s->r.input.end;
}

static void crop_down(struct pipeline *p, struct stage *s, struct interval available)
{
    if (p->error) return;
    if (s->kind == CROP && !crop_valid(s)) { s->r = (struct region) {0}; return; }
    if (!contains(available, s->r.input)) { p->error = -ERANGE; return; }
    if (s->kind == SPLIT) {
        s->r.input = available;
        for (int i = 0; i < s->children; ++i)
            if (!complete(s->down[i])) crop_down(p, s->down[i], available);
        return;
    }
    if (s->kind == INPUT) {
        s->r.input = s->r.output = available;
        s->r.crop_start = s->r.crop_end = 0;
    } else if (s->kind == CONTEXT) {
        struct interval output = s->r.input;
        if (!(available.start % 2) && (!(available.end % 2) || available.end == s->input_size)) output = available;
        s->r.input = available; s->r.output = output;
        s->r.crop_start = output.start - available.start; s->r.crop_end = available.end - output.end;
    } else if (s->kind == CROP) {
        s->r.input = available;
        s->r.crop_start = s->r.output.start - (available.start - s->offset);
        s->r.crop_end = available.end - s->offset - s->r.output.end;
    } else if (s->kind == OUTPUT) {
        s->r.input = available;
        s->r.crop_start = s->r.output.start - available.start;
        s->r.crop_end = available.end - s->r.output.end;
    } else {
        s->r.crop_start = s->r.input.start - available.start;
        s->r.crop_end = available.end - s->r.input.end;
        s->r.input = available;
    }
    if (s->r.crop_start < 0 || s->r.crop_end < 0) { p->error = -ERANGE; return; }
    if (s->kind != OUTPUT) crop_down(p, s->down[0], s->r.output);
}

static struct region snapshot(struct stage *s)
{
    if (!s) return (struct region) {0};
    if (complete(s) || inactive(s)) s->r = (struct region) {0};
    return s->r;
}

static int axis(const struct pisp_be_config *c, const cc_rectangle crops[2],
                 const struct cc_pisp_output_layout layouts[2], uint32_t branches, bool vertical,
                 struct axis_tile tiles[64], int capacity)
{
    int size = vertical ? c->input_format.height : c->input_format.width;
    struct pipeline p = { .maximum = vertical ? 3072 : 640, .first = true };
    p.input = stage(&p, INPUT, NULL, size, size);
    p.input->align_min = !vertical && PISP_IMAGE_FORMAT_COMPRESSED(c->input_format.format) ? 8 : 2;
    if (!vertical) {
        if ((c->global.bayer_enables & PISP_BE_BAYER_ENABLE_TDN_INPUT && PISP_IMAGE_FORMAT_COMPRESSED(c->tdn_input_format.format)) ||
            (c->global.bayer_enables & PISP_BE_BAYER_ENABLE_STITCH_INPUT && PISP_IMAGE_FORMAT_COMPRESSED(c->stitch_input_format.format)))
            p.input->align_min = 8;
        if (c->global.bayer_enables & PISP_BE_BAYER_ENABLE_TDN_OUTPUT)
            p.input->align_min = PISP_IMAGE_FORMAT_COMPRESSED(c->tdn_output_format.format) ? 16 : p.input->align_min < 8 ? 8 : p.input->align_min;
        if (c->global.bayer_enables & PISP_BE_BAYER_ENABLE_STITCH_OUTPUT)
            p.input->align_min = PISP_IMAGE_FORMAT_COMPRESSED(c->stitch_output_format.format) ? 16 : p.input->align_min < 8 ? 8 : p.input->align_min;
    }
    p.context = stage(&p, CONTEXT, p.input, size, size);
    struct stage *split = stage(&p, SPLIT, p.context, size, size);
    for (uint32_t i = 0; i < branches; ++i) {
        int cropped = (int)(vertical ? crops[i].height : crops[i].width);
        p.crop[i] = stage(&p, CROP, split, size, cropped);
        p.crop[i]->offset = vertical ? crops[i].y : crops[i].x;
        struct stage *previous = p.crop[i];
        int scaled = cropped;
        if (c->global.rgb_enables & (uint32_t)PISP_BE_RGB_ENABLE_DOWNSCALE(i)) {
            scaled = vertical ? c->downscale_extra[i].scaled_height : c->downscale_extra[i].scaled_width;
            p.downscale[i] = stage(&p, DOWNSCALE, previous, cropped, scaled);
            p.downscale[i]->factor = vertical ? c->downscale[i].scale_factor_v : c->downscale[i].scale_factor_h;
            previous = p.downscale[i];
        }
        int output = vertical ? c->output_format[i].image.height : c->output_format[i].image.width;
        if (c->global.rgb_enables & (uint32_t)PISP_BE_RGB_ENABLE_RESAMPLE(i)) {
            p.resample[i] = stage(&p, RESAMPLE, previous, scaled, output);
            p.resample[i]->factor = vertical ? c->resample[i].scale_factor_v : c->resample[i].scale_factor_h;
            previous = p.resample[i];
        }
        p.output[i] = stage(&p, OUTPUT, previous, output, output);
        p.output[i]->align_max = (int)(vertical ? layouts[i].ysub : layouts[i].alignment);
        p.output[i]->align_min = vertical ? (int)layouts[i].ysub : (int)layouts[i].alignment / 4;
        p.output[i]->mirrored = !vertical && (c->output_format[i].transform & PISP_BE_TRANSFORM_HFLIP);
    }
    uint32_t tiling_branches = branches;
    if (c->global.bayer_enables & (PISP_BE_BAYER_ENABLE_TDN_OUTPUT | PISP_BE_BAYER_ENABLE_STITCH_OUTPUT)) {
        /* History must cover the whole source even when both RGB crops are
         * smaller. A third tiling sink keeps those raw regions in the job. */
        uint32_t i = tiling_branches++;
        p.crop[i] = stage(&p, CROP, split, size, size);
        p.output[i] = stage(&p, OUTPUT, p.crop[i], size, size);
        p.output[i]->align_min = p.output[i]->align_max = p.input->align_min;
    }
    /* snapshot() clears inactive regions. Keep each branch's output cursor
     * separately so a tile serving only its sibling cannot rewind it. */
    int output_ends[3] = {0};
    for (int n = 0; n < capacity; ++n) {
        for (uint32_t i = 0; i < tiling_branches; ++i) {
            if (!p.output[i]->done) start_up(&p, p.output[i], output_ends[i]);
        }
        end_down(&p, p.input, p.input->r.input.start + p.maximum);
        crop_down(&p, p.input, p.input->r.input);
        if (p.error) return p.error;
        tiles[n].input = snapshot(p.input);
        tiles[n].context = snapshot(p.context);
        bool done = true, progress = false;
        for (uint32_t i = 0; i < tiling_branches; ++i) {
            if (i < branches) {
                tiles[n].crop[i] = snapshot(p.crop[i]);
                tiles[n].downscale[i] = snapshot(p.downscale[i]);
                tiles[n].resample[i] = snapshot(p.resample[i]);
                tiles[n].output[i] = snapshot(p.output[i]);
            }
            if (p.output[i]->done) continue;
            int end = p.output[i]->r.output.end;
            if (end > output_ends[i]) { progress = true; output_ends[i] = end; }
            if (end >= p.output[i]->output_size) p.output[i]->done = true;
            else done = false;
        }
        if (done) return n + 1;
        if (!progress) return -ERANGE;
        p.first = false;
    }
    return -E2BIG;
}

static int branch_axis(const struct pisp_be_config *c, uint32_t branch, const struct axis_tile *a,
                        int *crop_start, int *crop_end, struct interval *resample)
{
    const struct region *r = c->global.rgb_enables & (uint32_t)PISP_BE_RGB_ENABLE_DOWNSCALE(branch) ? &a->downscale[branch] :
        c->global.rgb_enables & (uint32_t)PISP_BE_RGB_ENABLE_RESAMPLE(branch) ? &a->resample[branch] : &a->output[branch];
    *crop_start = a->crop[branch].crop_start + r->crop_start;
    *crop_end = a->crop[branch].crop_end + r->crop_end;
    if (c->global.rgb_enables & (uint32_t)PISP_BE_RGB_ENABLE_DOWNSCALE(branch)) *resample = a->downscale[branch].output;
    else {
        *resample = a->crop[branch].output;
        resample->start -= a->resample[branch].crop_start;
        resample->end = resample->start + length(a->crop[branch].output) -
                        a->resample[branch].crop_start - a->resample[branch].crop_end;
    }
    return *crop_start < 0 || *crop_end < 0 || length(*resample) < 0 ? -ERANGE : 0;
}

static uint16_t downscale_phase(int offset, uint16_t factor)
{
    /* A later-starting output may need negative filter context coordinates. */
    int64_t fraction = (int64_t)offset * factor % 4096;
    if (fraction < 0) fraction += 4096;
    return (uint16_t)(4096 - fraction);
}

static uint16_t resample_phase(int offset, uint16_t factor)
{
    return (uint16_t)((((uint64_t)(uint32_t)offset * 16U * factor / 4096U) % 16U) * 256U);
}

int cc_pisp_tiles(struct pisp_be_tiles_config *parameters, const cc_rectangle crops[2],
                   const struct cc_pisp_output_layout layouts[2], uint32_t branches)
{
    struct axis_tile xs[64] = {0}, ys[64] = {0};
    const struct pisp_be_config *c = &parameters->config;
    int nx = axis(c, crops, layouts, branches, false, xs, 64);
    if (nx < 0) return nx;
    int ny = axis(c, crops, layouts, branches, true, ys, 64 / nx);
    if (ny < 0) return ny;
    for (int y = 0; y < ny; ++y) for (int x = 0; x < nx; ++x) {
        struct pisp_tile *t = &parameters->tiles[parameters->num_tiles++];
        int ix = xs[x].input.input.start, iy = ys[y].input.input.start;
        int iw = length(xs[x].input.input), ih = length(ys[y].input.input);
        if (ix < 0 || iy < 0 || iw < 16 || ih < 16 || iw > 640 || ih > 3072 ||
            ix + iw > c->input_format.width || iy + ih > c->input_format.height || (ix | iy | iw | ih) & 1)
            return -ERANGE;
        if (!x) t->edge |= PISP_LEFT_EDGE;
        if (!y) t->edge |= PISP_TOP_EDGE;
        if (x == nx - 1) t->edge |= PISP_RIGHT_EDGE;
        if (y == ny - 1) t->edge |= PISP_BOTTOM_EDGE;
        t->input_offset_x = (uint16_t)ix; t->input_offset_y = (uint16_t)iy;
        t->input_width = (uint16_t)iw; t->input_height = (uint16_t)ih;
        uint32_t pixel_bytes = PISP_IMAGE_FORMAT_COMPRESSED(c->input_format.format) ? 1U : 2U;
        if (ix % (int)(pixel_bytes == 1 ? 8U : 2U)) return -ERANGE;
        t->input_addr_offset = (uint32_t)iy * (uint32_t)c->input_format.stride + (uint32_t)ix * pixel_bytes;
        const struct pisp_image_format_config *aux[4] = {&c->tdn_input_format,&c->tdn_output_format,
            &c->stitch_input_format,&c->stitch_output_format};
        uint32_t offsets[4] = {0};
        for (uint32_t i = 0; i < 4; ++i)
            if (aux[i]->width)
                offsets[i] = (uint32_t)iy * (uint32_t)aux[i]->stride +
                    (uint32_t)ix * (PISP_IMAGE_FORMAT_COMPRESSED(aux[i]->format) ? 1U : 2U);
        t->tdn_input_addr_offset = offsets[0]; t->tdn_output_addr_offset = offsets[1];
        t->stitch_input_addr_offset = offsets[2]; t->stitch_output_addr_offset = offsets[3];
        if (c->global.bayer_enables & PISP_BE_BAYER_ENABLE_LSC) {
            t->lsc_grid_offset_x = ((uint32_t)ix + c->lsc_extra.offset_x) * c->lsc.grid_step_x;
            t->lsc_grid_offset_y = ((uint32_t)iy + c->lsc_extra.offset_y) * c->lsc.grid_step_y;
        }
        if (c->global.bayer_enables & PISP_BE_BAYER_ENABLE_CAC) {
            t->cac_grid_offset_x = ((uint32_t)ix + c->cac_extra.offset_x) * c->cac.grid_step_x;
            t->cac_grid_offset_y = ((uint32_t)iy + c->cac_extra.offset_y) * c->cac.grid_step_y;
        }
        for (uint32_t i = 0; i < branches; ++i) {
            int ow = length(xs[x].output[i].output), oh = length(ys[y].output[i].output);
            if (!ow || !oh) { t->crop_x_start[i] = (uint16_t)iw; t->crop_y_start[i] = (uint16_t)ih; continue; }
            int cx0, cx1, cy0, cy1;
            struct interval rx, ry;
            if (branch_axis(c, i, &xs[x], &cx0, &cx1, &rx) || branch_axis(c, i, &ys[y], &cy0, &cy1, &ry)) return -ERANGE;
            int ox = xs[x].output[i].output.start, oy = ys[y].output[i].output.start;
            bool right = ox + ow == c->output_format[i].image.width;
            if (ow < 0 || ow > 640 || oh < 16 || oh > 3072 || length(rx) <= 0 || length(ry) <= 0 ||
                ox < 0 || oy < 0 || ox + ow > c->output_format[i].image.width ||
                oy + oh > c->output_format[i].image.height || cx0 + cx1 >= iw || cy0 + cy1 >= ih ||
                (!right && (ow < 16 || iw - cx0 - cx1 < 16 || length(rx) < 16)) ||
                ih - cy0 - cy1 < 16 || length(ry) < 16) return -ERANGE;
            t->crop_x_start[i] = (uint16_t)cx0; t->crop_x_end[i] = (uint16_t)cx1;
            t->crop_y_start[i] = (uint16_t)cy0; t->crop_y_end[i] = (uint16_t)cy1;
            t->resample_in_width[i] = (uint16_t)length(rx); t->resample_in_height[i] = (uint16_t)length(ry);
            t->output_width[i] = (uint16_t)ow; t->output_height[i] = (uint16_t)oh;
            for (uint32_t plane = 0; plane < 3; ++plane) {
                if (c->global.rgb_enables & (uint32_t)PISP_BE_RGB_ENABLE_DOWNSCALE(i)) {
                    t->downscale_phase_x[plane * 2 + i] = downscale_phase(rx.start, c->downscale[i].scale_factor_h);
                    t->downscale_phase_y[plane * 2 + i] = downscale_phase(ry.start, c->downscale[i].scale_factor_v);
                }
                if (c->global.rgb_enables & (uint32_t)PISP_BE_RGB_ENABLE_RESAMPLE(i)) {
                    t->resample_phase_x[plane * 2 + i] = resample_phase(ox, c->resample[i].scale_factor_h);
                    t->resample_phase_y[plane * 2 + i] = resample_phase(oy, c->resample[i].scale_factor_v);
                }
            }
            uint32_t dx = (uint32_t)ox, dy = (uint32_t)oy;
            if (c->output_format[i].transform & PISP_BE_TRANSFORM_HFLIP) dx = c->output_format[i].image.width - dx - (uint32_t)ow;
            if (c->output_format[i].transform & PISP_BE_TRANSFORM_VFLIP) dy = c->output_format[i].image.height - dy - 1U;
            t->output_offset_x[i] = (uint16_t)dx; t->output_offset_y[i] = (uint16_t)dy;
            t->output_addr_offset[i] = dy * (uint32_t)layouts[i].native.stride + layouts[i].pixel_bytes * dx;
            if (layouts[i].components > 1)
                t->output_addr_offset2[i] = dy / layouts[i].ysub * (uint32_t)layouts[i].native.stride2 + dx / layouts[i].xsub;
        }
    }
    return 0;
}
