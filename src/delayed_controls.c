/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Staggered sensor writes and priority-before-batch ordering follow libcamera's
 * delayed-controls model (Copyright (C) 2020 Raspberry Pi Ltd). This C17
 * implementation uses explicit target frames, stamped history and acknowledged
 * writes so missed deadlines and adjusted driver values cannot silently alias. */
#include "delayed_controls.h"

#include <errno.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

struct planned {
    uint64_t frame, cookie;
    uint32_t mask, written;
    bool valid;
    int64_t values[CC_SENSOR_MAX_CONTROLS];
};
struct historic {
    uint64_t frame, cookie;
    bool valid;
    int64_t values[CC_SENSOR_MAX_CONTROLS];
};
struct cc_delayed_controls {
    size_t count;
    cc_sensor_control_delay params[CC_SENSOR_MAX_CONTROLS];
    uint64_t next, cookie;
    bool started;
    int64_t effective[CC_SENSOR_MAX_CONTROLS];
    struct planned plans[CC_SENSOR_CONTROL_HISTORY];
    struct historic history[CC_SENSOR_CONTROL_HISTORY];
};

int cc_delayed_reset(struct cc_delayed_controls *d, const cc_control *initial)
{
    if (!d || !initial) return -EINVAL;
    for (size_t i = 0; i < d->count; ++i)
        if (initial[i].id != d->params[i].id || initial[i].data || initial[i].size) return -EINVAL;
    memset(d->plans, 0, sizeof(d->plans)); memset(d->history, 0, sizeof(d->history));
    d->next = d->cookie = 0; d->started = false;
    for (size_t i = 0; i < d->count; ++i) d->effective[i] = initial[i].value;
    return 0;
}
int cc_delayed_create(const cc_sensor_control_delay *params, const cc_control *initial, size_t count,
                      struct cc_delayed_controls **out)
{
    if (!out) return -EINVAL;
    *out = NULL;
    if (!params || !initial || !count || count > CC_SENSOR_MAX_CONTROLS) return -EINVAL;
    for (size_t i = 0; i < count; ++i) {
        if (!params[i].id || params[i].frames > CC_SENSOR_MAX_DELAY || params[i].flags & ~CC_SENSOR_CONTROL_PRIORITY)
            return -EINVAL;
        for (size_t j = 0; j < i; ++j) if (params[i].id == params[j].id) return -EEXIST;
    }
    struct cc_delayed_controls *d = calloc(1, sizeof(*d));
    if (!d) return -ENOMEM;
    d->count = count; memcpy(d->params, params, count * sizeof(*params));
    int ret = cc_delayed_reset(d, initial);
    if (ret < 0) { free(d); return ret; }
    *out = d;
    return 0;
}
void cc_delayed_destroy(struct cc_delayed_controls *d) { free(d); }

int cc_delayed_schedule(struct cc_delayed_controls *d, uint32_t target, const cc_control *controls,
                        size_t count, uint64_t cookie)
{
    if (!d || (count && !controls) || count > d->count) return -EINVAL;
    uint32_t distance = target - (uint32_t)d->next;
    if (distance > INT32_MAX) return -ETIME;
    if (distance >= CC_SENSOR_CONTROL_HISTORY || d->next > UINT64_MAX - distance) return -ERANGE;
    uint64_t frame = d->next + distance;
    struct planned *p = &d->plans[frame % CC_SENSOR_CONTROL_HISTORY];
    if (p->valid && p->frame != frame) return -ENOSPC;
    uint32_t indices[CC_SENSOR_MAX_CONTROLS], mask = 0;
    for (size_t i = 0; i < count; ++i) {
        if (controls[i].data || controls[i].size) return -ENOTSUP;
        size_t n = 0;
        while (n < d->count && d->params[n].id != controls[i].id) ++n;
        if (n == d->count) return -ENOENT;
        uint32_t bit = UINT32_C(1) << n;
        if (mask & bit) return -EEXIST;
        if (frame < d->params[n].frames || frame - d->params[n].frames < d->next) return -ETIME;
        indices[i] = (uint32_t)n; mask |= bit;
    }
    if (!p->valid) *p = (struct planned) { .valid = true, .frame = frame };
    for (size_t i = 0; i < count; ++i) p->values[indices[i]] = controls[i].value;
    p->mask |= mask; p->cookie = cookie;
    return 0;
}

int cc_delayed_schedule_next(struct cc_delayed_controls *d, const cc_control *controls,
                             size_t count, uint64_t cookie, uint32_t *target)
{
    if (!d || !target || (count && !controls) || count > d->count) return -EINVAL;
    uint32_t delay = 0;
    for (size_t i = 0; i < count; ++i) {
        size_t n = 0;
        while (n < d->count && d->params[n].id != controls[i].id) ++n;
        if (n == d->count) return -ENOENT;
        if (d->params[n].frames > delay) delay = d->params[n].frames;
    }
    if (d->next >= UINT64_MAX - delay) return -ERANGE;
    uint32_t frame = (uint32_t)(d->next + delay);
    int ret = cc_delayed_schedule(d, frame, controls, count, cookie);
    if (!ret) *target = frame;
    return ret;
}

int cc_delayed_prepare(struct cc_delayed_controls *d, uint32_t sequence, struct cc_delayed_batch *out)
{
    if (!d || !out) return -EINVAL;
    if (d->started && sequence == (uint32_t)(d->next - 1U)) return -EALREADY;
    uint32_t delta = sequence - (uint32_t)d->next;
    if (delta > INT32_MAX || d->next > UINT64_MAX - delta - 1U) return -ESTALE;
    uint64_t frame = d->next + delta;
    struct cc_delayed_batch b = {.frame = frame};
    for (size_t slot = 0; slot < CC_SENSOR_CONTROL_HISTORY; ++slot) {
        const struct planned *p = &d->plans[slot];
        if (!p->valid) continue;
        for (size_t i = 0; i < d->count; ++i) {
            uint32_t bit = UINT32_C(1) << i;
            if (!(p->mask & bit) || p->written & bit) continue;
            uint64_t due = p->frame - d->params[i].frames;
            if (due < frame) return -ESTALE;
            if (due == frame) {
                size_t n = b.count++;
                b.controls[n] = (cc_control) {.id = d->params[i].id, .value = p->values[i]};
                b.indices[n] = (uint32_t)i; b.targets[n] = p->frame;
                b.priority[n] = (d->params[i].flags & CC_SENSOR_CONTROL_PRIORITY) != 0;
            }
        }
    }
    *out = b;
    return 0;
}

static int effect(struct cc_delayed_controls *d, uint64_t frame)
{
    struct planned *p = &d->plans[frame % CC_SENSOR_CONTROL_HISTORY];
    if (!p->valid || p->frame != frame) return 0;
    if (p->mask != p->written) return -ESTALE;
    for (size_t i = 0; i < d->count; ++i)
        if (p->mask & (UINT32_C(1) << i)) d->effective[i] = p->values[i];
    d->cookie = p->cookie; p->valid = false;
    return 0;
}
int cc_delayed_commit(struct cc_delayed_controls *d, const struct cc_delayed_batch *b)
{
    if (!d || !b || b->count > d->count || b->frame < d->next || b->frame == UINT64_MAX) return -EINVAL;
    struct cc_delayed_batch expected;
    int prepared = cc_delayed_prepare(d, (uint32_t)b->frame, &expected);
    if (prepared < 0 || expected.frame != b->frame || expected.count != b->count) return -EINVAL;
    /* Verify the acknowledgement before modifying state. The caller has only
     * changed returned scalar values to those acknowledged by the driver. */
    for (size_t n = 0; n < b->count; ++n) {
        uint32_t i = b->indices[n];
        if (i >= d->count || b->controls[n].id != d->params[i].id || b->controls[n].data || b->controls[n].size ||
            i != expected.indices[n] || b->targets[n] != expected.targets[n] || b->priority[n] != expected.priority[n])
            return -EINVAL;
        const struct planned *p = &d->plans[b->targets[n] % CC_SENSOR_CONTROL_HISTORY];
        uint32_t bit = UINT32_C(1) << i;
        if (!p->valid || p->frame != b->targets[n] || !(p->mask & bit) || p->written & bit ||
            p->frame - d->params[i].frames != b->frame) return -EINVAL;
    }
    for (size_t n = 0; n < b->count; ++n) {
        struct planned *p = &d->plans[b->targets[n] % CC_SENSOR_CONTROL_HISTORY];
        p->values[b->indices[n]] = b->controls[n].value;
        p->written |= UINT32_C(1) << b->indices[n];
    }
    uint64_t start = b->frame >= CC_SENSOR_CONTROL_HISTORY ? b->frame - CC_SENSOR_CONTROL_HISTORY + 1U : 0;
    if (start < d->next) start = d->next;
    /* A long gap without due writes is safe. Fold the few pending effects that
     * precede the retained history, then fill at most 128 stamped frames. */
    for (;;) {
        uint64_t first = UINT64_MAX;
        for (size_t i = 0; i < CC_SENSOR_CONTROL_HISTORY; ++i)
            if (d->plans[i].valid && d->plans[i].frame < start && d->plans[i].frame < first)
                first = d->plans[i].frame;
        if (first == UINT64_MAX) break;
        int ret = effect(d, first);
        if (ret < 0) return ret;
    }
    for (uint64_t frame = start; frame <= b->frame; ++frame) {
        int ret = effect(d, frame);
        if (ret < 0) return ret;
        struct historic *h = &d->history[frame % CC_SENSOR_CONTROL_HISTORY];
        h->frame = frame; h->cookie = d->cookie; h->valid = true;
        memcpy(h->values, d->effective, d->count * sizeof(*d->effective));
    }
    d->next = b->frame + 1U; d->started = true;
    return 0;
}
int cc_delayed_get(const struct cc_delayed_controls *d, uint32_t sequence, cc_sensor_controls *out)
{
    if (!d || !out) return -EINVAL;
    if (!d->started) return -EAGAIN;
    uint32_t age = (uint32_t)(d->next - 1U) - sequence;
    if (age > INT32_MAX) return -EAGAIN;
    if (age >= CC_SENSOR_CONTROL_HISTORY || age >= d->next) return -ESTALE;
    uint64_t frame = d->next - 1U - age;
    const struct historic *h = &d->history[frame % CC_SENSOR_CONTROL_HISTORY];
    if (!h->valid || h->frame != frame) return -ESTALE;
    cc_sensor_controls values = {.frame_sequence = sequence, .count = (uint32_t)d->count, .cookie = h->cookie};
    for (size_t i = 0; i < d->count; ++i)
        values.controls[i] = (cc_control) {.id = d->params[i].id, .value = h->values[i]};
    *out = values;
    return 0;
}
