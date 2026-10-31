/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "fuzz.h"
#include "delayed_controls.h"
#include <errno.h>

struct plan {
    uint32_t mask;
    int64_t values[CC_SENSOR_MAX_CONTROLS];
    uint64_t cookie;
    bool valid;
};

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    struct fuzz_input in = {data, size};
    uint8_t mode = fuzz_u8(&in);
    size_t count = 1U + fuzz_u8(&in) % CC_SENSOR_MAX_CONTROLS;
    cc_sensor_control_delay params[CC_SENSOR_MAX_CONTROLS];
    cc_control initial[CC_SENSOR_MAX_CONTROLS];
    int64_t effective[CC_SENSOR_MAX_CONTROLS];
    uint32_t max_delay = 0;
    for (size_t i = 0; i < count; ++i) {
        uint32_t delay = fuzz_u8(&in) % (CC_SENSOR_MAX_DELAY + 1U);
        if (delay > max_delay) max_delay = delay;
        params[i] = (cc_sensor_control_delay){(uint32_t)i + 1, delay, i & 1U ? CC_SENSOR_CONTROL_PRIORITY : 0};
        effective[i] = (int64_t)fuzz_u64(&in);
        initial[i] = (cc_control){.id = (uint32_t)i + 1, .value = effective[i]};
    }
    struct cc_delayed_controls *d = NULL;
    FUZZ_CHECK(cc_delayed_create(params, initial, count, &d) == 0);
    uint64_t next = 0, cookie = 0;
    if (mode & 1U) {
        /* Reach the wrap boundary without billions of iterations. */
        struct cc_delayed_batch b;
        FUZZ_CHECK(cc_delayed_prepare(d, UINT32_C(0x7fffffff), &b) == 0 && b.count == 0);
        FUZZ_CHECK(cc_delayed_commit(d, &b) == 0);
        FUZZ_CHECK(cc_delayed_prepare(d, UINT32_C(0xfffffffe), &b) == 0 && b.count == 0);
        FUZZ_CHECK(cc_delayed_commit(d, &b) == 0);
        next = UINT64_C(0xffffffff);
    }
    struct plan plans[CC_SENSOR_CONTROL_HISTORY] = {{0}};
    unsigned steps = 1U + fuzz_u8(&in);
    for (unsigned step = 0; step < steps; ++step, ++next) {
        uint8_t op = fuzz_u8(&in);
        uint64_t target = next + max_delay + fuzz_u8(&in) % (CC_SENSOR_CONTROL_HISTORY - max_delay);
        uint32_t mask = fuzz_u32(&in);
        cc_control updates[CC_SENSOR_MAX_CONTROLS]; size_t n = 0;
        uint32_t selected_delay = 0;
        for (size_t i = 0; i < count; ++i) if (mask & (UINT32_C(1) << i)) {
            int64_t value = (int64_t)fuzz_u64(&in);
            updates[n++] = (cc_control){.id = (uint32_t)i + 1, .value = value};
            if (params[i].frames > selected_delay) selected_delay = params[i].frames;
        }
        if (op & 2U) target = next + selected_delay;
        struct plan *p = &plans[target % CC_SENSOR_CONTROL_HISTORY];
        for (size_t j = 0; j < n; ++j) {
            uint32_t i = updates[j].id - 1U;
            p->values[i] = updates[j].value; p->mask |= UINT32_C(1) << i;
        }
        p->valid = true; p->cookie = fuzz_u64(&in);
        if (op & 2U) {
            uint32_t assigned;
            FUZZ_CHECK(cc_delayed_schedule_next(d, updates, n, p->cookie, &assigned) == 0);
            FUZZ_CHECK(assigned == (uint32_t)target);
        } else FUZZ_CHECK(cc_delayed_schedule(d, (uint32_t)target, updates, n, p->cookie) == 0);
        cc_control unknown = {.id = UINT32_MAX};
        FUZZ_CHECK(cc_delayed_schedule(d, (uint32_t)target, &unknown, 1, 0) == -ENOENT);
        struct cc_delayed_batch b;
        FUZZ_CHECK(cc_delayed_prepare(d, (uint32_t)next, &b) == 0);
        uint32_t found = 0;
        for (size_t j = 0; j < b.count; ++j) {
            uint32_t i = b.controls[j].id - 1U;
            FUZZ_CHECK(i < count && !(found & (UINT32_C(1) << i)));
            found |= UINT32_C(1) << i;
            struct plan *due = &plans[(next + params[i].frames) % CC_SENSOR_CONTROL_HISTORY];
            FUZZ_CHECK(due->valid && (due->mask & (UINT32_C(1) << i)));
            FUZZ_CHECK(b.controls[j].value == due->values[i]);
            FUZZ_CHECK(b.priority[j] == !!(params[i].flags & CC_SENSOR_CONTROL_PRIORITY));
            if (op & 1U) due->values[i] = b.controls[j].value = (int64_t)fuzz_u64(&in);
        }
        for (size_t i = 0; i < count; ++i) {
            struct plan *due = &plans[(next + params[i].frames) % CC_SENSOR_CONTROL_HISTORY];
            FUZZ_CHECK(!!(found & (UINT32_C(1) << i)) ==
                       !!(due->valid && (due->mask & (UINT32_C(1) << i))));
        }
        struct cc_delayed_batch bad = b;
        bad.count = count + 1;
        FUZZ_CHECK(cc_delayed_commit(d, &bad) == -EINVAL);
        if (b.count) {
            bad = b; bad.indices[0] = UINT32_MAX;
            FUZZ_CHECK(cc_delayed_commit(d, &bad) == -EINVAL);
        }
        FUZZ_CHECK(cc_delayed_commit(d, &b) == 0);
        struct plan *now = &plans[next % CC_SENSOR_CONTROL_HISTORY];
        if (now->valid) {
            for (size_t i = 0; i < count; ++i)
                if (now->mask & (UINT32_C(1) << i)) effective[i] = now->values[i];
            cookie = now->cookie; memset(now, 0, sizeof(*now));
        }
        cc_sensor_controls got;
        FUZZ_CHECK(cc_delayed_get(d, (uint32_t)next, &got) == 0);
        FUZZ_CHECK(got.count == count && got.cookie == cookie && got.frame_sequence == (uint32_t)next);
        for (size_t i = 0; i < count; ++i)
            FUZZ_CHECK(got.controls[i].id == i + 1 && got.controls[i].value == effective[i]);
        FUZZ_CHECK(cc_delayed_prepare(d, (uint32_t)next, &b) == -EALREADY);
        FUZZ_CHECK(cc_delayed_get(d, (uint32_t)(next + 1U), &got) == -EAGAIN);
        if (step >= CC_SENSOR_CONTROL_HISTORY)
            FUZZ_CHECK(cc_delayed_get(d, (uint32_t)(next - CC_SENSOR_CONTROL_HISTORY), &got) == -ESTALE);
    }
    FUZZ_CHECK(cc_delayed_reset(d, initial) == 0);
    cc_sensor_controls got;
    FUZZ_CHECK(cc_delayed_get(d, 0, &got) == -EAGAIN);
    cc_delayed_destroy(d);
    return 0;
}
