/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "delayed_controls.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
#define OK(x) CHECK((x) == 0)

static struct cc_delayed_controls *create(void)
{
    const cc_sensor_control_delay params[] = {{1,2,0},{2,1,0},{3,2,CC_SENSOR_CONTROL_PRIORITY},{4,0,0}};
    const cc_control initial[] = {{.id=1,.value=10},{.id=2,.value=20},{.id=3,.value=30},{.id=4,.value=40}};
    struct cc_delayed_controls *d = NULL;
    OK(cc_delayed_create(params, initial, 4, &d));
    return d;
}
static void advance(struct cc_delayed_controls *d, uint32_t frame)
{
    struct cc_delayed_batch b;
    OK(cc_delayed_prepare(d, frame, &b)); OK(cc_delayed_commit(d, &b));
}
static void test_stagger(void)
{
    struct cc_delayed_controls *d = create();
    cc_control values[] = {{.id=1,.value=100},{.id=2,.value=200},{.id=3,.value=300},{.id=4,.value=400}};
    OK(cc_delayed_schedule(d, 2, values, 4, 55));
    struct cc_delayed_batch b;
    OK(cc_delayed_prepare(d, 0, &b)); CHECK(b.count == 2);
    for (size_t i = 0; i < b.count; ++i) {
        CHECK(b.controls[i].id == 1 || b.controls[i].id == 3);
        CHECK(b.priority[i] == (b.controls[i].id == 3));
        if (b.controls[i].id == 1) b.controls[i].value = 99; /* Driver adjustment. */
    }
    struct cc_delayed_batch incomplete = b; incomplete.count = 0;
    CHECK(cc_delayed_commit(d, &incomplete) == -EINVAL);
    OK(cc_delayed_commit(d, &b));
    cc_sensor_controls v;
    OK(cc_delayed_get(d, 0, &v)); CHECK(v.controls[0].value == 10 && !v.cookie);
    OK(cc_delayed_prepare(d, 1, &b)); CHECK(b.count == 1 && b.controls[0].id == 2);
    OK(cc_delayed_commit(d, &b));
    OK(cc_delayed_get(d, 1, &v)); CHECK(v.controls[1].value == 20 && !v.cookie);
    OK(cc_delayed_prepare(d, 2, &b)); CHECK(b.count == 1 && b.controls[0].id == 4);
    OK(cc_delayed_commit(d, &b));
    OK(cc_delayed_get(d, 2, &v));
    CHECK(v.count == 4 && v.cookie == 55 && v.frame_sequence == 2);
    CHECK(v.controls[0].value == 99 && v.controls[1].value == 200 && v.controls[2].value == 300 && v.controls[3].value == 400);
    CHECK(cc_delayed_prepare(d, 2, &b) == -EALREADY);
    CHECK(cc_delayed_prepare(d, 1, &b) == -ESTALE);
    CHECK(cc_delayed_schedule(d, 4, values, 1, 77) == -ETIME);
    OK(cc_delayed_schedule(d, 5, values, 1, 66));
    cc_control duplicate[] = {{.id=1,.value=999},{.id=1,.value=1000}};
    CHECK(cc_delayed_schedule(d, 5, duplicate, 2, 77) == -EEXIST);
    cc_control unknown[] = {{.id=1,.value=999},{.id=999,.value=1000}};
    CHECK(cc_delayed_schedule(d, 5, unknown, 2, 77) == -ENOENT);
    advance(d, 3); advance(d, 4); advance(d, 5);
    OK(cc_delayed_get(d, 5, &v)); CHECK(v.controls[0].value == 100 && v.cookie == 66);
    CHECK(cc_delayed_get(d, 6, &v) == -EAGAIN);
    for (uint32_t f = 6; f <= 140; ++f) advance(d, f);
    CHECK(cc_delayed_get(d, 12, &v) == -ESTALE);
    OK(cc_delayed_get(d, 13, &v)); CHECK(v.controls[0].value == 100 && v.cookie == 66);
    cc_delayed_destroy(d);
}
static void test_gaps_and_wrap(void)
{
    struct cc_delayed_controls *d = create();
    cc_control control = {.id=1,.value=123};
    OK(cc_delayed_schedule(d, 2, &control, 1, 44));
    struct cc_delayed_batch b, unchanged;
    memset(&b, 0xa5, sizeof(b)); unchanged = b;
    CHECK(cc_delayed_prepare(d, 1, &b) == -ESTALE && !memcmp(&b, &unchanged, sizeof(b)));
    advance(d, 0); advance(d, 1000); /* Write was acknowledged before the gap. */
    cc_sensor_controls v;
    OK(cc_delayed_get(d, 1000, &v)); CHECK(v.cookie == 44 && v.controls[0].value == 123);
    CHECK(cc_delayed_get(d, 2, &v) == -ESTALE);
    advance(d, UINT32_C(0x7fffffff)); advance(d, UINT32_C(0xfffffffe));
    OK(cc_delayed_schedule(d, 1, &control, 1, 99));
    advance(d, UINT32_MAX); advance(d, 0); advance(d, 1);
    OK(cc_delayed_get(d, 0, &v)); CHECK(v.cookie == 44);
    OK(cc_delayed_get(d, 1, &v)); CHECK(v.cookie == 99);
    CHECK(cc_delayed_schedule(d, 1, &control, 1, 0) == -ETIME);
    CHECK(cc_delayed_schedule(d, 130, &control, 1, 0) == -ERANGE);
    cc_delayed_destroy(d);
}
static uint32_t random_value(uint32_t *state) { *state = *state * UINT32_C(1664525) + UINT32_C(1013904223); return *state; }
static void test_model(void)
{
    enum {N=2000, C=5};
    struct model { uint32_t mask; int64_t desired[C], actual[C]; uint64_t cookie; bool context; } model[N+32] = {{0}};
    cc_sensor_control_delay params[C]; cc_control initial[C];
    int64_t effective[C];
    for (uint32_t i = 0; i < C; ++i) {
        params[i] = (cc_sensor_control_delay) {i+1,i,i==2 ? CC_SENSOR_CONTROL_PRIORITY : 0};
        initial[i] = (cc_control) {.id=i+1,.value=(int64_t)i}; effective[i] = (int64_t)i;
    }
    struct cc_delayed_controls *d;
    OK(cc_delayed_create(params, initial, C, &d));
    uint32_t random = 12345678; uint64_t cookie = 0;
    for (uint32_t frame = 0; frame < N; ++frame) {
        uint32_t target = frame + 4U + random_value(&random) % 12U;
        cc_control updates[C]; size_t count = 0;
        uint32_t mask = random_value(&random) & 31U;
        for (uint32_t i = 0; i < C; ++i) if (mask & (1U << i)) {
            int64_t value = (int64_t)(random_value(&random) % 2000U);
            updates[count++] = (cc_control) {.id=i+1,.value=value};
            model[target].mask |= 1U << i; model[target].desired[i] = value;
        }
        model[target].cookie = frame + 1U; model[target].context = true;
        OK(cc_delayed_schedule(d, target, updates, count, frame + 1U));
        struct cc_delayed_batch b;
        OK(cc_delayed_prepare(d, frame, &b));
        uint32_t found = 0;
        for (size_t n = 0; n < b.count; ++n) {
            uint32_t i = b.controls[n].id - 1U, t = frame + i;
            CHECK(i < C && model[t].mask & (1U << i) && !(found & (1U << i)));
            CHECK(b.controls[n].value == model[t].desired[i] && b.priority[n] == (i == 2));
            found |= 1U << i;
            int64_t actual = b.controls[n].value > 1000 ? 1000 : b.controls[n].value;
            b.controls[n].value = model[t].actual[i] = actual;
        }
        for (uint32_t i = 0; i < C; ++i) CHECK(!!(found & (1U << i)) == !!(model[frame+i].mask & (1U << i)));
        OK(cc_delayed_commit(d, &b));
        for (uint32_t i = 0; i < C; ++i) if (model[frame].mask & (1U << i)) effective[i] = model[frame].actual[i];
        if (model[frame].context) cookie = model[frame].cookie;
        cc_sensor_controls values;
        OK(cc_delayed_get(d, frame, &values)); CHECK(values.cookie == cookie && values.count == C);
        for (uint32_t i = 0; i < C; ++i) CHECK(values.controls[i].id == i+1 && values.controls[i].value == effective[i]);
    }
    cc_delayed_destroy(d);
}
static void test_full_mask(void)
{
    cc_sensor_control_delay params[32]; cc_control initial[32], updates[32];
    for (uint32_t i = 0; i < 32; ++i) {
        params[i] = (cc_sensor_control_delay) {i+1,64,0};
        initial[i] = (cc_control) {.id=i+1,.value=0}; updates[i] = (cc_control) {.id=i+1,.value=100+i};
    }
    struct cc_delayed_controls *d;
    OK(cc_delayed_create(params, initial, 32, &d));
    uint32_t target;
    OK(cc_delayed_schedule_next(d, updates, 32, UINT64_MAX, &target)); CHECK(target == 64);
    for (uint32_t f = 0; f <= 64; ++f) advance(d, f);
    cc_sensor_controls values;
    OK(cc_delayed_get(d, 64, &values)); CHECK(values.count == 32 && values.cookie == UINT64_MAX);
    CHECK(values.controls[31].value == 131);
    params[0].frames = 65;
    struct cc_delayed_controls *bad = (void *)(uintptr_t)1;
    CHECK(cc_delayed_create(params, initial, 32, &bad) == -EINVAL && !bad);
    cc_delayed_destroy(d);
}
static void test_next_target(void)
{
    struct cc_delayed_controls *d = create();
    cc_control controls[] = {{.id=1,.value=101}, {.id=2,.value=202}};
    uint32_t target = 999;
    OK(cc_delayed_schedule_next(d, controls, 2, 7, &target)); CHECK(target == 2);
    advance(d, 0); advance(d, 1); advance(d, 2);
    cc_sensor_controls v;
    OK(cc_delayed_get(d, 2, &v)); CHECK(v.cookie == 7 && v.controls[0].value == 101);
    OK(cc_delayed_schedule_next(d, controls + 1, 1, 8, &target)); CHECK(target == 4);
    advance(d, 3); advance(d, 4);
    OK(cc_delayed_get(d, 4, &v)); CHECK(v.cookie == 8);
    /* Zero-delay controls need no extra frame, even when other configured
     * controls have delays. Targets wrap with the sensor, not request numbers. */
    cc_control zero = {.id=4,.value=404};
    OK(cc_delayed_schedule_next(d, &zero, 1, 9, &target)); CHECK(target == 5);
    advance(d, 5);
    advance(d, UINT32_C(0x7fffffff)); advance(d, UINT32_C(0xfffffffe));
    OK(cc_delayed_schedule_next(d, controls, 2, 10, &target)); CHECK(target == 1);
    advance(d, UINT32_MAX); advance(d, 0); advance(d, 1);
    OK(cc_delayed_get(d, 1, &v)); CHECK(v.cookie == 10 && v.controls[1].value == 202);
    cc_control bad[] = {{.id=1,.value=1}, {.id=999,.value=2}};
    target = 123;
    CHECK(cc_delayed_schedule_next(d, bad, 2, 0, &target) == -ENOENT && target == 123);
    bad[1] = bad[0];
    CHECK(cc_delayed_schedule_next(d, bad, 2, 0, &target) == -EEXIST && target == 123);
    bad[1] = (cc_control) {.id=2,.data=bad,.size=1};
    CHECK(cc_delayed_schedule_next(d, bad, 2, 0, &target) == -ENOTSUP && target == 123);
    advance(d, 2); advance(d, 3); advance(d, 4);
    OK(cc_delayed_get(d, 4, &v)); CHECK(v.cookie == 10); /* Rejections scheduled nothing. */
    cc_delayed_destroy(d);
}
int main(void)
{
    test_stagger(); test_gaps_and_wrap(); test_model(); test_full_mask(); test_next_target();
    puts("Delayed-control stagger, acknowledgements, contexts, atomic validation, history and wrap passed");
    return 0;
}
