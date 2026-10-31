/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_DELAYED_CONTROLS_H
#define CCAMERA_DELAYED_CONTROLS_H
#include <ccamera/sensor.h>
#include <stdbool.h>

struct cc_delayed_controls;
struct cc_delayed_batch {
    uint64_t frame;
    size_t count;
    cc_control controls[CC_SENSOR_MAX_CONTROLS];
    uint32_t indices[CC_SENSOR_MAX_CONTROLS];
    uint64_t targets[CC_SENSOR_MAX_CONTROLS];
    bool priority[CC_SENSOR_MAX_CONTROLS];
};
int cc_delayed_create(const cc_sensor_control_delay *params, const cc_control *initial, size_t count,
                      struct cc_delayed_controls **out);
void cc_delayed_destroy(struct cc_delayed_controls *delayed);
int cc_delayed_reset(struct cc_delayed_controls *delayed, const cc_control *initial);
int cc_delayed_schedule(struct cc_delayed_controls *delayed, uint32_t target, const cc_control *controls,
                        size_t count, uint64_t cookie);
int cc_delayed_schedule_next(struct cc_delayed_controls *delayed, const cc_control *controls,
                             size_t count, uint64_t cookie, uint32_t *target);
int cc_delayed_prepare(struct cc_delayed_controls *delayed, uint32_t sequence, struct cc_delayed_batch *batch);
int cc_delayed_commit(struct cc_delayed_controls *delayed, const struct cc_delayed_batch *batch);
int cc_delayed_get(const struct cc_delayed_controls *delayed, uint32_t sequence, cc_sensor_controls *controls);
#endif
