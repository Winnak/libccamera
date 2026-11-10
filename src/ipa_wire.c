/* SPDX-License-Identifier: LGPL-2.1-or-later */
#define _GNU_SOURCE
#include "ipa_wire.h"
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

union wire_object {
    cc_ipa_configuration configuration;
    cc_ipa_request request;
    cc_ipa_settings settings;
    cc_ipa_frame frame;
    cc_ipa_metadata metadata;
    cc_ipa_reference_tuning reference;
    cc_ipa_vc4_options vc4;
};
struct allocation { struct allocation *next; void *data; };
struct cc_ipa_wire {
    union wire_object object;
    struct allocation *allocations;
    size_t bytes, count;
    int fds[CC_IPA_WIRE_MAX_FDS];
    uint32_t num_fds;
    bool owned_fds;
};
struct codec {
    cc_ipa_wire *owner;
    const unsigned char *input;
    unsigned char *output;
    size_t position, size, capacity;
    int error;
    bool decode, allow_fds;
    bool used_fds[CC_IPA_WIRE_MAX_FDS];
};
#define FIELD(value, field) transfer(c, &(value)->field, sizeof((value)->field))
#define BLOCK(value) transfer(c, (value), sizeof(*(value)))
static void transfer(struct codec *c, void *value, size_t size)
{
    if (c->error) return;
    if (size > CC_IPA_WIRE_MAX_DATA - c->position) { c->error = -E2BIG; return; }
    if (size && !value) { c->error = -EINVAL; return; }
    if (c->decode) {
        if (size > c->size - c->position) { c->error = -EPROTO; return; }
        if (size) memcpy(value, c->input + c->position, size);
    } else {
        size_t end = c->position + size;
        if (end > c->capacity) {
            size_t capacity = c->capacity ? c->capacity : 1024U;
            while (capacity < end) capacity *= 2;
            unsigned char *output = realloc(c->output, capacity);
            if (!output) { c->error = -ENOMEM; return; }
            c->output = output; c->capacity = capacity;
        }
        if (size) memcpy(c->output + c->position, value, size);
    }
    c->position += size;
}
static void *allocate(struct codec *c, size_t size)
{
    if (c->error) return NULL;
    if (c->owner->count >= 256U || size > CC_IPA_WIRE_MAX_DATA + 65536U - c->owner->bytes) {
        c->error = -E2BIG; return NULL;
    }
    struct allocation *allocation = malloc(sizeof(*allocation));
    if (!allocation) { c->error = -ENOMEM; return NULL; }
    allocation->data = calloc(1, size ? size : 1);
    if (!allocation->data) { free(allocation); c->error = -ENOMEM; return NULL; }
    allocation->next = c->owner->allocations; c->owner->allocations = allocation;
    c->owner->bytes += size; ++c->owner->count; return allocation->data;
}
static size_t count(struct codec *c, size_t value, size_t maximum)
{
    if (c->error) return 0;
    if (!c->decode && value > maximum) { c->error = -E2BIG; return 0; }
    uint32_t wire = c->decode ? 0 : (uint32_t)value; transfer(c, &wire, sizeof(wire));
    if (wire > maximum) { c->error = -EPROTO; return 0; }
    return c->error ? 0 : wire;
}
static void *blob(struct codec *c, const void *data, size_t size, size_t maximum, size_t *out_size)
{
    size_t length = count(c, size, maximum); void *result = (void *)data;
    if (c->decode) {
        if (length > c->size - c->position) { c->error = -EPROTO; return NULL; }
        result = length ? allocate(c, length) : NULL;
    }
    transfer(c, result, length); if (out_size) *out_size = length; return result;
}
static void *optional(struct codec *c, const void *value, size_t size)
{
    uint32_t present = value != NULL;
    if (c->decode) present = 0;
    transfer(c, &present, sizeof(present));
    if (present > 1) { c->error = -EPROTO; return NULL; }
    return !present || c->error ? NULL : c->decode ? allocate(c, size) : (void *)value;
}
static void sensor(struct codec *c, const cc_sensor_model **model)
{
    const char *name = NULL; size_t length = 0;
    if (!c->decode && *model) {
        const cc_sensor_properties *properties = cc_sensor_get_properties(*model);
        if (!properties || !properties->model) { c->error = -EINVAL; return; }
        name = properties->model; length = strlen(name) + 1;
    }
    char *value = blob(c, name, length, 256, &length);
    if (!c->decode || c->error) return;
    *model = NULL;
    if (length && (!value || length < 2 || value[length - 1] || memchr(value, 0, length - 1) ||
                   cc_sensor_lookup(value, model))) c->error = -EPROTO;
}
static void control(struct codec *c, cc_control *value)
{
    FIELD(value, id); FIELD(value, value);
    size_t size = value->size;
    struct bcm2835_isp_lens_shading lens; const void *input = value->data;
    bool shading = value->id == V4L2_CID_USER_BCM2835_ISP_LENS_SHADING;
    if (!c->decode && shading && !c->error) {
        if (size != sizeof(lens) || !input) { c->error = -EINVAL; return; }
        memcpy(&lens, input, sizeof(lens)); input = &lens;
        if (lens.enabled) {
            if (!c->allow_fds) { c->error = -ENOTSUP; return; }
            if (lens.dmabuf < 0) { c->error = -EBADF; return; }
            if (c->owner->num_fds == CC_IPA_WIRE_MAX_FDS) { c->error = -E2BIG; return; }
            c->owner->fds[c->owner->num_fds] = lens.dmabuf;
            lens.dmabuf = (int32_t)c->owner->num_fds++;
        } else lens.dmabuf = -1;
    }
    void *data = blob(c, input, size, CC_IPA_WIRE_MAX_DATA, &size);
    if (c->decode) { value->data = data; value->size = (uint32_t)size; }
    if (c->decode && shading && !c->error) {
        if (size != sizeof(lens)) { c->error = -EPROTO; return; }
        memcpy(&lens, data, sizeof(lens));
        if (lens.enabled) {
            if (lens.dmabuf < 0 || (uint32_t)lens.dmabuf >= c->owner->num_fds) { c->error = -EPROTO; return; }
            c->used_fds[lens.dmabuf] = true; lens.dmabuf = c->owner->fds[lens.dmabuf];
        } else if (lens.dmabuf != -1) { c->error = -EPROTO; return; }
        memcpy(data, &lens, sizeof(lens));
    }
}
static void controls(struct codec *c, cc_control *values, size_t size)
{
    for (size_t i = 0; i < size && !c->error; ++i) control(c, &values[i]);
}
static const cc_control *control_array(struct codec *c, const cc_control *values, size_t size, size_t *out_size)
{
    size = count(c, size, CC_IPA_MAX_CONTROLS); cc_control *result = (cc_control *)values;
    if (c->decode) result = size ? allocate(c, size * sizeof(*result)) : NULL;
    if (size && !result && !c->error) c->error = -EINVAL;
    controls(c, result, size); *out_size = size; return result;
}
static void control_info(struct codec *c, cc_control_info *value)
{
    FIELD(value, id); FIELD(value, type); FIELD(value, flags); FIELD(value, name);
    FIELD(value, minimum); FIELD(value, maximum); FIELD(value, default_value); FIELD(value, step);
    FIELD(value, elem_size); FIELD(value, elems); FIELD(value, nr_of_dims); FIELD(value, dims);
}
static void timing(struct codec *c, cc_sensor_timing *value)
{
    FIELD(value, pixel_rate); FIELD(value, width); FIELD(value, height); FIELD(value, hblank); FIELD(value, vblank);
}
static void timing_limits(struct codec *c, cc_sensor_timing_limits *value)
{
    FIELD(value, pixel_rate); FIELD(value, width); FIELD(value, height);
    FIELD(value, min_hblank); FIELD(value, max_hblank); FIELD(value, hblank_step);
    FIELD(value, min_vblank); FIELD(value, max_vblank); FIELD(value, vblank_step);
    FIELD(value, min_exposure); FIELD(value, exposure_step);
}
static void af_controls(struct codec *c, cc_af_controls *value)
{
    FIELD(value, flags); FIELD(value, mode); FIELD(value, range); FIELD(value, speed); FIELD(value, pause);
    FIELD(value, use_windows); FIELD(value, num_windows); FIELD(value, windows); FIELD(value, lens_position);
}
static void af_status(struct codec *c, cc_af_status *value)
{
    FIELD(value, flags); FIELD(value, state); FIELD(value, pause_state); FIELD(value, lens_setting); FIELD(value, lens_position);
}
static void af_tuning(struct codec *c, cc_af_tuning *value)
{
    FIELD(value, ranges);
    for (size_t i = 0; i < 2; ++i) {
        cc_af_speed_tuning *speed = &value->speeds[i];
        FIELD(speed, step_coarse); FIELD(speed, step_fine); FIELD(speed, contrast_ratio); FIELD(speed, retrigger_ratio);
        FIELD(speed, retrigger_delay); FIELD(speed, pdaf_gain); FIELD(speed, pdaf_squelch); FIELD(speed, max_slew);
        FIELD(speed, pdaf_frames); FIELD(speed, dropout_frames); FIELD(speed, step_frames);
    }
    FIELD(value, confidence_epsilon); FIELD(value, confidence_threshold); FIELD(value, confidence_clip);
    FIELD(value, skip_frames); FIELD(value, check_for_ir); FIELD(value, num_map_points); FIELD(value, map);
}
static void vc4_tuning(struct codec *c, cc_vc4_tuning *value)
{
    FIELD(value, black); FIELD(value, gamma); FIELD(value, ccm); FIELD(value, geq); FIELD(value, denoise);
    FIELD(value, cdn); FIELD(value, sharpen); FIELD(value, dpc); FIELD(value, lens_shading);
    FIELD(value, lens_shading_enabled); FIELD(value, colour_temperature);
}
static void configuration(struct codec *c, cc_ipa_configuration *value)
{
    FIELD(value, pipeline); FIELD(value, statistics_fourcc); FIELD(value, source); FIELD(value, sensor_format);
    sensor(c, &value->sensor); timing(c, &value->timing);
    value->num_sensor_controls = (uint32_t)count(c, value->num_sensor_controls, CC_IPA_MAX_CONTROLS);
    for (size_t i = 0; i < value->num_sensor_controls && !c->error; ++i) {
        control_info(c, &value->sensor_controls[i]); control(c, &value->initial_controls[i]);
        BLOCK(&value->sensor_delays[i]);
    }
    control_info(c, &value->red_balance); control_info(c, &value->blue_balance);
    control_info(c, &value->digital_gain); control_info(c, &value->lens);
    FIELD(value, embedded); timing_limits(c, &value->timing_limits); FIELD(value, geometry);
}
static void request(struct codec *c, cc_ipa_request *value)
{
    FIELD(value, sequence); FIELD(value, cookie); timing(c, &value->timing);
    value->sensor_controls = control_array(c, value->sensor_controls, value->num_sensor_controls, &value->num_sensor_controls);
    cc_af_controls *af = optional(c, value->af, sizeof(*af)); value->af = af;
    if (af) af_controls(c, af);
    timing_limits(c, &value->timing_limits);
}
static void settings(struct codec *c, cc_ipa_settings *value)
{
    FIELD(value, flags);
    value->num_sensor_controls = (uint32_t)count(c, value->num_sensor_controls, CC_IPA_MAX_CONTROLS);
    controls(c, value->sensor_controls, value->num_sensor_controls);
    value->num_isp_controls = (uint32_t)count(c, value->num_isp_controls, CC_IPA_MAX_CONTROLS);
    controls(c, value->isp_controls, value->num_isp_controls);
    FIELD(value, frontend); FIELD(value, backend); af_status(c, &value->af); vc4_tuning(c, &value->vc4);
}
static void statistics(struct codec *c, cc_ipa_statistics *value)
{
    FIELD(value, flags); FIELD(value, luminance); FIELD(value, red); FIELD(value, green); FIELD(value, blue); FIELD(value, colour_samples);
}
static void frame(struct codec *c, cc_ipa_frame *value)
{
    FIELD(value, sequence); FIELD(value, cookie); FIELD(value, timestamp_ns);
    FIELD(&value->sensor, frame_sequence); FIELD(&value->sensor, cookie);
    value->sensor.count = (uint32_t)count(c, value->sensor.count, CC_SENSOR_MAX_CONTROLS);
    controls(c, value->sensor.controls, value->sensor.count);
    statistics(c, &value->statistics); FIELD(value, statistics_fourcc);
    value->statistics_data = blob(c, value->statistics_data, value->statistics_size, CC_IPA_WIRE_MAX_DATA, &value->statistics_size);
    value->isp_controls = control_array(c, value->isp_controls, value->num_isp_controls, &value->num_isp_controls);
    struct pisp_be_config *backend = optional(c, value->backend, sizeof(*backend)); value->backend = backend;
    if (backend) BLOCK(backend);
    af_status(c, &value->af);
    value->embedded_data = blob(c, value->embedded_data, value->embedded_size, CC_IPA_WIRE_MAX_DATA, &value->embedded_size);
    value->request_sensor_controls = control_array(c, value->request_sensor_controls, value->num_request_sensor_controls, &value->num_request_sensor_controls);
    value->settings_sensor_controls = control_array(c, value->settings_sensor_controls, value->num_settings_sensor_controls, &value->num_settings_sensor_controls);
    cc_vc4_tuning *vc4 = optional(c, value->vc4, sizeof(*vc4)); value->vc4 = vc4;
    if (vc4) vc4_tuning(c, vc4);
    FIELD(value, source_sequence); FIELD(value, untrusted);
}
static void metadata(struct codec *c, cc_ipa_metadata *value)
{
    FIELD(value, flags); FIELD(value, exposure_ns); FIELD(value, analogue_gain); FIELD(value, red_gain);
    FIELD(value, blue_gain); FIELD(value, luminance); af_status(c, &value->af);
    cc_sensor_embedded_status *status = &value->sensor_status;
    FIELD(status, flags); FIELD(status, exposure_lines); FIELD(status, analogue_gain_code);
    FIELD(status, line_length_pck); FIELD(status, frame_length_lines); FIELD(status, analogue_gain); FIELD(status, temperature_c);
    FIELD(&value->hdr, histogram); FIELD(&value->hdr, counted); FIELD(&value->hdr, average);
    FIELD(value, colour_temperature); FIELD(value, lux); FIELD(value, frame_duration_ns);
}
static void reference(struct codec *c, cc_ipa_reference_tuning *value)
{
    FIELD(value, flags); FIELD(value, target_luminance); FIELD(value, exposure_speed); FIELD(value, white_balance_speed);
    FIELD(value, min_colour_gain); FIELD(value, max_colour_gain); FIELD(value, max_exposure_ns);
    sensor(c, &value->sensor); cc_af_tuning *af = optional(c, value->af, sizeof(*af)); value->af = af;
    if (af) af_tuning(c, af);
    FIELD(value, min_frame_duration_ns); FIELD(value, max_frame_duration_ns);
}
static size_t object_size(enum cc_ipa_wire_kind kind)
{
    switch (kind) {
    case CC_IPA_WIRE_CONFIGURATION: return sizeof(cc_ipa_configuration);
    case CC_IPA_WIRE_REQUEST: return sizeof(cc_ipa_request);
    case CC_IPA_WIRE_SETTINGS: return sizeof(cc_ipa_settings);
    case CC_IPA_WIRE_FRAME: return sizeof(cc_ipa_frame);
    case CC_IPA_WIRE_METADATA: return sizeof(cc_ipa_metadata);
    case CC_IPA_WIRE_REFERENCE: return sizeof(cc_ipa_reference_tuning);
    case CC_IPA_WIRE_VC4: return sizeof(cc_ipa_vc4_options);
    default: return 0;
    }
}
static void visit(struct codec *c, enum cc_ipa_wire_kind kind)
{
    const uint32_t expected[] = {UINT32_C(0x43495041), CC_IPA_WIRE_VERSION, (uint32_t)kind,
        CC_IPA_ABI_VERSION, sizeof(struct pisp_fe_config), sizeof(struct pisp_be_config)};
    uint32_t header[6]; memcpy(header, expected, sizeof(header)); transfer(c, header, sizeof(header));
    if (c->error) return;
    if (memcmp(header, expected, sizeof(header))) { c->error = -EPROTONOSUPPORT; return; }
    uint32_t num_fds = c->owner->num_fds; size_t fd_count_offset = c->position;
    transfer(c, &num_fds, sizeof(num_fds));
    if (c->decode && num_fds != c->owner->num_fds) { c->error = -EPROTO; return; }
    union wire_object *value = &c->owner->object;
    switch (kind) {
    case CC_IPA_WIRE_CONFIGURATION: configuration(c, &value->configuration); break;
    case CC_IPA_WIRE_REQUEST: request(c, &value->request); break;
    case CC_IPA_WIRE_SETTINGS: settings(c, &value->settings); break;
    case CC_IPA_WIRE_FRAME: frame(c, &value->frame); break;
    case CC_IPA_WIRE_METADATA: metadata(c, &value->metadata); break;
    case CC_IPA_WIRE_REFERENCE: reference(c, &value->reference); break;
    case CC_IPA_WIRE_VC4: FIELD(&value->vc4, mains_hz); FIELD(&value->vc4, red_gain); FIELD(&value->vc4, blue_gain); break;
    }
    if (!c->error && !c->decode) memcpy(c->output + fd_count_offset, &c->owner->num_fds, sizeof(uint32_t));
    if (!c->error && c->decode) for (size_t i = 0; i < c->owner->num_fds; ++i)
        if (!c->used_fds[i]) { c->error = -EPROTO; break; }
}
void cc_ipa_wire_destroy(cc_ipa_wire *wire)
{
    if (!wire) return;
    struct allocation *allocation = wire->allocations;
    while (allocation) { struct allocation *next = allocation->next; free(allocation->data); free(allocation); allocation = next; }
    if (wire->owned_fds) for (size_t i = 0; i < wire->num_fds; ++i) close(wire->fds[i]);
    free(wire);
}
const void *cc_ipa_wire_object(const cc_ipa_wire *wire) { return wire ? &wire->object : NULL; }
static int encode(enum cc_ipa_wire_kind kind, const void *object, void **data, size_t *size, int *fds, uint32_t *num_fds)
{
    if (data) *data = NULL;
    if (size) *size = 0;
    if (num_fds) *num_fds = 0;
    size_t length = object_size(kind);
    if (!object || !data || !size || !length) return -EINVAL;
    cc_ipa_wire *wire = calloc(1, sizeof(*wire)); if (!wire) return -ENOMEM;
    memcpy(&wire->object, object, length); struct codec codec = {.owner = wire, .allow_fds = fds != NULL}; visit(&codec, kind);
    if (!codec.error && fds && num_fds) { memcpy(fds, wire->fds, wire->num_fds * sizeof(*fds)); *num_fds = wire->num_fds; }
    cc_ipa_wire_destroy(wire);
    if (codec.error) { free(codec.output); return codec.error; }
    *data = codec.output; *size = codec.position; return 0;
}
int cc_ipa_wire_encode(enum cc_ipa_wire_kind kind, const void *object, void **data, size_t *size)
{
    return encode(kind, object, data, size, NULL, NULL);
}
int cc_ipa_wire_encode_fds(enum cc_ipa_wire_kind kind, const void *object, void **data, size_t *size,
                           int fds[CC_IPA_WIRE_MAX_FDS], uint32_t *num_fds)
{
    if (!fds || !num_fds) { if (data) *data = NULL; if (size) *size = 0; return -EINVAL; }
    return encode(kind, object, data, size, fds, num_fds);
}
static int decode(enum cc_ipa_wire_kind kind, const void *data, size_t size, const int *fds, uint32_t num_fds, cc_ipa_wire **out)
{
    if (!out) return -EINVAL;
    *out = NULL;
    if (!data || !object_size(kind) || num_fds > CC_IPA_WIRE_MAX_FDS || (num_fds && !fds)) return -EINVAL;
    if (size > CC_IPA_WIRE_MAX_DATA) return -E2BIG;
    cc_ipa_wire *wire = calloc(1, sizeof(*wire)); if (!wire) return -ENOMEM;
    wire->owned_fds = num_fds != 0;
    for (size_t i = 0; i < num_fds; ++i) {
        int fd = fcntl(fds[i], F_DUPFD_CLOEXEC, 0);
        if (fd < 0) { int error = -errno; cc_ipa_wire_destroy(wire); return error; }
        wire->fds[wire->num_fds++] = fd;
    }
    struct codec codec = {.owner = wire, .input = data, .size = size, .decode = true}; visit(&codec, kind);
    if (!codec.error && codec.position != size) codec.error = -EPROTO;
    if (codec.error) { cc_ipa_wire_destroy(wire); return codec.error; }
    *out = wire; return 0;
}
int cc_ipa_wire_decode(enum cc_ipa_wire_kind kind, const void *data, size_t size, cc_ipa_wire **out)
{
    return decode(kind, data, size, NULL, 0, out);
}
int cc_ipa_wire_decode_fds(enum cc_ipa_wire_kind kind, const void *data, size_t size,
                           const int *fds, uint32_t num_fds, cc_ipa_wire **out)
{
    return decode(kind, data, size, fds, num_fds, out);
}
