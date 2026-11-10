/* SPDX-License-Identifier: LGPL-2.1-or-later */
#define _GNU_SOURCE
#include "ipa_wire.h"
#include <ccamera/capture.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
#define OK(x) CHECK((x) == 0)
#define ERROR(x,e) CHECK((x) == -(e))
static const char *corpus;
static cc_ipa_wire *roundtrip(enum cc_ipa_wire_kind kind, const void *object)
{
    void *data = NULL, *again = NULL; size_t size = 0, again_size = 0;
    OK(cc_ipa_wire_encode(kind, object, &data, &size)); CHECK(data && size);
    if (corpus) {
        char path[4096]; int length = snprintf(path, sizeof(path), "%s/%u.bin", corpus, (unsigned)kind);
        CHECK(length > 0 && (size_t)length < sizeof(path)); FILE *file = fopen(path, "wb"); CHECK(file);
        CHECK(fputc((unsigned char)kind, file) != EOF && fwrite(data, 1, size, file) == size); OK(fclose(file));
    }
    cc_ipa_wire *wire = NULL; OK(cc_ipa_wire_decode(kind, data, size, &wire)); CHECK(wire && cc_ipa_wire_object(wire));
    OK(cc_ipa_wire_encode(kind, cc_ipa_wire_object(wire), &again, &again_size));
    CHECK(size == again_size && !memcmp(data, again, size)); free(again);
    /* Every truncation, unaligned input and appended garbage. */
    for (size_t i = 0; i < size; ++i) {
        cc_ipa_wire *bad = (void *)(uintptr_t)1; CHECK(cc_ipa_wire_decode(kind, data, i, &bad) < 0); CHECK(!bad);
    }
    unsigned char *unaligned = malloc(size + 1); CHECK(unaligned); memcpy(unaligned + 1, data, size);
    cc_ipa_wire *copy = NULL; OK(cc_ipa_wire_decode(kind, unaligned + 1, size, &copy)); cc_ipa_wire_destroy(copy);
    memcpy(unaligned, data, size); unaligned[size] = 0; ERROR(cc_ipa_wire_decode(kind, unaligned, size + 1, &copy), EPROTO);
    for (size_t i = 0; i < 24; i += 4) {
        unaligned[i] ^= 1; ERROR(cc_ipa_wire_decode(kind, unaligned, size, &copy), EPROTONOSUPPORT); CHECK(!copy); unaligned[i] ^= 1;
    }
    free(unaligned); free(data); return wire;
}
static void pointer_inputs(void)
{
    const cc_sensor_model *sensor; OK(cc_sensor_lookup("imx708", &sensor));
    static const cc_control control = {123, -99, NULL, 0};
    static const cc_af_controls af = {.flags = CC_AF_SET_MODE, .mode = CC_AF_CONTINUOUS};
    cc_ipa_request request = {.sequence = UINT64_MAX, .cookie = 456, .sensor_controls = &control, .num_sensor_controls = 1,
        .af = &af, .timing = {90000000, 1920, 1080, 20, 30}};
    cc_ipa_wire *wire = roundtrip(CC_IPA_WIRE_REQUEST, &request); const cc_ipa_request *decoded = cc_ipa_wire_object(wire);
    CHECK(decoded->sequence == UINT64_MAX && decoded->cookie == 456 && decoded->num_sensor_controls == 1);
    CHECK(decoded->sensor_controls != &control && decoded->sensor_controls[0].value == -99);
    CHECK(decoded->af != &af && decoded->af->mode == CC_AF_CONTINUOUS); cc_ipa_wire_destroy(wire);
    cc_af_tuning tuning; cc_af_reference_tuning(&tuning);
    cc_ipa_reference_tuning reference = {.flags = CC_IPA_REFERENCE_AF, .sensor = sensor, .af = &tuning,
        .min_frame_duration_ns = 123, .max_frame_duration_ns = 456};
    wire = roundtrip(CC_IPA_WIRE_REFERENCE, &reference); const cc_ipa_reference_tuning *remote = cc_ipa_wire_object(wire);
    CHECK(remote->sensor == sensor && remote->af && remote->af != &tuning);
    CHECK(remote->af->num_map_points == tuning.num_map_points && !memcmp(remote->af->map, tuning.map, sizeof(tuning.map)));
    CHECK(remote->min_frame_duration_ns == 123 && remote->max_frame_duration_ns == 456); cc_ipa_wire_destroy(wire);
    cc_ipa_configuration configuration = {.pipeline = CC_IPA_CFE, .sensor = sensor, .num_sensor_controls = 1};
    configuration.sensor_controls[0] = (cc_control_info){.id = 123, .minimum = -11, .maximum = 33};
    configuration.initial_controls[0] = control; configuration.sensor_delays[0] = (cc_sensor_control_delay){123, 2, 1};
    wire = roundtrip(CC_IPA_WIRE_CONFIGURATION, &configuration); const cc_ipa_configuration *config = cc_ipa_wire_object(wire);
    CHECK(config->sensor == sensor && config->sensor_controls[0].minimum == -11 && config->sensor_delays[0].frames == 2);
    cc_ipa_wire_destroy(wire);
}
static void payloads(void)
{
    unsigned char payload[] = {1, 2, 3, 4, 5}; cc_control isp = {.id = 456, .data = payload, .size = sizeof(payload)};
    cc_ipa_settings *settings = calloc(1, sizeof(*settings)); CHECK(settings);
    settings->num_isp_controls = 1; settings->isp_controls[0] = isp;
    settings->flags = CC_IPA_FRONTEND | CC_IPA_BACKEND | CC_IPA_VC4_TUNING;
    settings->frontend.global.enables = 123; settings->backend.dirty_flags_rgb = 456;
    settings->vc4.lens_shading[2][191] = 7.5;
    cc_ipa_wire *wire = roundtrip(CC_IPA_WIRE_SETTINGS, settings); const cc_ipa_settings *decoded = cc_ipa_wire_object(wire);
    CHECK(decoded->isp_controls[0].data != payload && decoded->isp_controls[0].size == sizeof(payload));
    CHECK(!memcmp(decoded->isp_controls[0].data, payload, sizeof(payload)) && decoded->frontend.global.enables == 123);
    CHECK(decoded->backend.dirty_flags_rgb == 456 && decoded->vc4.lens_shading[2][191] == 7.5); cc_ipa_wire_destroy(wire);
    const char stats[] = "native statistics bytes", embedded[] = "native embedded bytes";
    cc_control scalar = {.id = 123, .value = -88};
    cc_ipa_frame frame = {.sequence = 123, .cookie = 456, .timestamp_ns = UINT64_MAX, .statistics_data = stats,
        .statistics_size = sizeof(stats), .embedded_data = embedded, .embedded_size = sizeof(embedded),
        .isp_controls = &isp, .num_isp_controls = 1, .request_sensor_controls = &scalar, .num_request_sensor_controls = 1,
        .settings_sensor_controls = &scalar, .num_settings_sensor_controls = 1, .backend = &settings->backend,
        .vc4 = &settings->vc4, .source_sequence = UINT32_MAX, .untrusted = 1};
    frame.sensor.count = 1; frame.sensor.controls[0] = scalar;
    wire = roundtrip(CC_IPA_WIRE_FRAME, &frame); const cc_ipa_frame *remote = cc_ipa_wire_object(wire);
    CHECK(remote->statistics_data != stats && remote->statistics_size == sizeof(stats) && !memcmp(remote->statistics_data, stats, sizeof(stats)));
    CHECK(remote->embedded_data != embedded && remote->embedded_size == sizeof(embedded) && !memcmp(remote->embedded_data, embedded, sizeof(embedded)));
    CHECK(remote->isp_controls[0].data != payload && !memcmp(remote->isp_controls[0].data, payload, sizeof(payload)));
    CHECK(remote->request_sensor_controls[0].value == -88 && remote->settings_sensor_controls[0].value == -88);
    CHECK(remote->sensor.controls[0].value == -88 && remote->source_sequence == UINT32_MAX && remote->untrusted == 1);
    CHECK(remote->backend != frame.backend && remote->backend->dirty_flags_rgb == 456 && remote->vc4 != frame.vc4);
    CHECK(remote->vc4->lens_shading[2][191] == 7.5); free(settings); cc_ipa_wire_destroy(wire);
}
static void limits(void)
{
    cc_ipa_wire *wire = (void *)(uintptr_t)1; char byte = 0; void *data = (void *)(uintptr_t)1; size_t size = 123;
    ERROR(cc_ipa_wire_decode(0, &byte, 1, &wire), EINVAL); CHECK(!wire);
    ERROR(cc_ipa_wire_decode(CC_IPA_WIRE_FRAME, &byte, CC_IPA_WIRE_MAX_DATA + 1U, &wire), E2BIG); CHECK(!wire);
    cc_ipa_request request = {.num_sensor_controls = CC_IPA_MAX_CONTROLS + 1U};
    ERROR(cc_ipa_wire_encode(CC_IPA_WIRE_REQUEST, &request, &data, &size), E2BIG); CHECK(!data && !size);
    request.num_sensor_controls = 1; ERROR(cc_ipa_wire_encode(CC_IPA_WIRE_REQUEST, &request, &data, &size), EINVAL);
    cc_ipa_frame frame = {.statistics_size = 1}; ERROR(cc_ipa_wire_encode(CC_IPA_WIRE_FRAME, &frame, &data, &size), EINVAL);
    frame.statistics_size = CC_IPA_WIRE_MAX_DATA + 1U;
    ERROR(cc_ipa_wire_encode(CC_IPA_WIRE_FRAME, &frame, &data, &size), E2BIG);
    unsigned char *large = calloc(1, CC_IPA_WIRE_MAX_DATA); CHECK(large);
    frame.statistics_data = large; frame.statistics_size = CC_IPA_WIRE_MAX_DATA;
    ERROR(cc_ipa_wire_encode(CC_IPA_WIRE_FRAME, &frame, &data, &size), E2BIG); CHECK(!data && !size);
    frame.statistics_size = 131072; OK(cc_ipa_wire_encode(CC_IPA_WIRE_FRAME, &frame, &data, &size));
    CHECK(size > 131072); OK(cc_ipa_wire_decode(CC_IPA_WIRE_FRAME, data, size, &wire));
    const cc_ipa_frame *decoded = cc_ipa_wire_object(wire); CHECK(decoded->statistics_size == 131072);
    cc_ipa_wire_destroy(wire); free(data); free(large);
    ERROR(cc_ipa_wire_encode(CC_IPA_WIRE_FRAME, NULL, &data, &size), EINVAL); CHECK(!data && !size);
    CHECK(!cc_ipa_wire_object(NULL)); cc_ipa_wire_destroy(NULL);
}
static void descriptors(void)
{
    int pipes[2]; OK(pipe2(pipes, O_CLOEXEC | O_NONBLOCK));
    struct bcm2835_isp_lens_shading lens = {.enabled = 1, .dmabuf = pipes[1]};
    cc_ipa_settings settings = {.num_isp_controls = 1,
        .isp_controls = {{.id = V4L2_CID_USER_BCM2835_ISP_LENS_SHADING, .data = &lens, .size = sizeof(lens)}}};
    void *data = NULL; size_t size = 0; int fds[CC_IPA_WIRE_MAX_FDS]; uint32_t count = 0;
    ERROR(cc_ipa_wire_encode(CC_IPA_WIRE_SETTINGS, &settings, &data, &size), ENOTSUP);
    OK(cc_ipa_wire_encode_fds(CC_IPA_WIRE_SETTINGS, &settings, &data, &size, fds, &count)); CHECK(count == 1 && fds[0] == pipes[1]);
    cc_ipa_wire *wire = NULL; ERROR(cc_ipa_wire_decode(CC_IPA_WIRE_SETTINGS, data, size, &wire), EPROTO); CHECK(!wire);
    struct bcm2835_isp_lens_shading token = lens; token.dmabuf = 0;
    unsigned char *record = memmem(data, size, &token, sizeof(token)); CHECK(record);
    int32_t bad_index = -1; memcpy(record + offsetof(struct bcm2835_isp_lens_shading, dmabuf), &bad_index, sizeof(bad_index));
    ERROR(cc_ipa_wire_decode_fds(CC_IPA_WIRE_SETTINGS, data, size, fds, count, &wire), EPROTO); CHECK(!wire);
    CHECK(fcntl(pipes[1], F_GETFD) & FD_CLOEXEC); bad_index = 0;
    memcpy(record + offsetof(struct bcm2835_isp_lens_shading, dmabuf), &bad_index, sizeof(bad_index));
    OK(cc_ipa_wire_decode_fds(CC_IPA_WIRE_SETTINGS, data, size, fds, count, &wire));
    const cc_ipa_settings *decoded = cc_ipa_wire_object(wire); struct bcm2835_isp_lens_shading received;
    memcpy(&received, decoded->isp_controls[0].data, sizeof(received)); CHECK(received.dmabuf != pipes[1] && (fcntl(received.dmabuf, F_GETFD) & FD_CLOEXEC));
    OK(close(pipes[1])); char byte; CHECK(read(pipes[0], &byte, 1) < 0 && errno == EAGAIN);
    cc_ipa_wire_destroy(wire); CHECK(read(pipes[0], &byte, 1) == 0); OK(close(pipes[0])); free(data);
    lens.enabled = 0; lens.dmabuf = 999;
    wire = roundtrip(CC_IPA_WIRE_SETTINGS, &settings); decoded = cc_ipa_wire_object(wire);
    memcpy(&received, decoded->isp_controls[0].data, sizeof(received)); CHECK(received.dmabuf == -1); cc_ipa_wire_destroy(wire);
    /* Request snapshots must own descriptor handles as well as payload bytes. */
    OK(pipe2(pipes, O_CLOEXEC | O_NONBLOCK)); lens.enabled = 1; lens.dmabuf = pipes[1];
    cc_request *request = NULL; OK(cc_request_create(1, &request));
    OK(cc_request_set_isp_controls(request, settings.isp_controls, 1));
    cc_control rollback[] = {settings.isp_controls[0], {.id = 1, .size = 1}};
    ERROR(cc_request_set_isp_controls(request, rollback, 2), EINVAL); CHECK(fcntl(pipes[1], F_GETFD) & FD_CLOEXEC);
    OK(close(pipes[1])); CHECK(read(pipes[0], &byte, 1) < 0 && errno == EAGAIN);
    OK(cc_request_set_isp_controls(request, NULL, 0)); CHECK(read(pipes[0], &byte, 1) == 0);
    OK(close(pipes[0])); OK(cc_request_destroy(request));
    OK(pipe2(pipes, O_CLOEXEC | O_NONBLOCK)); lens.dmabuf = pipes[1]; OK(cc_request_create(1, &request));
    int saved = dup(0); (void)close(0); OK(cc_request_set_isp_controls(request, settings.isp_controls, 1));
    CHECK(fcntl(0, F_GETFD) & FD_CLOEXEC); OK(cc_request_destroy(request)); CHECK(fcntl(0, F_GETFD) < 0 && errno == EBADF);
    if (saved >= 0) { CHECK(dup2(saved, 0) == 0); OK(close(saved)); }
    OK(close(pipes[0])); OK(close(pipes[1]));
}
int main(int argc, char **argv)
{
    CHECK(argc == 1 || (argc == 3 && !strcmp(argv[1], "--corpus")));
    if (argc == 3) corpus = argv[2];
    pointer_inputs(); payloads(); limits(); descriptors();
    cc_ipa_metadata metadata = {.flags = CC_IPA_METADATA_HDR, .exposure_ns = 123, .colour_temperature = 4500};
    metadata.hdr.histogram[127] = UINT32_MAX; metadata.hdr.counted = UINT64_MAX;
    cc_ipa_wire *wire = roundtrip(CC_IPA_WIRE_METADATA, &metadata); const cc_ipa_metadata *decoded = cc_ipa_wire_object(wire);
    CHECK(decoded->hdr.histogram[127] == UINT32_MAX && decoded->hdr.counted == UINT64_MAX); cc_ipa_wire_destroy(wire);
    cc_ipa_vc4_options options = {.mains_hz = 50, .red_gain = 1.5, .blue_gain = 2.5};
    wire = roundtrip(CC_IPA_WIRE_VC4, &options); const cc_ipa_vc4_options *vc4 = cc_ipa_wire_object(wire);
    CHECK(vc4->mains_hz == 50 && vc4->red_gain == 1.5 && vc4->blue_gain == 2.5); cc_ipa_wire_destroy(wire);
    puts("IPA wire deep-copy, pointer reconstruction, limits and malformed-input checks passed"); return 0;
}
