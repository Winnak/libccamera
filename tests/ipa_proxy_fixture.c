/* SPDX-License-Identifier: LGPL-2.1-or-later */
#define _GNU_SOURCE
#include "ipa_proxy_fixture.h"
#include <ccamera/ipa_module.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>

CC_API const cc_ipa_module_info cc_ipa_module_descriptor = {
    CC_IPA_MODULE_API_VERSION, CC_IPA_ABI_VERSION, 1, "ProxyTest", "proxy-test"
};
struct context { uint32_t mode; unsigned char payload[PROXY_PAYLOAD_SIZE]; struct bcm2835_isp_lens_shading lens; };
static void trace(char event)
{
    const char *path = getenv("CCAMERA_TEST_PROXY_TRACE"); if (!path) return;
    FILE *file = fopen(path, "a"); if (file) { (void)fprintf(file, "%ld %c\n", (long)getpid(), event); (void)fclose(file); }
}
__attribute__((constructor)) static void loaded(void) { trace('L'); }
__attribute__((destructor)) static void unloaded(void) { trace('U'); }
static int configure(void *context, const cc_ipa_configuration *configuration)
{
    struct context *c = context; trace('c');
    if (c->mode == 2) return -EIO;
    if (!configuration->sensor || strcmp(cc_sensor_get_properties(configuration->sensor)->model, "imx708") ||
        configuration->source.width != 640 || configuration->num_sensor_controls != 1 ||
        configuration->initial_controls[0].value != 123 || configuration->sensor_delays[0].frames != 2) return -EINVAL;
    return 0;
}
static int start(void *context) { trace('s'); return ((struct context *)context)->mode == 3 ? -EIO : 0; }
static void stop(void *context)
{
    trace('t'); if (((struct context *)context)->mode == 12) for (;;) pause();
}
static int prepare(void *context, const cc_ipa_request *request, cc_ipa_settings *settings)
{
    struct context *c = context; trace('p');
    if (c->mode == 4) return -EIO;
    if (c->mode == 6) _exit(23);
    if (c->mode == 7) for (;;) pause();
    if (c->mode == 8) { settings->flags = UINT32_MAX; return 0; }
    if (request->sequence != UINT64_MAX || request->cookie != 456 || request->num_sensor_controls != 1 ||
        request->sensor_controls[0].value != 777 || !request->af || request->af->mode != CC_AF_CONTINUOUS) return -EINVAL;
    *settings = (cc_ipa_settings){.flags = CC_IPA_FRONTEND | CC_IPA_BACKEND | CC_IPA_LENS,
        .num_sensor_controls = 1, .sensor_controls = {{.id = 1234, .value = 888}},
        .num_isp_controls = 2, .isp_controls = {{.id = 5678, .data = c->payload,
            .size = c->mode == 10 ? 16U * 1024U * 1024U + 1U : PROXY_PAYLOAD_SIZE},
            {.id = V4L2_CID_USER_BCM2835_ISP_LENS_SHADING, .data = &c->lens, .size = sizeof(c->lens)}},
        .af = {.flags = CC_AF_LENS_VALID, .state = CC_AF_FOCUSED, .lens_setting = 500, .lens_position = 1.5}};
    settings->frontend.global.enables = 123; settings->backend.dirty_flags_rgb = 456; return 0;
}
static int process(void *context, const cc_ipa_frame *frame, cc_ipa_metadata *metadata)
{
    struct context *c = context; trace('f');
    if (c->mode == 5) return -EIO;
    if (c->mode == 9) { metadata->flags = UINT32_MAX; return 0; }
    if (frame->sequence != UINT64_MAX || frame->cookie != 456 || frame->statistics_size != PROXY_PAYLOAD_SIZE ||
        frame->embedded_size != PROXY_PAYLOAD_SIZE || frame->num_isp_controls != 2 ||
        frame->isp_controls[0].size != PROXY_PAYLOAD_SIZE || frame->num_request_sensor_controls != 1 ||
        frame->request_sensor_controls[0].value != 777 || frame->num_settings_sensor_controls != 1 ||
        frame->settings_sensor_controls[0].value != 888 || frame->sensor.count != 1 || frame->sensor.controls[0].value != 999 ||
        !frame->backend || frame->backend->dirty_flags_rgb != 456 || !frame->vc4 || frame->vc4->colour_temperature != 4500) return -EINVAL;
    if (frame->isp_controls[1].id != V4L2_CID_USER_BCM2835_ISP_LENS_SHADING || frame->isp_controls[1].size != sizeof(c->lens)) return -EINVAL;
    struct bcm2835_isp_lens_shading lens; memcpy(&lens, frame->isp_controls[1].data, sizeof(lens)); char text[19];
    if (!lens.enabled || pread(lens.dmabuf, text, sizeof(text), 0) != sizeof(text) || memcmp(text, "lens shading table", sizeof(text))) return -EINVAL;
    for (size_t i = 0; i < PROXY_PAYLOAD_SIZE; ++i)
        if (((const unsigned char *)frame->statistics_data)[i] != (unsigned char)i ||
            ((const unsigned char *)frame->embedded_data)[i] != (unsigned char)(i + 1U) ||
            ((const unsigned char *)frame->isp_controls[0].data)[i] != (unsigned char)(i + 2U)) return -EINVAL;
    *metadata = (cc_ipa_metadata){.flags = CC_IPA_METADATA_AE | CC_IPA_METADATA_AWB | CC_IPA_METADATA_AF,
        .exposure_ns = 1234567, .analogue_gain = 2, .luminance = 0.25, .red_gain = 1.5, .blue_gain = 2.5,
        .af = frame->af}; return 0;
}
static void destroy(void *context)
{
    struct context *c = context; trace('D'); if (c->mode == 13) for (;;) pause(); close(c->lens.dmabuf); free(context);
}
CC_API int cc_ipa_module_create_v1(uint32_t abi, size_t ops_size, const void *data, size_t size, cc_ipa_ops *ops, void **context)
{
    trace('C');
    if (abi != CC_IPA_ABI_VERSION || ops_size != sizeof(*ops) || size != sizeof(struct proxy_fixture_init)) return -EINVAL;
    struct proxy_fixture_init init; memcpy(&init, data, sizeof(init));
    struct stat user, net;
    if (init.parent_pid == getpid() || stat("/proc/self/ns/user", &user) || stat("/proc/self/ns/net", &net) ||
        init.user_namespace == user.st_ino || init.net_namespace == net.st_ino) return -EPROTO;
    if (init.mode == 1) return -ERANGE;
    if (init.mode == 11) _exit(24);
    if (init.mode == 14) for (;;) pause();
    struct context *c = calloc(1, sizeof(*c)); if (!c) return -ENOMEM;
    c->lens = (struct bcm2835_isp_lens_shading){.enabled = 1, .dmabuf = memfd_create("proxy-shading", MFD_CLOEXEC)};
    if (c->lens.dmabuf < 0) { int error = -errno; free(c); return error; }
    if (write(c->lens.dmabuf, "lens shading table", 19) != 19) { int error = -errno; close(c->lens.dmabuf); free(c); return error ? error : -EIO; }
    c->mode = init.mode; for (size_t i = 0; i < PROXY_PAYLOAD_SIZE; ++i) c->payload[i] = (unsigned char)(i + 2U);
    *ops = (cc_ipa_ops){.abi_version = CC_IPA_ABI_VERSION, .size = sizeof(*ops), .configure = configure,
        .start = start, .stop = stop, .prepare = prepare, .process = process, .destroy = destroy,
        .capabilities = CC_IPA_CAP_LENS | CC_IPA_CAP_SENSOR_TIMING | CC_IPA_CAP_SENSOR_GEOMETRY};
    *context = c; return 0;
}
