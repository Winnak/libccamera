/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <ccamera/ipa_module.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void trace(char event)
{
    const char *path = getenv("CCAMERA_TEST_MODULE_TRACE");
    if (!path) return;
    FILE *file = fopen(path, "a");
    if (file) { (void)fputc(event, file); (void)fclose(file); }
}
__attribute__((constructor)) static void loaded(void) { trace('L'); }
__attribute__((destructor)) static void unloaded(void) { trace('U'); }

#ifndef CC_IPA_FIXTURE_NO_INFO
__attribute__((visibility("default")))
const cc_ipa_module_info cc_ipa_module_descriptor = {
#ifdef CC_IPA_FIXTURE_BAD_API
    .module_api_version = 999,
#else
    .module_api_version = CC_IPA_MODULE_API_VERSION,
#endif
#ifdef CC_IPA_FIXTURE_BAD_ABI
    .ipa_abi_version = 999,
#else
    .ipa_abi_version = CC_IPA_ABI_VERSION,
#endif
#if defined(CC_IPA_FIXTURE_MANAGER_V4)
    .pipeline_version = 4,
#elif defined(CC_IPA_FIXTURE_MANAGER_V9)
    .pipeline_version = 9,
#else
    .pipeline_version = 7,
#endif
#ifdef CC_IPA_FIXTURE_BAD_PIPELINE
    .pipeline_name = "Test\001Pipeline",
#else
    .pipeline_name = "TestPipeline",
#endif
#ifdef CC_IPA_FIXTURE_BAD_NAME
    .name = "bad?module",
#elif defined(CC_IPA_FIXTURE_MANAGER_V4)
    .name = "alternate",
#else
    .name = "test-module",
#endif
};
#endif

#ifndef CC_IPA_FIXTURE_NO_FACTORY
struct context { uint32_t mode; uint64_t exposure; };
static int configure(void *context, const cc_ipa_configuration *configuration)
{
    struct context *c = context;
    trace('c');
    c->exposure = configuration->source.width;
    return c->mode == 6 ? 1 : 0;
}
static int start(void *context) { (void)context; trace('s'); return 0; }
static void stop(void *context) { (void)context; trace('t'); }
static int prepare(void *context, const cc_ipa_request *request, cc_ipa_settings *settings)
{
    (void)context; (void)request; (void)settings;
    trace('p'); return 0;
}
static int process(void *context, const cc_ipa_frame *frame, cc_ipa_metadata *metadata)
{
    struct context *c = context;
    trace('f');
    *metadata = (cc_ipa_metadata){.flags = CC_IPA_METADATA_AE, .exposure_ns = c->exposure,
        .analogue_gain = 1, .luminance = frame->statistics.luminance};
    return 0;
}
static void destroy(void *context) { trace('D'); free(context); }
#ifdef CC_IPA_FIXTURE_UNRESOLVED
extern void cc_ipa_fixture_missing_dependency(void);
#endif
#ifdef CC_IPA_FIXTURE_ORIGIN
extern int cc_ipa_fixture_dependency(void);
#endif
__attribute__((visibility("default")))
int cc_ipa_module_create_v1(uint32_t version, size_t ops_size, const void *data, size_t size,
                            cc_ipa_ops *ops, void **context)
{
#ifdef CC_IPA_FIXTURE_UNRESOLVED
    cc_ipa_fixture_missing_dependency();
#endif
#ifdef CC_IPA_FIXTURE_ORIGIN
    if (cc_ipa_fixture_dependency() != 81) return -EPROTO;
#endif
    if (version != CC_IPA_ABI_VERSION || ops_size != sizeof(*ops)) return -EPROTONOSUPPORT;
    if (size && size != sizeof(uint32_t)) return -EINVAL;
    uint32_t mode = 0;
    if (size) memcpy(&mode, data, sizeof(mode));
    trace('C');
    if (mode == 4) return 1;
    if (mode == 5) return -ERANGE;
    struct context *c = calloc(1, sizeof(*c));
    if (!c) return -ENOMEM;
    c->mode = mode;
    *ops = (cc_ipa_ops){.abi_version = CC_IPA_ABI_VERSION, .size = sizeof(*ops),
        .configure = configure, .start = start, .stop = stop, .prepare = prepare,
        .process = process, .destroy = destroy, .capabilities = CC_IPA_CAP_SENSOR_GEOMETRY};
    if (mode == 1) ops->abi_version = 999;
    if (mode == 2) ops->size = 0;
    if (mode == 3) ops->prepare = NULL;
    if (mode == 6 || mode == 7) ops->capabilities = 0;
    *context = c;
    return 0;
}
#endif
