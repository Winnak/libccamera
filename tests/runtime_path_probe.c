/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <ccamera/ipa_manager.h>
#include <ccamera/ipa_proxy.h>
#include <stdbool.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
#define OK(x) CHECK((x) == 0)
int main(int argc, char **argv)
{
    CHECK(argc == 5);
    size_t expected = (size_t)strtoul(argv[1], NULL, 10);
    bool environment = !strcmp(argv[3], "environment");
    if (!strcmp(argv[4], "fd0")) (void)close(STDIN_FILENO);
    if (!strcmp(argv[4], "cwd")) OK(chdir("/"));
    cc_ipa_manager_options options = {.flags = CC_IPA_MANAGER_BUILD_TREE |
        (environment ? CC_IPA_MANAGER_ENVIRONMENT : 0U)};
    cc_ipa_manager *manager = NULL; OK(cc_ipa_manager_open(&options, &manager));
    if (cc_ipa_manager_count(manager) != expected)
        fprintf(stderr, "Runtime discovery count %zu, expected %zu\n", cc_ipa_manager_count(manager), expected);
    CHECK(cc_ipa_manager_count(manager) == expected);
    if (expected) {
        size_t index = 999; OK(cc_ipa_manager_select(manager, "TestPipeline", 7, 7, NULL, &index));
        const char *path = cc_ipa_manager_get_path(manager, index);
        if (strcmp(path, argv[2])) fprintf(stderr, "Runtime selected %s, expected %s\n", path, argv[2]);
        CHECK(!strcmp(path, argv[2]));
        uint32_t mode = 7; cc_ipa *ipa = NULL;
        if (getenv("CCAMERA_TEST_RUNTIME_PROXY")) {
            int ret = cc_ipa_manager_create_proxy(manager, "TestPipeline", 7, 7, NULL, &mode, sizeof(mode), NULL, &ipa);
            if (ret == -ENOSYS || ret == -EPERM || ret == -EACCES) { cc_ipa_manager_close(manager); return 77; }
            OK(ret);
        } else OK(cc_ipa_manager_create(manager, "TestPipeline", 7, 7, NULL, &mode, sizeof(mode), &ipa));
        cc_ipa_manager_close(manager); manager = NULL;
        cc_ipa_configuration configuration = {.pipeline = CC_IPA_VC4, .source = {.width = 97}};
        OK(cc_ipa_configure(ipa, &configuration)); OK(cc_ipa_start(ipa));
        cc_ipa_request request = {0}; cc_ipa_settings settings; OK(cc_ipa_prepare(ipa, &request, &settings));
        cc_ipa_frame frame = {.statistics = {.luminance = 0.25}}; cc_ipa_metadata metadata;
        OK(cc_ipa_process(ipa, &frame, &metadata)); CHECK(metadata.exposure_ns == 97);
        cc_ipa_destroy(ipa);
    }
    cc_ipa_manager_close(manager);
    return 0;
}
