/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "fuzz.h"
#include "ipa_manager.h"

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    struct fuzz_input in = {data, size};
    uint8_t mode = fuzz_u8(&in);
    char environment[4097]; size_t length = in.size < sizeof(environment) - 1 ? in.size : sizeof(environment) - 1;
    if (length) memcpy(environment, in.data, length);
    environment[length] = 0;
    cc_ipa_module_directory inputs[4] = {{"/explicit-first", 0}, {"", 0}, {"/explicit-last", 2}, {environment, 0}};
    cc_ipa_manager_options options = {.directories = inputs, .num_directories = mode & 4U ? 4 : 3,
        .flags = mode & 7U, .max_modules = mode & 8U ? UINT32_MAX : 0};
    if (mode & 16U) inputs[0].max_depth = UINT32_MAX;
    if (mode & 32U) options.flags = UINT32_MAX;
    cc_ipa_module_directory *paths = (cc_ipa_module_directory *)(uintptr_t)1;
    size_t count = 999;
    bool privileged = (mode & 64U) != 0;
    int ret = cc_ipa_manager_search_paths(&options, environment, privileged, "/installed", "/build", &paths, &count);
    if (ret < 0) FUZZ_CHECK(paths == (cc_ipa_module_directory *)(uintptr_t)1 && count == 999);
    else {
        FUZZ_CHECK(count >= 2 && count <= CC_IPA_MANAGER_MAX_DIRECTORIES);
        FUZZ_CHECK(!strcmp(paths[0].path, "/explicit-first") && paths[0].max_depth == 0);
        FUZZ_CHECK(!strcmp(paths[1].path, "/explicit-last") && paths[1].max_depth == 2);
        size_t begin = 2;
        if ((mode & 4U) && environment[0]) { FUZZ_CHECK(!strcmp(paths[2].path, environment)); ++begin; }
        size_t end = count;
        if (options.flags & CC_IPA_MANAGER_INSTALLED) { FUZZ_CHECK(!strcmp(paths[end - 1].path, "/installed")); --end; }
        if (options.flags & CC_IPA_MANAGER_BUILD_TREE) {
            FUZZ_CHECK(!strcmp(paths[end - 1].path, "/build") && paths[end - 1].max_depth == 2); --end;
        }
        if (!(options.flags & CC_IPA_MANAGER_ENVIRONMENT) || privileged) FUZZ_CHECK(begin == end);
        for (size_t i = begin; i < end; ++i) {
            FUZZ_CHECK(paths[i].path[0] && !strchr(paths[i].path, ':') && paths[i].max_depth == 0);
        }
        cc_ipa_manager_free_paths(paths, count);
    }
    /* Ignoring an environment source must make its contents irrelevant. */
    cc_ipa_manager_options disabled = {.flags = CC_IPA_MANAGER_ENVIRONMENT};
    paths = NULL; count = 999;
    FUZZ_CHECK(cc_ipa_manager_search_paths(&disabled, environment, true, "", NULL, &paths, &count) == 0);
    FUZZ_CHECK(!paths && !count);
    return 0;
}
