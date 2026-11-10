/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "fuzz.h"
#include "ipa_module.h"
#include "../ipa_elf_fixture.h"

static void inspect(const void *data, size_t size)
{
    cc_ipa_module_info info, original; memset(&info, 0xa5, sizeof(info)); original = info;
    bool factory = true;
    int ret = cc_ipa_module_elf(data, size, &info, &factory);
    if (ret < 0) FUZZ_CHECK(factory && !memcmp(&info, &original, sizeof(info)));
    else {
        FUZZ_CHECK(info.module_api_version == CC_IPA_MODULE_API_VERSION && info.ipa_abi_version == CC_IPA_ABI_VERSION);
        FUZZ_CHECK(info.name[0] && memchr(info.name, 0, sizeof(info.name)));
        FUZZ_CHECK(info.pipeline_name[0] && memchr(info.pipeline_name, 0, sizeof(info.pipeline_name)));
    }
}
int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    inspect(data, size);
    struct fuzz_input in = {data, size};
    uint8_t mode = fuzz_u8(&in);
    unsigned char unaligned[TEST_ELF_SIZE + 1], *elf = unaligned + 1;
    test_ipa_elf(elf);
    unsigned patches = mode & 7U;
    for (unsigned i = 0; i < patches; ++i) fuzz_patch(&in, elf, TEST_ELF_SIZE);
    size_t length = mode & 8U ? fuzz_u16(&in) % (TEST_ELF_SIZE + 1U) : TEST_ELF_SIZE;
    inspect(elf, length);
    return 0;
}
