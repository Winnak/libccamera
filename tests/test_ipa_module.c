/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "ipa_module.h"
#include "ipa_elf_fixture.h"
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
#define OK(x) CHECK((x) == 0)
#define ERROR(x,e) CHECK((x) == -(e))
static char directory[] = "/tmp/ccamera-ipa-module-XXXXXX";
static char trace_path[256];
static void write_file(const char *path, const void *data, size_t size)
{
    FILE *file = fopen(path, "wb"); CHECK(file);
    CHECK(fwrite(data, 1, size, file) == size); OK(fclose(file));
}
static void copy_file(const char *source, const char *destination)
{
    FILE *input = fopen(source, "rb"), *output = fopen(destination, "wb"); CHECK(input && output);
    unsigned char bytes[4096]; size_t size;
    while ((size = fread(bytes, 1, sizeof(bytes), input))) CHECK(fwrite(bytes, 1, size, output) == size);
    CHECK(!ferror(input)); OK(fclose(input)); OK(fclose(output));
}
static void reset_trace(void) { write_file(trace_path, "", 0); }
static void expect_trace(const char *expected)
{
    char text[1024];
    FILE *file = fopen(trace_path, "rb"); CHECK(file);
    size_t n = fread(text, 1, sizeof(text) - 1, file);
    CHECK(!ferror(file)); OK(fclose(file)); text[n] = 0;
    if (strcmp(text, expected)) fprintf(stderr, "trace: '%s', expected '%s'\n", text, expected);
    CHECK(!strcmp(text, expected));
}
static void test_lifetime(const char *path)
{
    reset_trace();
    cc_ipa_module *m = NULL; OK(cc_ipa_module_open(path, &m)); expect_trace("");
    const cc_ipa_module_info *info = cc_ipa_module_get_info(m);
    CHECK(info && !strcmp(info->name, "test-module") && info->pipeline_version == 7);
    CHECK(cc_ipa_module_get_path(m)[0] == '/');
    CHECK(cc_ipa_module_match(m, "TestPipeline", 7, 7) == 1);
    CHECK(cc_ipa_module_match(m, "TestPipeline", 6, 8) == 1);
    CHECK(cc_ipa_module_match(m, "testpipeline", 0, UINT32_MAX) == 0);
    CHECK(cc_ipa_module_match(m, "TestPipeline", 8, 9) == 0);
    CHECK(cc_ipa_module_match(m, "TestPipeline", 0, 6) == 0);
    ERROR(cc_ipa_module_match(m, "TestPipeline", 8, 7), EINVAL);
    const void *signature = (void *)(uintptr_t)1; size_t size = 100;
    OK(cc_ipa_module_get_signature(m, &signature, &size)); CHECK(!signature && !size);
    cc_ipa *a = NULL, *b = NULL;
    OK(cc_ipa_module_create(m, NULL, 0, &a)); OK(cc_ipa_module_create(m, NULL, 0, &b)); expect_trace("LCC");
    CHECK(cc_ipa_get_capabilities(a) == CC_IPA_CAP_SENSOR_GEOMETRY);
    ERROR(cc_ipa_start(a), EPIPE);
    cc_ipa_configuration c = {.source = {.width = 640}};
    OK(cc_ipa_configure(a, &c)); c.source.width = 320; OK(cc_ipa_configure(b, &c));
    OK(cc_ipa_start(a)); OK(cc_ipa_start(b));
    cc_ipa_request request = {0}; cc_ipa_settings settings;
    cc_ipa_frame frame = {.statistics = {.luminance = 0.25}}; cc_ipa_metadata metadata;
    OK(cc_ipa_prepare(a, &request, &settings)); OK(cc_ipa_process(a, &frame, &metadata));
    CHECK(metadata.exposure_ns == 640 && metadata.luminance == 0.25);
    cc_ipa_module_close(m); expect_trace("LCCccsspf");
    cc_ipa_destroy(a); expect_trace("LCCccsspftD");
    OK(cc_ipa_prepare(b, &request, &settings)); OK(cc_ipa_process(b, &frame, &metadata));
    CHECK(metadata.exposure_ns == 320);
    cc_ipa_stop(b); cc_ipa_stop(b); cc_ipa_destroy(b); expect_trace("LCCccsspftDpftDU");
}
static void test_factories(const char *path)
{
    reset_trace(); cc_ipa_module *m = NULL; OK(cc_ipa_module_open(path, &m));
    cc_ipa *ipa = (cc_ipa *)(uintptr_t)1;
    ERROR(cc_ipa_module_create(m, NULL, 1, &ipa), EINVAL); CHECK(!ipa); expect_trace("");
    const int errors[] = {0, EPROTONOSUPPORT, EPROTONOSUPPORT, EINVAL, EPROTO, ERANGE};
    for (uint32_t mode = 1; mode <= 5; ++mode) {
        ERROR(cc_ipa_module_create(m, &mode, sizeof(mode), &ipa), errors[mode]); CHECK(!ipa);
    }
    uint32_t mode = 6;
    OK(cc_ipa_module_create(m, &mode, sizeof(mode), &ipa));
    cc_ipa_configuration c = {0}; ERROR(cc_ipa_configure(ipa, &c), EPROTO);
    ERROR(cc_ipa_start(ipa), EPIPE); cc_ipa_destroy(ipa);
    OK(cc_ipa_module_create(m, NULL, 0, &ipa)); cc_ipa_destroy(ipa);
    cc_ipa_module_close(m); expect_trace("LCDCDCDCCCcDCDU");
}
static void test_overlapping_handles(const char *path)
{
    reset_trace(); cc_ipa_module *a = NULL, *b = NULL; cc_ipa *first = NULL, *second = NULL;
    OK(cc_ipa_module_open(path, &a)); OK(cc_ipa_module_open(path, &b)); expect_trace("");
    OK(cc_ipa_module_create(a, NULL, 0, &first)); OK(cc_ipa_module_create(b, NULL, 0, &second));
    CHECK(!dlsym(RTLD_DEFAULT, "cc_ipa_module_create_v1"));
    cc_ipa_module_close(a); cc_ipa_module_close(b); expect_trace("LCC");
    cc_ipa_destroy(first); expect_trace("LCCD"); cc_ipa_destroy(second); expect_trace("LCCDDU");
}
static void test_invalid_modules(char **paths)
{
    reset_trace();
    const int errors[] = {ENOENT, EPROTONOSUPPORT, EPROTONOSUPPORT, EINVAL, EINVAL};
    for (size_t i = 0; i < 5; ++i) {
        cc_ipa_module *m = (cc_ipa_module *)(uintptr_t)1;
        ERROR(cc_ipa_module_open(paths[i], &m), errors[i]); CHECK(!m);
    }
    expect_trace("");
    for (size_t i = 5; i < 7; ++i) {
        cc_ipa_module *m = NULL; cc_ipa *ipa = NULL;
        OK(cc_ipa_module_open(paths[i], &m));
        ERROR(cc_ipa_module_create(m, NULL, 0, &ipa), i == 5 ? ENOENT : ENOEXEC);
        CHECK(!ipa); cc_ipa_module_close(m);
    }
    expect_trace("");
}
static void test_files(const char *source)
{
    char path[256], replacement[256], sign[256];
    CHECK(snprintf(path, sizeof(path), "%s/module.so", directory) > 0);
    CHECK(snprintf(replacement, sizeof(replacement), "%s/replacement.so", directory) > 0);
    CHECK(snprintf(sign, sizeof(sign), "%s/module.so.sign", directory) > 0);
    copy_file(source, path); const unsigned char signature[] = {1, 0, 255, 4};
    write_file(sign, signature, sizeof(signature));
    reset_trace(); cc_ipa_module *m = NULL; cc_ipa *ipa = NULL;
    OK(cc_ipa_module_open(path, &m));
    const void *data; size_t size;
    OK(cc_ipa_module_get_signature(m, &data, &size));
    CHECK(size == sizeof(signature) && !memcmp(data, signature, size));
    write_file(sign, "changed", 7);
    CHECK(!memcmp(data, signature, sizeof(signature)));
    ERROR(cc_ipa_module_get_signature(NULL, &data, &size), EINVAL);
    CHECK(size == sizeof(signature) && !memcmp(data, signature, size));
    copy_file(source, replacement); OK(rename(replacement, path));
    ERROR(cc_ipa_module_create(m, NULL, 0, &ipa), ESTALE); CHECK(!ipa);
    cc_ipa_module_close(m); expect_trace("");
    OK(cc_ipa_module_open(path, &m)); OK(truncate(path, 20));
    ERROR(cc_ipa_module_create(m, NULL, 0, &ipa), ESTALE); cc_ipa_module_close(m); expect_trace("");
    copy_file(source, path); OK(cc_ipa_module_open(path, &m)); OK(unlink(path));
    ERROR(cc_ipa_module_create(m, NULL, 0, &ipa), ESTALE); cc_ipa_module_close(m); expect_trace("");
    copy_file(source, path); OK(truncate(sign, 1024 * 1024 + 1));
    ERROR(cc_ipa_module_open(path, &m), EFBIG); CHECK(!m); OK(unlink(sign));
    OK(truncate(path, 64 * 1024 * 1024 + 1)); ERROR(cc_ipa_module_open(path, &m), EFBIG); CHECK(!m);
    write_file(path, "not an ELF file", 15); ERROR(cc_ipa_module_open(path, &m), ENOEXEC);
    OK(unlink(path)); ERROR(cc_ipa_module_open(path, &m), ENOENT);
    OK(mkfifo(path, 0600)); ERROR(cc_ipa_module_open(path, &m), EINVAL); OK(unlink(path));
    ERROR(cc_ipa_module_open(directory, &m), EINVAL);
    /* A still-live old image with identical metadata must not satisfy a new
     * handle opened after replacement under the same dlopen path. */
    copy_file(source, path); OK(cc_ipa_module_open(path, &m));
    OK(cc_ipa_module_create(m, NULL, 0, &ipa)); cc_ipa_module_close(m);
    copy_file(source, replacement); OK(rename(replacement, path));
    cc_ipa_module *new_module = NULL; cc_ipa *new_ipa = NULL;
    OK(cc_ipa_module_open(path, &new_module));
    ERROR(cc_ipa_module_create(new_module, NULL, 0, &new_ipa), ESTALE); CHECK(!new_ipa);
    cc_ipa_destroy(ipa); /* Unload old code; creating from the new image can retry. */
    OK(cc_ipa_module_create(new_module, NULL, 0, &new_ipa));
    cc_ipa_module_close(new_module); cc_ipa_destroy(new_ipa); expect_trace("LCDULCDU");
    reset_trace();
    copy_file(source, path);
    int saved = dup(STDIN_FILENO); (void)close(STDIN_FILENO);
    OK(cc_ipa_module_open(path, &m));
    int flags = fcntl(STDIN_FILENO, F_GETFD); CHECK(flags >= 0 && (flags & FD_CLOEXEC));
    cc_ipa_module_close(m); CHECK(fcntl(STDIN_FILENO, F_GETFD) == -1 && errno == EBADF);
    if (saved >= 0) { CHECK(dup2(saved, STDIN_FILENO) == STDIN_FILENO); OK(close(saved)); }
    OK(unlink(path)); expect_trace("");
}
static void rejected(const unsigned char *data, size_t size)
{
    cc_ipa_module_info info, original; memset(&info, 0xa5, sizeof(info)); original = info;
    bool factory = true;
    CHECK(cc_ipa_module_elf(data, size, &info, &factory) < 0);
    CHECK(factory && !memcmp(&info, &original, sizeof(info)));
}
static void test_elf(void)
{
    unsigned char unaligned[TEST_ELF_SIZE + 1], *data = unaligned + 1;
    test_ipa_elf(data); cc_ipa_module_info info; bool factory = false;
    OK(cc_ipa_module_elf(data, TEST_ELF_SIZE, &info, &factory));
    CHECK(factory && !strcmp(info.name, "test-module"));
    for (size_t i = 0; i < TEST_ELF_CODE + 1; ++i) rejected(data, i);
    for (unsigned field = 0; field < 12; ++field) {
        test_ipa_elf(data); test_elf_header h; memcpy(&h, data, sizeof(h));
        switch (field) {
        case 0: h.e_ident[EI_MAG0] = 0; break;
        case 1: h.e_ident[EI_CLASS] ^= 3; break;
        case 2: h.e_ident[EI_DATA] ^= 3; break;
        case 3: h.e_ident[EI_VERSION] = 0; break;
        case 4: h.e_version = 0; break;
        case 5: h.e_type = ET_EXEC; break;
        case 6: if (h.e_machine == EM_NONE) continue; h.e_machine = EM_NONE; break;
        case 7: h.e_ehsize = 0; break;
        case 8: h.e_shentsize = 0; break;
        case 9: h.e_shoff = UINT32_MAX; break;
        case 10: h.e_shnum = UINT16_MAX; break;
        case 11: h.e_shnum = 0; break;
        }
        memcpy(data, &h, sizeof(h)); rejected(data, TEST_ELF_SIZE);
    }
    for (unsigned field = 0; field < 11; ++field) {
        test_ipa_elf(data); test_elf_section s; memcpy(&s, data + TEST_ELF_SECTIONS + sizeof(s), sizeof(s));
        switch (field) {
        case 0: s.sh_entsize = 0; break;
        case 1: --s.sh_size; break;
        case 2: s.sh_offset = UINT32_MAX; break;
        case 3: s.sh_size = UINT32_MAX; break;
        case 4: s.sh_link = UINT32_MAX; break;
        case 5: s.sh_link = 0; break;
        default: break;
        }
        if (field < 6) memcpy(data + TEST_ELF_SECTIONS + sizeof(s), &s, sizeof(s));
        else {
            memcpy(&s, data + TEST_ELF_SECTIONS + 3 * sizeof(s), sizeof(s));
            if (field == 6) s.sh_type = SHT_NOBITS;
            if (field == 7) s.sh_flags = 0;
            if (field == 8) s.sh_offset = UINT32_MAX;
            if (field == 9) s.sh_flags |= SHF_COMPRESSED;
            if (field == 10) s.sh_addr = 8192;
            memcpy(data + TEST_ELF_SECTIONS + 3 * sizeof(s), &s, sizeof(s));
        }
        rejected(data, TEST_ELF_SIZE);
    }
    for (unsigned field = 0; field < 9; ++field) {
        test_ipa_elf(data); test_elf_symbol s; memcpy(&s, data + TEST_ELF_SYMBOLS + sizeof(s), sizeof(s));
        switch (field) {
        case 0: s.st_name = UINT32_MAX; break;
        case 1: s.st_info = ELF64_ST_INFO(STB_LOCAL, STT_OBJECT); break;
        case 2: s.st_info = ELF64_ST_INFO(STB_GLOBAL, STT_FUNC); break;
        case 3: s.st_other = STV_HIDDEN; break;
        case 4: s.st_shndx = SHN_UNDEF; break;
        case 5: s.st_shndx = SHN_XINDEX; break;
        case 6: s.st_value = 4095; break;
        case 7: s.st_size = UINT32_MAX; break;
        case 8: --s.st_size; break;
        }
        memcpy(data + TEST_ELF_SYMBOLS + sizeof(s), &s, sizeof(s)); rejected(data, TEST_ELF_SIZE);
    }
    test_ipa_elf(data); memset(data + TEST_ELF_STRINGS, 'a', 256); rejected(data, TEST_ELF_SIZE);
    test_ipa_elf(data); memset(data + TEST_ELF_INFO + offsetof(cc_ipa_module_info, name), 'a', CC_IPA_MODULE_NAME_SIZE);
    rejected(data, TEST_ELF_SIZE);
    test_ipa_elf(data); data[TEST_ELF_INFO + offsetof(cc_ipa_module_info, name)] = '\\'; rejected(data, TEST_ELF_SIZE);
    test_ipa_elf(data); data[TEST_ELF_INFO + offsetof(cc_ipa_module_info, pipeline_name)] = 0; rejected(data, TEST_ELF_SIZE);
    /* Linkers/sanitizers may add trailing storage to an exported object. */
    test_ipa_elf(data); test_elf_symbol symbol; test_elf_section section;
    memcpy(&symbol, data + TEST_ELF_SYMBOLS + sizeof(symbol), sizeof(symbol)); symbol.st_size += 32;
    memcpy(data + TEST_ELF_SYMBOLS + sizeof(symbol), &symbol, sizeof(symbol));
    memcpy(&section, data + TEST_ELF_SECTIONS + 3 * sizeof(section), sizeof(section)); section.sh_size += 32;
    memcpy(data + TEST_ELF_SECTIONS + 3 * sizeof(section), &section, sizeof(section));
    OK(cc_ipa_module_elf(data, TEST_ELF_SIZE, &info, &factory));
    /* Ambiguous/overlapping dynamic symbol tables must not cause repeated scans. */
    memcpy(&section, data + TEST_ELF_SECTIONS + sizeof(section), sizeof(section));
    memcpy(data + TEST_ELF_SECTIONS, &section, sizeof(section)); rejected(data, TEST_ELF_SIZE);
}
int main(int argc, char **argv)
{
    CHECK(argc == 10); CHECK(mkdtemp(directory));
    CHECK(snprintf(trace_path, sizeof(trace_path), "%s/trace", directory) > 0);
    OK(setenv("CCAMERA_TEST_MODULE_TRACE", trace_path, 1));
    cc_ipa_module *m = (cc_ipa_module *)(uintptr_t)1; cc_ipa *ipa = (cc_ipa *)(uintptr_t)1;
    ERROR(cc_ipa_module_open(NULL, &m), EINVAL); CHECK(!m);
    ERROR(cc_ipa_module_open("", &m), EINVAL); ERROR(cc_ipa_module_open(argv[1], NULL), EINVAL);
    ERROR(cc_ipa_module_create(NULL, NULL, 0, &ipa), EINVAL); CHECK(!ipa);
    ERROR(cc_ipa_module_create(NULL, NULL, 0, NULL), EINVAL);
    CHECK(!cc_ipa_module_get_info(NULL) && !cc_ipa_module_get_path(NULL));
    ERROR(cc_ipa_module_match(NULL, "TestPipeline", 0, 9), EINVAL); cc_ipa_module_close(NULL);
    test_elf(); test_invalid_modules(argv + 2); test_lifetime(argv[1]); test_lifetime(argv[9]);
    test_factories(argv[1]); test_overlapping_handles(argv[1]); test_files(argv[1]);
    OK(unlink(trace_path)); OK(rmdir(directory));
    puts("IPA shared modules: ELF inspection, factory validation and retained lifetime passed");
    return 0;
}
