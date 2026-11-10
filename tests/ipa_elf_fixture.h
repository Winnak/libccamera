/* SPDX-License-Identifier: CC0-1.0 */
#ifndef CCAMERA_TEST_IPA_ELF_FIXTURE_H
#define CCAMERA_TEST_IPA_ELF_FIXTURE_H
#include <ccamera/ipa_module.h>
#include <elf.h>
#include <stdint.h>
#include <string.h>
#if UINTPTR_MAX == UINT64_MAX
typedef Elf64_Ehdr test_elf_header;
typedef Elf64_Shdr test_elf_section;
typedef Elf64_Sym test_elf_symbol;
#define TEST_ELF_CLASS ELFCLASS64
#else
typedef Elf32_Ehdr test_elf_header;
typedef Elf32_Shdr test_elf_section;
typedef Elf32_Sym test_elf_symbol;
#define TEST_ELF_CLASS ELFCLASS32
#endif
#define TEST_ELF_SIZE 2048U
#define TEST_ELF_SECTIONS 128U
#define TEST_ELF_SYMBOLS 512U
#define TEST_ELF_STRINGS 640U
#define TEST_ELF_INFO 768U
#define TEST_ELF_CODE 1536U
static void test_ipa_elf(unsigned char data[TEST_ELF_SIZE])
{
    memset(data, 0, TEST_ELF_SIZE);
    const uint16_t endian = 1;
    test_elf_header h = {.e_type = ET_DYN, .e_version = EV_CURRENT,
        .e_ehsize = sizeof(h), .e_shoff = TEST_ELF_SECTIONS,
        .e_shentsize = sizeof(test_elf_section), .e_shnum = 5};
    memcpy(h.e_ident, ELFMAG, SELFMAG);
    h.e_ident[EI_CLASS] = TEST_ELF_CLASS;
    h.e_ident[EI_DATA] = *(const unsigned char *)&endian ? ELFDATA2LSB : ELFDATA2MSB;
    h.e_ident[EI_VERSION] = EV_CURRENT;
#if defined(__x86_64__)
    h.e_machine = EM_X86_64;
#elif defined(__aarch64__)
    h.e_machine = EM_AARCH64;
#elif defined(__i386__)
    h.e_machine = EM_386;
#elif defined(__arm__)
    h.e_machine = EM_ARM;
#elif defined(__riscv)
    h.e_machine = EM_RISCV;
#elif defined(__powerpc64__)
    h.e_machine = EM_PPC64;
#elif defined(__powerpc__)
    h.e_machine = EM_PPC;
#elif defined(__s390x__)
    h.e_machine = EM_S390;
#endif
    memcpy(data, &h, sizeof(h));
    static const char strings[] = "\0cc_ipa_module_descriptor\0cc_ipa_module_create_v1\0";
    memcpy(data + TEST_ELF_STRINGS, strings, sizeof(strings));
    test_elf_section sections[5] = {{0},
        {.sh_type = SHT_DYNSYM, .sh_offset = TEST_ELF_SYMBOLS,
         .sh_size = 3 * sizeof(test_elf_symbol), .sh_link = 2, .sh_entsize = sizeof(test_elf_symbol)},
        {.sh_type = SHT_STRTAB, .sh_offset = TEST_ELF_STRINGS, .sh_size = sizeof(strings)},
        {.sh_type = SHT_PROGBITS, .sh_flags = SHF_ALLOC, .sh_addr = 4096,
         .sh_offset = TEST_ELF_INFO, .sh_size = sizeof(cc_ipa_module_info)},
        {.sh_type = SHT_PROGBITS, .sh_flags = SHF_ALLOC | SHF_EXECINSTR, .sh_addr = 8192,
         .sh_offset = TEST_ELF_CODE, .sh_size = 1}};
    memcpy(data + TEST_ELF_SECTIONS, sections, sizeof(sections));
    test_elf_symbol symbols[3] = {{0},
        {.st_name = 1, .st_info = ELF64_ST_INFO(STB_GLOBAL, STT_OBJECT), .st_shndx = 3,
         .st_value = 4096, .st_size = sizeof(cc_ipa_module_info)},
        {.st_name = 1 + sizeof("cc_ipa_module_descriptor"), .st_info = ELF64_ST_INFO(STB_GLOBAL, STT_FUNC),
         .st_shndx = 4, .st_value = 8192, .st_size = 1}};
    memcpy(data + TEST_ELF_SYMBOLS, symbols, sizeof(symbols));
    cc_ipa_module_info info = {.module_api_version = CC_IPA_MODULE_API_VERSION,
        .ipa_abi_version = CC_IPA_ABI_VERSION, .pipeline_version = 7,
        .pipeline_name = "TestPipeline", .name = "test-module"};
    memcpy(data + TEST_ELF_INFO, &info, sizeof(info));
}
#endif
