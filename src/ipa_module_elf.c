/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Metadata inspection follows libcamera's ipa_module.cpp ELF symbol lookup.
 * Reference: Copyright (C) 2019 Google Inc. */
#include "ipa_module.h"
#include <elf.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>

#if UINTPTR_MAX == UINT64_MAX
typedef Elf64_Ehdr elf_header;
typedef Elf64_Shdr elf_section;
typedef Elf64_Sym elf_symbol;
#define NATIVE_CLASS ELFCLASS64
#else
typedef Elf32_Ehdr elf_header;
typedef Elf32_Shdr elf_section;
typedef Elf32_Sym elf_symbol;
#define NATIVE_CLASS ELFCLASS32
#endif

static bool span(size_t size, uint64_t offset, uint64_t length)
{
    return offset <= size && length <= size - offset;
}
static bool section_at(const unsigned char *data, size_t size, const elf_header *h,
                       uint32_t index, elf_section *out)
{
    if (index >= h->e_shnum) return false;
    uint64_t offset = h->e_shoff + (uint64_t)index * h->e_shentsize;
    if (!span(size, offset, sizeof(*out))) return false;
    memcpy(out, data + (size_t)offset, sizeof(*out));
    return true;
}
static bool native_machine(uint16_t machine)
{
#if defined(__x86_64__)
    return machine == EM_X86_64;
#elif defined(__aarch64__)
    return machine == EM_AARCH64;
#elif defined(__i386__)
    return machine == EM_386;
#elif defined(__arm__)
    return machine == EM_ARM;
#elif defined(__riscv)
    return machine == EM_RISCV;
#elif defined(__powerpc64__)
    return machine == EM_PPC64;
#elif defined(__powerpc__)
    return machine == EM_PPC;
#elif defined(__s390x__)
    return machine == EM_S390;
#else
    (void)machine; /* The system dynamic linker validates other architectures. */
    return true;
#endif
}
static bool name_valid(const char *name, size_t capacity)
{
    if (!name[0]) return false;
    for (size_t i = 0; i < capacity; ++i) {
        unsigned char c = (unsigned char)name[i];
        if (!c) return true;
        if (c < 32 || c > 126 || c == '*' || c == '?' || c == '\\') return false;
    }
    return false;
}
int cc_ipa_module_elf(const void *bytes, size_t size, cc_ipa_module_info *info, bool *factory)
{
    if (!bytes || !info || !factory) return -EINVAL;
    const unsigned char *data = bytes;
    elf_header h;
    if (size < sizeof(h)) return -ENOEXEC;
    memcpy(&h, data, sizeof(h));
    const uint16_t endian = 1;
    unsigned char native_data = *(const unsigned char *)&endian ? ELFDATA2LSB : ELFDATA2MSB;
    if (memcmp(h.e_ident, ELFMAG, SELFMAG) || h.e_ident[EI_CLASS] != NATIVE_CLASS ||
        h.e_ident[EI_DATA] != native_data || h.e_ident[EI_VERSION] != EV_CURRENT ||
        h.e_version != EV_CURRENT || h.e_type != ET_DYN || !native_machine(h.e_machine) ||
        h.e_ehsize != sizeof(h) || h.e_shentsize != sizeof(elf_section) || !h.e_shnum ||
        !span(size, h.e_shoff, (uint64_t)h.e_shnum * h.e_shentsize)) return -ENOEXEC;
    bool found = false, have_factory = false, have_symbols = false;
    cc_ipa_module_info result;
    for (uint32_t i = 0; i < h.e_shnum; ++i) {
        elf_section table, strings;
        if (!section_at(data, size, &h, i, &table)) return -ENOEXEC;
        if (table.sh_type != SHT_DYNSYM) continue;
        if (have_symbols) return -ENOEXEC;
        have_symbols = true;
        if (table.sh_entsize != sizeof(elf_symbol) || table.sh_size % sizeof(elf_symbol) ||
            !span(size, table.sh_offset, table.sh_size) ||
            !section_at(data, size, &h, table.sh_link, &strings) || strings.sh_type != SHT_STRTAB ||
            !span(size, strings.sh_offset, strings.sh_size)) return -ENOEXEC;
        for (uint64_t offset = 0; offset < table.sh_size; offset += sizeof(elf_symbol)) {
            elf_symbol symbol;
            memcpy(&symbol, data + (size_t)(table.sh_offset + offset), sizeof(symbol));
            if (symbol.st_name >= strings.sh_size) return -ENOEXEC;
            const char *name = (const char *)data + (size_t)(strings.sh_offset + symbol.st_name);
            uint64_t remaining = strings.sh_size - symbol.st_name;
            /* Compare only bounded target names. Scanning an entire string for
             * every unrelated symbol can turn a corrupt table into O(n^2). */
            bool metadata = remaining >= sizeof("cc_ipa_module_descriptor") &&
                !memcmp(name, "cc_ipa_module_descriptor", sizeof("cc_ipa_module_descriptor"));
            bool creator = remaining >= sizeof("cc_ipa_module_create_v1") &&
                !memcmp(name, "cc_ipa_module_create_v1", sizeof("cc_ipa_module_create_v1"));
            if (!metadata && !creator) continue;
            elf_section section;
            if (ELF64_ST_BIND(symbol.st_info) != STB_GLOBAL ||
                ELF64_ST_TYPE(symbol.st_info) != (metadata ? STT_OBJECT : STT_FUNC) ||
                ELF64_ST_VISIBILITY(symbol.st_other) != STV_DEFAULT ||
                symbol.st_shndx == SHN_UNDEF || symbol.st_shndx >= SHN_LORESERVE ||
                !section_at(data, size, &h, symbol.st_shndx, &section) ||
                section.sh_type == SHT_NOBITS || !(section.sh_flags & SHF_ALLOC) ||
                (section.sh_flags & SHF_COMPRESSED) ||
                !span(size, section.sh_offset, section.sh_size) || symbol.st_value < section.sh_addr)
                return -ENOEXEC;
            uint64_t relative = symbol.st_value - section.sh_addr;
            if (!span((size_t)section.sh_size, relative, symbol.st_size)) return -ENOEXEC;
            if (creator) {
                if (have_factory || !(section.sh_flags & SHF_EXECINSTR) || !symbol.st_size) return -ENOEXEC;
                have_factory = true;
            } else {
                /* Sanitizers may include a trailing redzone in the ELF symbol. */
                if (found || symbol.st_size < sizeof(result)) return -ENOEXEC;
                memcpy(&result, data + (size_t)(section.sh_offset + relative), sizeof(result));
                found = true;
            }
        }
    }
    if (!found) return -ENOENT;
    if (result.module_api_version != CC_IPA_MODULE_API_VERSION || result.ipa_abi_version != CC_IPA_ABI_VERSION)
        return -EPROTONOSUPPORT;
    if (!name_valid(result.name, sizeof(result.name)) ||
        !name_valid(result.pipeline_name, sizeof(result.pipeline_name))) return -EINVAL;
    *info = result;
    *factory = have_factory;
    return 0;
}
