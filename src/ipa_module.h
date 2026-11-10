/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_IPA_MODULE_INTERNAL_H
#define CCAMERA_IPA_MODULE_INTERNAL_H
#include <ccamera/ipa_module.h>
#include <stdbool.h>

/* Pure, bounded ELF inspection seam; never resolves or executes symbols. */
int cc_ipa_module_elf(const void *data, size_t size, cc_ipa_module_info *info, bool *factory);
/* Native, pointer-free inspection stamp for an exec worker. The parent never
 * loads code to export it; import retains the passed file and verifies both
 * stamp and metadata before allowing ordinary lazy loading. */
struct cc_ipa_module_transfer {
    cc_ipa_module_info info;
    uint64_t device, inode, size;
    int64_t mtime_seconds, ctime_seconds;
    int32_t mtime_nanoseconds, ctime_nanoseconds;
};
int cc_ipa_module_export(cc_ipa_module *module, int *fd, struct cc_ipa_module_transfer *transfer);
int cc_ipa_module_import(const char *path, int fd, const struct cc_ipa_module_transfer *transfer, cc_ipa_module **module);
void cc_ipa_set_owner(cc_ipa *ipa, void *owner, void (*release)(void *));
void *cc_ipa_context(cc_ipa *ipa, int (*configure)(void *, const cc_ipa_configuration *));
#endif
