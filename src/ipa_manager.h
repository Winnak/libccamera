/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_IPA_MANAGER_INTERNAL_H
#define CCAMERA_IPA_MANAGER_INTERNAL_H
#include <ccamera/ipa_manager.h>
#include <stdbool.h>
/* Test the environment policy without changing process credentials. */
int cc_ipa_manager_open_environment(const cc_ipa_manager_options *options, const char *environment,
    bool privileged, const char *installed_directory, const char *build_directory, cc_ipa_manager **manager);
/* Pure path-policy seam for fuzzing; no filesystem access. Owned paths on
 * success, unchanged outputs on error. */
int cc_ipa_manager_search_paths(const cc_ipa_manager_options *options, const char *environment,
    bool privileged, const char *installed_directory, const char *build_directory,
    cc_ipa_module_directory **directories, size_t *count);
void cc_ipa_manager_free_paths(cc_ipa_module_directory *directories, size_t count);
#endif
