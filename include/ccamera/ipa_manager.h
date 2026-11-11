/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_IPA_MANAGER_H
#define CCAMERA_IPA_MANAGER_H
#include <ccamera/ipa_module.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct cc_ipa_manager cc_ipa_manager;
#define CC_IPA_MANAGER_MAX_DIRECTORIES 64U
#define CC_IPA_MANAGER_MAX_MODULES 256U
#define CC_IPA_MANAGER_MAX_DEPTH 8U
enum cc_ipa_manager_flags {
    CC_IPA_MANAGER_ENVIRONMENT = 1U, /* Append CCAMERA_IPA_MODULE_PATH. */
    CC_IPA_MANAGER_INSTALLED = 2U,   /* Append the configured installation directory. */
    CC_IPA_MANAGER_BUILD_TREE = 4U   /* Search an identified uninstalled image's ipa/ at depth 2. */
};
typedef struct cc_ipa_module_directory {
    const char *path;
    uint32_t max_depth; /* 0: this directory only. Nested symlink directories are not traversed. */
} cc_ipa_module_directory;
typedef struct cc_ipa_manager_options {
    const cc_ipa_module_directory *directories; /* First, in caller priority order. */
    size_t num_directories;
    uint32_t flags;
    uint32_t max_modules; /* 0: MAX_MODULES; otherwise 1..MAX_MODULES. */
} cc_ipa_manager_options;

/* Discover a snapshot without loading code or running constructors. NULL options
 * enables environment, build-tree and installation directories; explicit options select the
 * sources exactly. Environment paths follow explicit directories, are colon
 * separated, skip empty components, and are ignored in a privileged execution.
 * A detected build tree follows environment paths and is searched at depth 2.
 * The installed directory is last. Environment/installed directories are nonrecursive.
 * Within each directory, .so candidates (including regular-file symlinks) are
 * sorted by complete path, including recursive results. Duplicate modules remain
 * in priority order. Missing/inaccessible directories and invalid modules are
 * skipped. Resource, traversal or limit failures abort the whole construction.
 * Limits: 64 directories, depth 8, 4096 candidates and 65536 directory entries
 * in total; environment path text is limited to 64 KiB. *manager is NULL on error.
 * Calls on a manager and its modules/IPAs must be serialized. */
CC_API int cc_ipa_manager_open(const cc_ipa_manager_options *options, cc_ipa_manager **manager);
/* Existing IPA instances retain their code after manager close. NULL is allowed. */
CC_API void cc_ipa_manager_close(cc_ipa_manager *manager);
CC_API size_t cc_ipa_manager_count(const cc_ipa_manager *manager);
/* Borrowed metadata/paths valid until close. NULL for invalid manager/index.
 * Paths are canonical; aliases can therefore appear more than once. */
CC_API const cc_ipa_module_info *cc_ipa_manager_get_info(const cc_ipa_manager *manager, size_t index);
CC_API const char *cc_ipa_manager_get_path(const cc_ipa_manager *manager, size_t index);
CC_API int cc_ipa_manager_get_signature(const cc_ipa_manager *manager, size_t index,
    const void **data, size_t *size);
CC_API int cc_ipa_manager_verify_signature(const cc_ipa_manager *manager, size_t index,
    cc_ipa_signature_status *status);
/* Select the FIRST matching module, not the highest version. name is optional
 * and matches the metadata name exactly. Versions are inclusive. Returns 0 or
 * a negative errno, -ENOENT if none matches; index stays unchanged on failure. */
CC_API int cc_ipa_manager_select(const cc_ipa_manager *manager, const char *pipeline_name,
    uint32_t minimum_version, uint32_t maximum_version, const char *name, size_t *index);
/* Select and create a trusted, in-process C IPA. A selected factory/link failure
 * is returned directly; there is no fallback to lower-priority modules. Signature
 * verification and process isolation are separate. *ipa is NULL on error. */
CC_API int cc_ipa_manager_create(cc_ipa_manager *manager, const char *pipeline_name,
    uint32_t minimum_version, uint32_t maximum_version, const char *name,
    const void *data, size_t size, cc_ipa **ipa);

#ifdef __cplusplus
}
#endif
#endif
