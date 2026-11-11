/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_IPA_MODULE_H
#define CCAMERA_IPA_MODULE_H
#include <ccamera/ipa.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CC_IPA_MODULE_API_VERSION 1U
#define CC_IPA_MODULE_NAME_SIZE 256U
typedef struct cc_ipa_module cc_ipa_module;
/* Pointer-free, native-endian ELF metadata. A module exports a const object
 * named cc_ipa_module_descriptor with this layout and default visibility.
 * Names must be nonempty, NUL-terminated printable ASCII without '*', '?' or
 * backslash. Pipeline names and version intervals match exactly. */
typedef struct cc_ipa_module_info {
    uint32_t module_api_version, ipa_abi_version, pipeline_version;
    char pipeline_name[CC_IPA_MODULE_NAME_SIZE];
    char name[CC_IPA_MODULE_NAME_SIZE];
} cc_ipa_module_info;

/* A module also exports cc_ipa_module_create_v1 with this signature. Verify
 * host_abi_version and ops_size before writing the supplied zeroed table.
 * data is module-defined and borrowed for this call. On success, ownership of
 * context passes to ops.destroy (if supplied). On every nonzero return the
 * factory must clean up its own allocations. A zero return with invalid ops
 * is rejected and ops.destroy is called if supplied, even on an ABI mismatch;
 * the destroy member must always follow this factory API's host table layout.
 * The factory and all callbacks return zero or a negative errno. */
typedef int (*cc_ipa_module_factory)(uint32_t host_abi_version, size_t ops_size,
    const void *data, size_t data_size, cc_ipa_ops *ops, void **context);
/* Plugin entry points, defined by the module rather than libccamera. These
 * declarations also give C++ module definitions external C linkage. */
extern CC_API const cc_ipa_module_info cc_ipa_module_descriptor;
CC_API int cc_ipa_module_create_v1(uint32_t host_abi_version, size_t ops_size,
    const void *data, size_t data_size, cc_ipa_ops *ops, void **context);

/* Inspect a regular ELF shared object without loading it or executing code.
 * path is explicit; there is no directory/environment search. Native ELF
 * class/endianness, metadata names and both API versions are checked. Objects
 * larger than 64 MiB or signatures larger than 1 MiB return -EFBIG. The optional
 * resolved-path.sign blob is copied without verification; absent/unreadable means empty.
 * *module is NULL on error. Calls on a module and its IPAs must be serialized. */
CC_API int cc_ipa_module_open(const char *path, cc_ipa_module **module);
/* Drop the caller's reference. Existing IPAs keep code and metadata alive until
 * after their stop/destroy callbacks. NULL is allowed. */
CC_API void cc_ipa_module_close(cc_ipa_module *module);
/* Borrowed data valid until close; NULL for a NULL module. The path is canonical. */
CC_API const cc_ipa_module_info *cc_ipa_module_get_info(const cc_ipa_module *module);
CC_API const char *cc_ipa_module_get_path(const cc_ipa_module *module);
/* Returns 1 for a match, 0 otherwise, or -EINVAL for invalid arguments. */
CC_API int cc_ipa_module_match(const cc_ipa_module *module, const char *pipeline_name,
    uint32_t minimum_version, uint32_t maximum_version);
/* Borrowed optional signature bytes. Outputs stay unchanged on error. */
CC_API int cc_ipa_module_get_signature(const cc_ipa_module *module, const void **data, size_t *size);
typedef enum cc_ipa_signature_status {
    CC_IPA_SIGNATURE_VALID,
    CC_IPA_SIGNATURE_MISSING,
    CC_IPA_SIGNATURE_INVALID,
    CC_IPA_SIGNATURE_UNAVAILABLE
} cc_ipa_signature_status;
/* Verify the complete retained file and captured signature without loading code.
 * Uses only the build's embedded RSA public key (SHA-256, PKCS#1 v1.5).
 * Returns 0 with a status, or a negative operational errno; status stays
 * unchanged on error. Changed/replaced files return -ESTALE. Crypto-disabled
 * builds report UNAVAILABLE. A readable empty signature is INVALID.
 * Modules and dependencies must remain trusted and unchanged through loading;
 * this is not an immutable file snapshot or dependency-signing mechanism. */
CC_API int cc_ipa_module_verify_signature(const cc_ipa_module *module, cc_ipa_signature_status *status);
/* Load trusted native code in this process with RTLD_NOW|RTLD_LOCAL, then
 * instantiate an ordinary C IPA. Loading is lazy and shared by all instances.
 * Detects replacement/modification since inspection (-ESTALE), missing factory
 * (-ENOENT), and dynamic-link failure (-ENOEXEC). Do not modify module files or
 * their dependencies concurrently. Inspection/signature-byte access alone is not a trust
 * check or sandbox. Linux /proc/self/maps identifies the loaded file, rejecting
 * stale dynamic-linker path caches. This does not load upstream C++ IPA binaries.
 * *ipa is NULL on error; success may be passed to cc_capture_set_ipa as usual. */
CC_API int cc_ipa_module_create(cc_ipa_module *module, const void *data, size_t size, cc_ipa **ipa);

#ifdef __cplusplus
}
#endif
#endif
