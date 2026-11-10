/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_IPA_PROXY_H
#define CCAMERA_IPA_PROXY_H
#include <ccamera/ipa_manager.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef struct cc_ipa_proxy_options {
    const char *worker_path; /* NULL searches environment/build/install trees. */
    int timeout_ms; /* 0 selects 2000 ms; -1 waits indefinitely. */
} cc_ipa_proxy_options;
/* Create a C IPA in an exec worker with new user and network namespaces.
 * Namespace/kernel/policy errors are returned without reducing isolation.
 * The returned ordinary IPA works with cc_capture_set_ipa. Each instance owns
 * an independent worker; parent code only inspects module metadata. Stop keeps
 * the worker for restart; destroy stops, terminates and reaps it. Calls on each
 * IPA must be serialized. No background thread or event loop is installed.
 * Module-defined factory data must be a relocatable byte schema: process-local
 * pointers/handles in arbitrary data cannot be translated. Standard C IPA
 * callback objects, built-in tuning and payload bytes are serialized deeply.
 * Total serialized data is bounded to 16 MiB per call. Timeout/worker/protocol
 * failure is persistent for that instance; create a new proxy to recover.
 * Worker lookup: explicit path, CCAMERA_IPA_PROXY_PATH directories (ignored
 * when privileged), active build image, then configured libexec directory.
 * This does not verify signatures or load upstream C++ IPA binaries.
 * *ipa=NULL on error; options and data are borrowed only during creation. */
CC_API int cc_ipa_module_create_proxy(cc_ipa_module *module, const void *data, size_t size,
                                      const cc_ipa_proxy_options *options, cc_ipa **ipa);
CC_API int cc_ipa_reference_create_proxy(const cc_ipa_reference_tuning *tuning,
                                         const cc_ipa_proxy_options *options, cc_ipa **ipa);
CC_API int cc_ipa_vc4_create_proxy(const cc_ipa_vc4_options *controller_options,
                                   const cc_ipa_proxy_options *options, cc_ipa **ipa);
CC_API int cc_ipa_manager_create_proxy(cc_ipa_manager *manager, const char *pipeline_name,
                                       uint32_t minimum_version, uint32_t maximum_version, const char *name,
                                       const void *data, size_t size, const cc_ipa_proxy_options *options, cc_ipa **ipa);

#ifdef __cplusplus
}
#endif
#endif
