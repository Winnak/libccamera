/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_IPA_VIMC_H
#define CCAMERA_IPA_VIMC_H
#include <ccamera/ipa_proxy.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Native C adaptation of the reference VIMC test IPA. Configuration bytes are
 * supplied explicitly rather than through a tuning-file locator. The mock
 * ignores their contents but requires a nonempty configuration. */
enum cc_ipa_vimc_operation { CC_IPA_VIMC_NONE, CC_IPA_VIMC_INIT, CC_IPA_VIMC_START, CC_IPA_VIMC_STOP };
enum cc_ipa_vimc_flags { CC_IPA_VIMC_FLAG1 = 1U, CC_IPA_VIMC_FLAG2 = 2U, CC_IPA_VIMC_FLAG3 = 4U, CC_IPA_VIMC_FLAG4 = 8U };
typedef struct cc_ipa_vimc_options {
    const void *configuration;
    size_t configuration_size;
    int trace_fd; /* -1 disables. Borrowed nonblocking writer; duplicated. */
    enum cc_ipa_vimc_operation operation;
    uint32_t flags;
} cc_ipa_vimc_options;
#define CC_IPA_VIMC_MAX_BUFFERS 16U
#define CC_IPA_VIMC_MAX_MAPPED_BYTES (16U * 1024U * 1024U)
typedef struct cc_ipa_vimc_plane { int fd; uint64_t offset, length; } cc_ipa_vimc_plane;
typedef struct cc_ipa_vimc_buffer { uint32_t id, num_planes; cc_ipa_vimc_plane planes[CC_MAX_PLANES]; } cc_ipa_vimc_buffer;
typedef struct cc_ipa_vimc_event { uint32_t buffer_id, flags; } cc_ipa_vimc_event;
/* Returns an ordinary IPA for the existing VIMC capture graph. The graph's
 * start/stop maps/unmaps two mock parameter buffers; each request sends a
 * queue operation and validates a computed-parameters notification. No image
 * algorithm runs. Init returns FLAG1, as in the reference. *ipa=NULL on error;
 * flags is unchanged on error. trace writes native uint32 operation values
 * INIT/START/STOP and are best effort. All object calls must be serialized. */
CC_API int cc_ipa_vimc_create_proxy(const cc_ipa_vimc_options *controller_options,
                                    const cc_ipa_proxy_options *options, cc_ipa **ipa, uint32_t *flags);
/* Explicit mock-buffer and asynchronous IPC operations for tests/diagnostics.
 * Use on a stopped, unattached VIMC proxy; capture manages its own buffers.
 * Map snapshots descriptors but shares their underlying memory read-only in
 * the worker. Caller descriptors may be closed after success. Duplicate IDs
 * are rejected atomically. Unknown unmap IDs are ignored, as in the reference.
 * queue/compute acceptance is asynchronous; -EBUSY means retry after dispatch.
 * Computing an unknown buffer ID emits no event. */
CC_API int cc_ipa_vimc_map_buffers(cc_ipa *ipa, const cc_ipa_vimc_buffer *buffers, size_t count);
CC_API int cc_ipa_vimc_unmap_buffers(cc_ipa *ipa, const uint32_t *ids, size_t count);
CC_API int cc_ipa_vimc_queue_request(cc_ipa *ipa, uint32_t frame, const cc_control *controls, size_t count);
CC_API int cc_ipa_vimc_compute_params(cc_ipa *ipa, uint32_t frame, uint32_t buffer_id);
CC_API int cc_ipa_vimc_dispatch(cc_ipa *ipa);
/* 0 polls (-EAGAIN), -1 waits indefinitely. Output unchanged on error. */
CC_API int cc_ipa_vimc_dequeue_event(cc_ipa *ipa, cc_ipa_vimc_event *event, int timeout_ms);

#ifdef __cplusplus
}
#endif
#endif
