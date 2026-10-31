/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_VC4_ISP_H
#define CCAMERA_VC4_ISP_H
#include <ccamera/ccamera.h>
#include <ccamera/stream.h>
#include <ccamera/vc4_tuning.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct cc_vc4_isp cc_vc4_isp;
enum cc_vc4_port {
    CC_VC4_INPUT = 0,
    CC_VC4_MAIN = 1,
    CC_VC4_LORES = 2,
    CC_VC4_STATS = 3
};
typedef struct cc_vc4_configuration {
    cc_format input;
    cc_format output[2];
    cc_format statistics; /* Returned native BSTA metadata format. */
    cc_rectangle crop;    /* Zero size requests a centered aspect-ratio crop. */
} cc_vc4_configuration;
typedef struct cc_vc4_result {
    uint32_t index;
    uint64_t cookie;
    cc_frame input;       /* Source metadata, data pointers are NULL. */
    cc_frame output[2];
    cc_frame statistics;
} cc_vc4_result;

/* Native bcm2835-isp processing, for Pi 4 and earlier. NULL media_path selects
 * an accessible instance by its media driver/entities, without fixed video
 * numbers. No tuning algorithm or encoder is selected implicitly. Serialize
 * all operations on an instance. */
CC_API int cc_vc4_isp_open(const char *media_path, cc_vc4_isp **isp);
CC_API void cc_vc4_isp_close(cc_vc4_isp *isp);
CC_API int cc_vc4_isp_configure(cc_vc4_isp *isp, cc_vc4_configuration *configuration);
/* output[0] is required; a zero output[1] selects a small YUV420 auxiliary
 * stream. Input must match the producer's negotiated memory layout exactly.
 * Actual output/metadata layouts and crop are returned on success. */
CC_API int cc_vc4_isp_allocate_buffers(cc_vc4_isp *isp, uint32_t *count);
/* Per-image MMAP/DMABUF selection; input stays DMABUF and statistics MMAP.
 * DMABUF outputs must be imported before submit. The default allocator above
 * uses MMAP for both outputs. Release/stop clears imported output bindings. */
CC_API int cc_vc4_isp_allocate_buffers_with_memory(cc_vc4_isp *isp, const enum cc_memory outputs[2], uint32_t *count);
CC_API int cc_vc4_isp_import_output(cc_vc4_isp *isp, uint32_t output, uint32_t index,
                                   const cc_dmabuf_plane *planes, uint32_t num_planes);
CC_API int cc_vc4_isp_free_buffers(cc_vc4_isp *isp);
/* One processing job is in flight at a time; completed outputs can be held
 * while a different slot runs. Submit duplicates the input fds. The producer
 * buffer must remain dequeued until completion, successful stop, or an error
 * that cancels the job. Close caller export fds after submit returns. */
/* Set all profile controls while no job is active. Grid storage is backend-owned.
 * Use immediately before submit; explicit native controls can then override it. */
CC_API int cc_vc4_isp_set_tuning(cc_vc4_isp *isp, const cc_vc4_tuning *tuning);
CC_API int cc_vc4_isp_submit_tuned(cc_vc4_isp *isp, uint32_t index,
    const cc_dmabuf_plane *planes, uint32_t num_planes, const cc_frame *input,
    uint64_t cookie, const cc_vc4_tuning *tuning);
CC_API int cc_vc4_isp_submit(cc_vc4_isp *isp, uint32_t index,
                           const cc_dmabuf_plane *planes, uint32_t num_planes,
                           const cc_frame *input, uint64_t cookie);
/* Dequeues input, both outputs and statistics as one result. 0 timeout is
 * nonblocking (-EAGAIN); >0 bounds the entire operation (-ETIMEDOUT); -1 waits.
 * A timeout leaves the job pending and output unchanged. Driver/protocol errors
 * cancel jobs and invalidate held results; failure to cancel closes all ISP
 * handles before releasing input references and requires reopening the ISP.
 * Completion closes the backend's duplicated input fds. Kernel attachments can
 * remain until free_buffers/close: free ISP buffers before producer buffers. */
CC_API int cc_vc4_isp_dequeue(cc_vc4_isp *isp, cc_vc4_result *result, int timeout_ms);
/* Result views remain valid until release, stop, free, or close. Release does
 * not queue more work. Consumers using exported output fds must finish access
 * before releasing/reusing a slot, and close fds before freeing its buffers. */
CC_API int cc_vc4_isp_release(cc_vc4_isp *isp, uint32_t index);
CC_API int cc_vc4_isp_stop(cc_vc4_isp *isp);
CC_API int cc_vc4_isp_export_plane(cc_vc4_isp *isp, enum cc_vc4_port port, uint32_t index, uint32_t plane);
/* Borrowed fds for an event loop. INPUT uses POLLOUT; other ports use POLLIN. */
CC_API int cc_vc4_isp_fd(const cc_vc4_isp *isp, enum cc_vc4_port port);
CC_API int cc_vc4_isp_query_control(cc_vc4_isp *isp, uint32_t id, cc_control_info *info);
CC_API int cc_vc4_isp_query_menu(cc_vc4_isp *isp,uint32_t id,uint32_t index,cc_control_menu *entry);
CC_API int cc_vc4_isp_next_control(cc_vc4_isp *isp, uint32_t after_id, cc_control_info *info);
CC_API int cc_vc4_isp_get_controls(cc_vc4_isp *isp, cc_control *controls, size_t count, size_t *error_index);
CC_API int cc_vc4_isp_set_controls(cc_vc4_isp *isp, const cc_control *controls, size_t count, size_t *error_index);

#ifdef __cplusplus
}
#endif
#endif
