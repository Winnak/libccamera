/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_IPC_PIPE_H
#define CCAMERA_IPC_PIPE_H
#include <ccamera/ipc.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CC_IPC_PIPE_DEFAULT_TIMEOUT_MS 2000
#define CC_IPC_PIPE_MAX_PENDING 64U
#define CC_IPC_PIPE_MAX_PENDING_BYTES (1024U * 1024U)
#define CC_IPC_PIPE_MAX_PENDING_FDS 256U
typedef struct cc_ipc_pipe cc_ipc_pipe;

/* Adopt *channel on success, setting it to NULL. peer_fd is -1 or a borrowed
 * exit-readiness descriptor (normally a pidfd), duplicated with CLOEXEC. Its
 * readiness fails the pipe after draining received messages. No threads or
 * callbacks: incoming notifications are queued for receive, including during
 * a synchronous call. Calls on a pipe must be serialized. *pipe=NULL on error. */
CC_API int cc_ipc_pipe_open(cc_ipc **channel, int peer_fd, cc_ipc_pipe **pipe);
CC_API void cc_ipc_pipe_close(cc_ipc_pipe *pipe);
/* Borrowed socket/peer descriptors for polling. Missing peer: -ENODATA.
 * events includes POLLOUT while an accepted send remains pending. */
CC_API int cc_ipc_pipe_fd(const cc_ipc_pipe *pipe);
CC_API int cc_ipc_pipe_peer_fd(const cc_ipc_pipe *pipe);
CC_API int cc_ipc_pipe_events(const cc_ipc_pipe *pipe, short *events);
CC_API size_t cc_ipc_pipe_pending(const cc_ipc_pipe *pipe);
/* Flush and receive without blocking, with a bounded receive budget. Unknown
 * cookies are notifications; a synchronous reply matches the active cookie.
 * Errors fail the pipe, releasing pending transport resources. Already queued
 * notifications remain readable. Overflow returns -ENOBUFS. */
CC_API int cc_ipc_pipe_dispatch(cc_ipc_pipe *pipe);
/* Snapshot and accept one message, as with cc_ipc_send_message. An occupied
 * transport send slot returns -EBUSY. The caller can release bytes/fds on 0. */
CC_API int cc_ipc_pipe_send(cc_ipc_pipe *pipe, const cc_ipc_message_header *header,
                            const cc_ipc_payload *payload);
/* Receive a queued notification (or an ordinary incoming message when no call
 * is active). timeout=0 polls (-EAGAIN), -1 waits indefinitely. Positive timeout
 * expires with -ETIMEDOUT without failing the pipe. *packet=NULL on error. */
CC_API int cc_ipc_pipe_receive(cc_ipc_pipe *pipe, cc_ipc_packet **packet, int timeout_ms);
/* Send and wait for the same cookie. Previously accepted output is flushed
 * first, within the same deadline. Use DEFAULT_TIMEOUT_MS for the archived 2s
 * policy; -1 waits indefinitely. Notifications are retained without invoking
 * application callbacks. Timeout after acceptance fails the pipe, preventing
 * uncertain remote state or a late reply from contaminating a later call.
 * Received packets include the command/cookie prefix and own their fds.
 * *response=NULL on error; a NULL response discards a successful reply. */
CC_API int cc_ipc_pipe_call(cc_ipc_pipe *pipe, const cc_ipc_message_header *header,
                            const cc_ipc_payload *payload, cc_ipc_packet **response, int timeout_ms);

#ifdef __cplusplus
}
#endif
#endif
