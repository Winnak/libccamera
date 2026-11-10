/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_IPC_H
#define CCAMERA_IPC_H
#include <ccamera/ccamera.h>
#include <poll.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CC_IPC_MAX_DATA 65536U
#define CC_IPC_MAX_FDS 253U
typedef struct cc_ipc cc_ipc;
typedef struct cc_ipc_packet cc_ipc_packet;
typedef struct cc_ipc_payload {
    const void *data;
    size_t size;
    const int *fds;
    uint32_t num_fds;
} cc_ipc_payload;
typedef struct cc_ipc_message_header { uint32_t command, cookie; } cc_ipc_message_header;

/* Connected nonblocking AF_UNIX datagrams, using the archived IPCUnixSocket
 * framing: an 8-byte native-endian size/fd-count header, then one payload
 * datagram with SCM_RIGHTS. No event loop, threads, filesystem socket or process
 * is created. Both descriptors are CLOEXEC; remote_fd belongs to the caller.
 * *channel is NULL and *remote_fd is -1 on failure. Calls must be serialized. */
CC_API int cc_ipc_create(cc_ipc **channel, int *remote_fd);
/* Duplicate an already connected, nonblocking AF_UNIX/SOCK_DGRAM endpoint.
 * Does not adopt or change the caller's descriptor. Success owns a CLOEXEC
 * duplicate; after binding, no other consumer may read/write that endpoint. */
CC_API int cc_ipc_bind(int fd, cc_ipc **channel);
CC_API void cc_ipc_close(cc_ipc *channel);
/* Borrowed descriptor for polling only. events includes POLLOUT while a send
 * is pending. Invalid/failed channels return a negative errno. */
CC_API int cc_ipc_fd(const cc_ipc *channel);
CC_API int cc_ipc_events(const cc_ipc *channel, short *events);
/* Accept one asynchronous payload, copying bytes and duplicating descriptors
 * with CLOEXEC before sending anything. Success means accepted, not delivered;
 * caller data/fds may be released immediately. An occupied send slot returns
 * -EBUSY. At least one byte or fd is required. Limits return -EMSGSIZE.
 * A partial header/payload send retains the whole snapshot until flush/close.
 * Kernel/protocol failures after transmission begins fail the channel; there
 * is no risk of silently pairing a later payload with the old header. */
CC_API int cc_ipc_send(cc_ipc *channel, const cc_ipc_payload *payload);
/* Nonblocking. 0: drained; -EAGAIN: retained and needs POLLOUT; other negative
 * results: channel failed, pending resources released. */
CC_API int cc_ipc_flush(cc_ipc *channel);
/* Nonblocking. A received header is retained until its payload arrives.
 * On success packet owns copied bytes and all received CLOEXEC descriptors.
 * Malformed/truncated packets fail the channel and close every received fd.
 * *packet is NULL on error, including -EAGAIN. */
CC_API int cc_ipc_receive(cc_ipc *channel, cc_ipc_packet **packet);
CC_API void cc_ipc_packet_destroy(cc_ipc_packet *packet);
/* Borrowed packet contents; NULL packet gives NULL data/fds and zero counts.
 * Descriptor ownership remains with packet unless explicitly taken. */
CC_API const void *cc_ipc_packet_data(const cc_ipc_packet *packet, size_t *size);
CC_API const int *cc_ipc_packet_fds(const cc_ipc_packet *packet, uint32_t *count);
/* Transfer one descriptor to the caller; -ENOENT if absent/already taken. */
CC_API int cc_ipc_packet_take_fd(cc_ipc_packet *packet, uint32_t index);
/* IPCMessage's native-endian command/cookie prefix. The additional 8 bytes
 * count toward MAX_DATA. Header extraction leaves output unchanged on error. */
CC_API int cc_ipc_send_message(cc_ipc *channel, const cc_ipc_message_header *header, const cc_ipc_payload *payload);
CC_API int cc_ipc_packet_header(const cc_ipc_packet *packet, cc_ipc_message_header *header);

#ifdef __cplusplus
}
#endif
#endif
