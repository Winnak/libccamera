/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_IPC_INTERNAL_H
#define CCAMERA_IPC_INTERNAL_H
#include <ccamera/ipc.h>
#include <stdbool.h>
#include <sys/socket.h>

#define CC_IPC_RIGHTS_CAPACITY 256U
#define CC_IPC_CONTROL_SIZE CMSG_SPACE(CC_IPC_RIGHTS_CAPACITY * sizeof(int))
/* Pure wire/ancillary decoders. Rights decoding returns all parsed descriptor
 * values even on error (including unsupported Linux pidfd records) so the
 * native caller can close them. It does no I/O. */
int cc_ipc_decode_header(const void *data, size_t size, uint32_t *length, uint32_t *fds);
int cc_ipc_decode_rights(const void *data, size_t size, int fds[CC_IPC_RIGHTS_CAPACITY], uint32_t *count);
struct cc_ipc_io {
    ssize_t (*send_message)(void *context, int fd, const struct msghdr *message, int flags);
    ssize_t (*receive_message)(void *context, int fd, struct msghdr *message, int flags);
};
/* Internal syscall fault seam; borrowed callbacks/context must outlive channel.
 * NULL restores native calls. The callbacks return byte counts/negative errno. */
void cc_ipc_set_io(cc_ipc *channel, const struct cc_ipc_io *io, void *context);
#endif
