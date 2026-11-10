/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_IPA_MESSAGE_H
#define CCAMERA_IPA_MESSAGE_H
#include "ipa_wire.h"
#include <ccamera/ipc_pipe.h>

/* The envelope uses inline bytes or one sealed memfd for a larger snapshot.
 * All data remains bounded by WIRE_MAX_DATA. No pointer or arbitrary fd number
 * is carried in the byte protocol. Sending does not retain caller storage.
 * Decode copies verified immutable data; outputs are NULL/0 on error. */
int cc_ipa_message_send(cc_ipc_pipe *pipe, const cc_ipc_message_header *header, const void *data, size_t size);
int cc_ipa_message_call(cc_ipc_pipe *pipe, const cc_ipc_message_header *header, const void *data, size_t size,
                        cc_ipc_packet **response, int timeout_ms);
int cc_ipa_message_decode(const cc_ipc_packet *packet, void **data, size_t *size);
int cc_ipa_message_send_fds(cc_ipc_pipe *pipe, const cc_ipc_message_header *header, const void *data, size_t size,
                            const int *fds, uint32_t num_fds);
int cc_ipa_message_call_fds(cc_ipc_pipe *pipe, const cc_ipc_message_header *header, const void *data, size_t size,
                            const int *fds, uint32_t num_fds, cc_ipc_packet **response, int timeout_ms);
/* Additional fds are borrowed from packet; the sealed snapshot fd is excluded. */
int cc_ipa_message_decode_fds(const cc_ipc_packet *packet, void **data, size_t *size, const int **fds, uint32_t *num_fds);
#endif
