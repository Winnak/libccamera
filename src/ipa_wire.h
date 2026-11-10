/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_IPA_WIRE_H
#define CCAMERA_IPA_WIRE_H
#include <ccamera/ipa.h>

/* Private, native-endian C IPA protocol. Never serializes process pointers.
 * Native kernel register blocks are copied byte-for-byte. Both ends must use
 * the same wire/IPA ABI and native block sizes. Decode owns all reconstructed
 * pointed-to storage; object views expire on destroy. The ordinary codec does
 * no OS I/O or module execution. */
#define CC_IPA_WIRE_VERSION 2U
#define CC_IPA_WIRE_MAX_DATA (16U * 1024U * 1024U)
#define CC_IPA_WIRE_MAX_FDS 192U
enum cc_ipa_wire_kind {
    CC_IPA_WIRE_CONFIGURATION = 1, CC_IPA_WIRE_REQUEST, CC_IPA_WIRE_SETTINGS,
    CC_IPA_WIRE_FRAME, CC_IPA_WIRE_METADATA, CC_IPA_WIRE_REFERENCE, CC_IPA_WIRE_VC4
};
typedef struct cc_ipa_wire cc_ipa_wire;
int cc_ipa_wire_encode(enum cc_ipa_wire_kind kind, const void *object, void **data, size_t *size);
int cc_ipa_wire_decode(enum cc_ipa_wire_kind kind, const void *data, size_t size, cc_ipa_wire **wire);
/* Native adapter variants. Encode publishes borrowed descriptor values, with
 * indexes in the bytes. Decode duplicates supplied actual descriptors and owns
 * them until destroy. The ordinary decoder remains pure and rejects fd tables. */
int cc_ipa_wire_encode_fds(enum cc_ipa_wire_kind kind, const void *object, void **data, size_t *size,
                           int fds[CC_IPA_WIRE_MAX_FDS], uint32_t *num_fds);
int cc_ipa_wire_decode_fds(enum cc_ipa_wire_kind kind, const void *data, size_t size,
                           const int *fds, uint32_t num_fds, cc_ipa_wire **wire);
const void *cc_ipa_wire_object(const cc_ipa_wire *wire);
void cc_ipa_wire_destroy(cc_ipa_wire *wire);
#endif
