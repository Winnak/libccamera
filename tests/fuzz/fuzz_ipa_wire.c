/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "ipa_wire.h"
#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    if (!size || size > CC_IPA_WIRE_MAX_DATA) return 0;
    enum cc_ipa_wire_kind kind = (enum cc_ipa_wire_kind)data[0];
    cc_ipa_wire *wire = (void *)(uintptr_t)1;
    int ret = cc_ipa_wire_decode(kind, data + 1, size - 1, &wire);
    if (ret < 0) { assert(!wire); return 0; }
    assert(wire && cc_ipa_wire_object(wire));
    void *encoded = NULL, *canonical = NULL; size_t encoded_size = 0, canonical_size = 0;
    ret = cc_ipa_wire_encode(kind, cc_ipa_wire_object(wire), &encoded, &encoded_size);
    if (!ret) {
        cc_ipa_wire *copy = NULL; int decoded = cc_ipa_wire_decode(kind, encoded, encoded_size, &copy);
        if (!decoded) {
            int encoded_again = cc_ipa_wire_encode(kind, cc_ipa_wire_object(copy), &canonical, &canonical_size);
            if (!encoded_again) assert(encoded_size == canonical_size && !memcmp(encoded, canonical, encoded_size));
            assert(!encoded_again || encoded_again == -ENOMEM);
        }
        /* Resource exhaustion is allowed; canonical protocol errors are not. */
        assert(!decoded || decoded == -ENOMEM);
        cc_ipa_wire_destroy(copy);
    }
    assert(!ret || ret == -ENOMEM);
    free(encoded); free(canonical); cc_ipa_wire_destroy(wire); return 0;
}
