/* SPDX-License-Identifier: LGPL-2.1-or-later AND BSD-2-Clause */
/* Generated from the supplied upstream RPi AF state machine with C++ standard
 * container/metadata adapters, independently of src/af.c. Reference copyright
 * (C) 2022-2023 Raspberry Pi Ltd; see LICENSES/BSD-2-Clause.txt. Six 300-update
 * traces cover CDAF, flat scenes, PDAF, dropout/CAF scene changes, windows and IR.
 * FNV hashes serialize state, pause, validity and native lens code as LE u32s. */
#ifndef CCAMERA_TEST_AF_REFERENCE_VECTORS_H
#define CCAMERA_TEST_AF_REFERENCE_VECTORS_H
#include <stdint.h>
static const uint64_t cc_af_reference_hashes[]={
    UINT64_C(2017129679657177009),UINT64_C(3529151172531622449),UINT64_C(15264016598808092831),
    UINT64_C(14839531945900442279),UINT64_C(5061162185457299000),UINT64_C(17334458221002363889)
};
#endif
