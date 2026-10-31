/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Endian-independent seeded FNV-1a hashes from the supplied archived upstream decoders:
 * md_parser_smia.cpp and CamHelperImx708::{parsePdafData,parseAEHist}.
 * The development C++ adapter preserves those function bodies; it is external
 * to normal C17 builds. SMIA: 256 packets; PDAF: 6,336 regions; HDR: 6 packets.
 * High-count HDR arithmetic is checked independently because the original
 * multiplies in 32 bits before adding to its 64-bit accumulator. */
static const uint64_t cc_embedded_smia_reference_hash = UINT64_C(5396402878369179011);
static const uint64_t cc_embedded_pdaf_reference_hash = UINT64_C(10567862036016885992);
static const uint64_t cc_embedded_hdr_reference_hash = UINT64_C(7977637069298070500);
