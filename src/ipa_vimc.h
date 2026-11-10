/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_IPA_VIMC_INTERNAL_H
#define CCAMERA_IPA_VIMC_INTERNAL_H
#include <ccamera/ipa_vimc.h>
#include <ccamera/ipc.h>
struct cc_ipa_vimc;
int cc_ipa_vimc_create(const cc_ipa_vimc_options *options, cc_ipa **ipa, struct cc_ipa_vimc **context, uint32_t *flags);
int cc_ipa_vimc_map(struct cc_ipa_vimc *context, const cc_ipa_vimc_buffer *buffers, size_t count);
void cc_ipa_vimc_unmap(struct cc_ipa_vimc *context, const uint32_t *ids, size_t count);
int cc_ipa_vimc_compute(struct cc_ipa_vimc *context, uint32_t id, cc_ipa_vimc_event *event);
/* Native wire records; fds are SCM_RIGHTS entries in plane order. */
struct cc_ipa_vimc_init { uint32_t operation, flags; };
struct cc_ipa_vimc_map_header { uint32_t id, num_planes; };
struct cc_ipa_vimc_map_plane { uint64_t offset, length; };
_Static_assert(CC_IPA_VIMC_MAX_BUFFERS * CC_MAX_PLANES <= CC_IPC_MAX_FDS, "VIMC maps fit transport descriptor capacity");
#endif
