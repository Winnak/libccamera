/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_MEDIA_TOPOLOGY_H
#define CCAMERA_MEDIA_TOPOLOGY_H
#include "internal.h"
struct cc_media_topology {
    struct media_v2_topology info;
    struct media_v2_entity *entities;
    struct media_v2_interface *interfaces;
    struct media_v2_pad *pads;
    struct media_v2_link *links;
    uint32_t *data_link_ids;
};
int cc_media_populate_v2(struct cc_media *media);
void cc_media_topology_free(struct cc_media_topology *topology);
int cc_media_topology_current(struct cc_media *media);
int cc_media_topology_lens(struct cc_media *media,uint32_t sensor,struct cc_entity **lens);
void cc_media_topology_link_changed(struct cc_media *media,size_t index,uint32_t flags);
#endif
