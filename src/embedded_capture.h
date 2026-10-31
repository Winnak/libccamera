/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_EMBEDDED_CAPTURE_H
#define CCAMERA_EMBEDDED_CAPTURE_H
#include "internal.h"
#include <ccamera/embedded.h>
struct cc_embedded_capture {
    cc_video_device *device;
    cc_stream *stream;
    cc_embedded_configuration configuration;
    size_t *path, path_count;
    uint32_t source_count;
    int associated[256], future;
    cc_frame future_frame, image;
    bool image_pending, have_timestamp;
    uint64_t last_timestamp;
};
int cc_embedded_allocate(struct cc_embedded_capture *e, uint32_t *source_count);
int cc_embedded_free(struct cc_embedded_capture *e);
int cc_embedded_start(struct cc_embedded_capture *e);
int cc_embedded_stop(struct cc_embedded_capture *e);
void cc_embedded_close(struct cc_embedded_capture *e);
int cc_embedded_release(struct cc_embedded_capture *e, uint32_t source_index);
int cc_embedded_collect(struct cc_embedded_capture *e, const cc_frame *image, cc_frame *metadata);
#endif
