/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_EXAMPLE_PPM_H
#define CCAMERA_EXAMPLE_PPM_H
#include <ccamera/ccamera.h>
#include <errno.h>
#include <stdio.h>

static int ppm(const char *path, const cc_frame *frame, const cc_format *format)
{
    if (frame->num_planes != 1 || format->fourcc != CC_FOURCC('R', 'G', 'B', '3'))
        return -ENOTSUP;
    const cc_plane *plane = &frame->planes[0];
    uint32_t stride = format->planes[0].stride;
    uint64_t row_bytes = (uint64_t)format->width * 3;
    if (!format->height || !row_bytes || row_bytes > stride || !plane->data ||
        plane->data_offset > plane->bytes_used || plane->bytes_used > plane->length ||
        (uint64_t)(format->height - 1) * stride + row_bytes > plane->bytes_used - plane->data_offset)
        return -EPROTO;
    FILE *out = fopen(path, "wb");
    if (!out)
        return -errno;
    int ret = fprintf(out, "P6\n%u %u\n255\n", format->width, format->height) < 0 ? -EIO : 0;
    const unsigned char *image = (const unsigned char *)plane->data + plane->data_offset;
    for (uint32_t y = 0; !ret && y < format->height; ++y) {
        if (fwrite(image + (size_t)y * stride, 1, (size_t)row_bytes, out) != row_bytes)
            ret = -EIO;
    }
    if (fclose(out) && !ret)
        ret = -EIO;
    return ret;
}
#endif
