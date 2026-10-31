/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_STREAM_H
#define CCAMERA_STREAM_H

#include <ccamera/ccamera.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct cc_video_device cc_video_device;
typedef struct cc_stream cc_stream;

/* Values correspond to Linux v4l2_buf_type, without requiring kernel headers
 * in applications. OUTPUT sends bytes to hardware; CAPTURE receives them. */
enum cc_stream_type {
    CC_STREAM_CAPTURE = 1,
    CC_STREAM_OUTPUT = 2,
    CC_STREAM_CAPTURE_MPLANE = 9,
    CC_STREAM_OUTPUT_MPLANE = 10,
    CC_STREAM_META_CAPTURE = 13,
    CC_STREAM_META_OUTPUT = 14
};
enum cc_memory {
    CC_MEMORY_MMAP = 1,
    CC_MEMORY_DMABUF = 4
};

/* Low-level queues for pipeline implementations. These functions do not
 * configure sensors/media links or choose an ISP/codec. Use cc_camera for
 * complete raw camera capture. Serialize calls sharing a device, including its
 * streams. No background threads or additional libraries are required. */
CC_API int cc_video_open(const char *path, cc_video_device **device);
/* Drops the owner's reference. Existing streams retain the same device fd;
 * capture and output streams on an M2M device must use this same open handle. */
CC_API void cc_video_close(cc_video_device *device);
/* Native V4L2 controls, including ISP compound payloads. Layout-changing
 * controls require all queues on this handle to have no allocated buffers and
 * invalidate their configurations. error_index follows the camera API. */
CC_API int cc_video_query_control(cc_video_device *device, uint32_t id, cc_control_info *info);
CC_API int cc_video_query_menu(cc_video_device *device, uint32_t id, uint32_t index, cc_control_menu *entry);
CC_API int cc_video_next_control(cc_video_device *device, uint32_t after_id, cc_control_info *info);
CC_API int cc_video_get_controls(cc_video_device *device, cc_control *controls, size_t count, size_t *error_index);
CC_API int cc_video_set_controls(cc_video_device *device, const cc_control *controls, size_t count, size_t *error_index);
CC_API int cc_video_create_stream(cc_video_device *device, enum cc_stream_type type, cc_stream **stream);
/* Cancels and frees this queue. If the driver refuses to stop/release it,
 * closes the underlying device for DMA safety. Other streams on that handle
 * then return -ENODEV and must be closed too. */
CC_API void cc_stream_close(cc_stream *stream);
/* Borrowed nonblocking fd. Never close it; it may be shared by two queues. */
CC_API int cc_stream_fd(const cc_stream *stream);
/* POLLIN for capture, POLLOUT for output, for the caller's poll/epoll loop. */
CC_API int cc_stream_poll_events(const cc_stream *stream);
CC_API int cc_stream_enum_format(cc_stream *stream, uint32_t index, cc_format_info *format);
CC_API int cc_stream_get_format(cc_stream *stream, cc_format *format);
CC_API int cc_stream_try_format(cc_stream *stream, cc_format *format);
CC_API int cc_stream_configure(cc_stream *stream, cc_format *format);
/* Image queues only, optional native support. Get/set require configuration;
 * set requires every queue sharing this device to have no allocated buffers.
 * Existing capture/output/extended modes are preserved. {0,0} requests reset
 * to the driver default. Returned fractions/ranges are exact native values;
 * successful discrete enumeration has equal min/max and step={0,0}. */
CC_API int cc_stream_enum_frame_interval(cc_stream *stream,uint32_t fourcc,uint32_t width,uint32_t height,
                                        uint32_t index,cc_frame_interval_range *range);
CC_API int cc_stream_get_frame_interval(cc_stream *stream,cc_fraction *interval);
CC_API int cc_stream_set_frame_interval(cc_stream *stream,cc_fraction *interval);
/* Native V4L2 selection targets/flags. Setting a selection returns the actual
 * rectangle. If it changes the buffer layout, reconfigure before allocating;
 * a layout change with allocated buffers requires stop/free/reconfigure. */
CC_API int cc_stream_get_selection(cc_stream *stream, uint32_t target, cc_rectangle *rectangle);
CC_API int cc_stream_set_selection(cc_stream *stream, uint32_t target, uint32_t flags, cc_rectangle *rectangle);
/* Metadata formats have one plane: fourcc is dataformat, planes[0].size is
 * buffersize. Width/height/stride are zero except for line-based metadata.
 * Image plane strides/sizes may be requested by setting num_planes and the
 * corresponding plane entries; actual values are returned. */
CC_API int cc_stream_allocate_buffers(cc_stream *stream, enum cc_memory memory, uint32_t *count);
CC_API int cc_stream_free_buffers(cc_stream *stream);
/* DMABUF slots must be bound before queueing. Duplicates all fds with CLOEXEC;
 * caller fds may be closed after success. Rebinding an idle slot is atomic on
 * failure. Each fd addresses a whole memory plane starting at offset zero.
 * Imported buffers are not CPU mapped by this library. */
CC_API int cc_stream_import(cc_stream *stream, uint32_t index,
                            const cc_dmabuf_plane *planes, uint32_t num_planes);
/* Unbind an idle DMABUF slot and close its duplicated fds. The kernel may keep
 * its last attachment until that slot is rebound or the queue is freed. */
CC_API int cc_stream_unimport(cc_stream *stream, uint32_t index);
/* Idle/dequeued MMAP buffer data is writable, permitting ISP configuration
 * payloads. Imported planes have data == NULL and length set to import size.
 * All pointers become inaccessible on queue, free, or close. */
CC_API int cc_stream_buffer(cc_stream *stream, uint32_t index, cc_frame *buffer);
/* Returns a new owned CLOEXEC fd, exporting MMAP or duplicating an import. */
CC_API int cc_stream_export_plane(cc_stream *stream, uint32_t index, uint32_t plane);
/* payload must be NULL for capture. Output requires its plane bytes_used,
 * data_offset, field and timestamp_ns; bytes_used includes data_offset.
 * Single-plane output requires data_offset == 0. Queue flags are initialized
 * by the library; driver completion flags are returned unchanged on dequeue. */
CC_API int cc_stream_queue(cc_stream *stream, uint32_t index, const cc_frame *payload);
CC_API int cc_stream_start(cc_stream *stream);
CC_API int cc_stream_dequeue(cc_stream *stream, cc_frame *frame, int timeout_ms);
CC_API int cc_stream_stop(cc_stream *stream);

#ifdef __cplusplus
}
#endif
#endif
