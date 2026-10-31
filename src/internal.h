/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_INTERNAL_H
#define CCAMERA_INTERNAL_H

#include <ccamera/ccamera.h>
#include <ccamera/stream.h>
#include <ccamera/vc4_isp.h>
#include <ccamera/cfe.h>
#include <ccamera/pisp_be.h>
#include <ccamera/uvc.h>
#include <linux/media.h>
#include <time.h>
#include <linux/videodev2.h>
#include <linux/v4l2-subdev.h>
#include <stdbool.h>
#include <poll.h>
#include <sys/types.h>

struct cc_io_ops {
    int (*open)(void *, const char *, int);
    int (*close)(void *, int);
    int (*ioctl)(void *, int, unsigned long, void *);
    int (*map)(void *, int, size_t, off_t, void **);
    int (*unmap)(void *, void *, size_t);
    int (*lock)(void *, int);
    int (*wait)(void *, int, short, int);
    int (*now)(void *, uint64_t *);
    int (*resolve)(void *, unsigned int, unsigned int, char *, size_t);
    int (*duplicate)(void *, int);
    int (*wait_many)(void *, struct pollfd *, size_t, int);
    int (*identity)(void *, int, uint64_t *, uint64_t *);
};
struct cc_io { const struct cc_io_ops *ops; void *ctx; };
extern const struct cc_io cc_system_io;

struct cc_entity {
    struct media_entity_desc desc;
    char name[64]; /* Full v2 name; desc.name is the legacy-width compatibility view. */
    struct media_pad_desc *pads;
    char path[CC_PATH_MAX];
};
struct cc_media {
    struct cc_io io;
    int fd;
    struct media_device_info info;
    struct cc_entity *entities;
    size_t num_entities;
    struct media_link_desc *links;
    size_t num_links;
    struct cc_media_topology *topology;
};
static inline const char *cc_entity_name(const struct cc_entity *entity)
{
    return entity->name[0] ? entity->name:entity->desc.name;
}

int cc_media_open(const struct cc_io *io, const char *path, bool exclusive, struct cc_media **out);
void cc_media_close(struct cc_media *media);
struct cc_entity *cc_media_entity(struct cc_media *media, uint32_t id);
int cc_media_route(struct cc_media *media, uint32_t sensor, uint32_t video,
                   size_t **links, size_t *count);
int cc_media_route_pad(struct cc_media *media, uint32_t sensor, uint32_t pad, uint32_t video,
                        size_t **links, size_t *count);
int cc_media_set_link(struct cc_media *media, size_t index, uint32_t flags);
int cc_media_lens(struct cc_media *media, uint32_t sensor, struct cc_entity **lens);
int cc_camera_open_io(const cc_camera_info *info, const struct cc_io *io, cc_camera **out);
int cc_enumerate_media_io(const struct cc_io *io, const char *path, cc_camera_info **list, size_t *count);
struct cc_request;
int cc_request_set_buffer_io(struct cc_request *request, unsigned int stream,
                             const cc_dmabuf_plane *planes, uint32_t num_planes, const struct cc_io *io);
int cc_request_set_acquire_fence_io(struct cc_request *request, unsigned int stream, int fd, const struct cc_io *io);
int cc_stream_clear_imports(cc_stream *stream);
int cc_stream_canonical_export(cc_stream *stream, uint32_t index, uint32_t plane);
cc_stream *cc_cfe_image_stream(cc_cfe *cfe);
cc_stream *cc_vc4_image_stream(cc_vc4_isp *isp, uint32_t output);
cc_stream *cc_be_image_stream(cc_pisp_be *be, uint32_t output);
cc_stream *cc_cfe_aux_stream(cc_cfe *cfe, uint32_t index);
cc_stream *cc_vc4_statistics_stream(cc_vc4_isp *isp);
cc_stream *cc_be_parameters_stream(cc_pisp_be *be);
cc_stream *cc_be_aux_output_stream(cc_pisp_be *be, uint32_t output);
int cc_query_capture(const struct cc_io *io, int fd, struct v4l2_capability *caps,
                     enum v4l2_buf_type *type);

struct cc_buffer {
    bool queued, bound;
    bool mapped[CC_MAX_PLANES];
    cc_frame frame;
    off_t offsets[CC_MAX_PLANES];
    int fds[CC_MAX_PLANES];
};

struct cc_video_device {
    struct cc_io io;
    int fd;
    uint32_t references, active_types, capabilities;
    struct v4l2_capability caps;
    struct cc_stream *streams;
};
struct cc_stream {
    struct cc_video_device *device;
    struct cc_stream *next;
    enum v4l2_buf_type type;
    enum v4l2_memory memory;
    bool configured, streaming, poisoned, release_pending, allocated;
    cc_format format;
    struct cc_buffer *buffers;
    uint32_t num_buffers, queued;
};

int cc_video_open_io(const char *path, const struct cc_io *io, cc_video_device **out);
int cc_control_query_io(const struct cc_io *io, int fd, uint32_t id, cc_control_info *info);
int cc_control_menu_io(const struct cc_io *io, int fd, uint32_t id, uint32_t index, cc_control_menu *entry);
int cc_uvc_match(struct cc_media *media, struct cc_entity **video);
int cc_uvc_open_media(cc_camera *camera);
int cc_uvc_validate_settings(const cc_uvc_controls *settings);
int cc_controls_io(const struct cc_io *io, int fd, const cc_control *input, cc_control *output,
                    size_t count, size_t *error_index, bool layout_busy, bool *layout_changed);
int cc_controls_apply_io(const struct cc_io *io, int fd, cc_control *controls, size_t count, bool layout_busy);
int cc_camera_apply_controls(cc_camera *camera, cc_control *controls, size_t count);
int cc_cfe_apply_controls(cc_cfe *cfe, cc_control *controls, size_t count);
int cc_vc4_isp_open_io(const char *path, const struct cc_io *io, cc_vc4_isp **out);
int cc_cfe_open_io(const cc_camera_info *info, const struct cc_io *io, cc_cfe **out);
int cc_pisp_be_open_io(const char *path, const struct cc_io *io, cc_pisp_be **out);
int cc_cfe_capture_info(cc_cfe *cfe, const struct cc_io **io, cc_cfe_configuration *configuration, uint32_t *count);
int cc_vc4_capture_info(cc_vc4_isp *isp, cc_vc4_configuration *configuration, uint32_t *count);
int cc_be_capture_info(cc_pisp_be *be, cc_pisp_be_configuration *configuration, uint32_t *count);
size_t cc_cfe_pending_fds(const cc_cfe *cfe, struct pollfd *fds);
size_t cc_vc4_pending_fds(const cc_vc4_isp *isp, struct pollfd *fds);
size_t cc_be_pending_fds(const cc_pisp_be *be, struct pollfd *fds);
int cc_camera_change_link(cc_camera *camera, size_t index, uint32_t flags);
uint32_t cc_bayer_mbus(uint32_t fourcc);
uint32_t cc_cfe_output_fourcc(uint32_t sensor_fourcc);

struct cc_camera {
    struct cc_io io;
    cc_camera_info info;
    int fd;
    cc_video_device *device;
    cc_stream *stream;
    struct cc_media *media;
    size_t *route;
    size_t route_count;
    int *subdev_fds; /* Indexed by media entity position. */
    bool *subdev_streams;
    uint32_t *saved_link_flags;
    bool *changed_links;
    int sensor_fd;
    int lens_fd; /* Lazily opened associated lens; -1 when not acquired. */
    uint32_t sensor_pad;
    uint32_t sensor_stream;
    struct cc_embedded_capture *embedded;
    struct cc_vimc *vimc;
};

int cc_vimc_match(struct cc_media *media,struct cc_entity **sensor,struct cc_entity **video,struct cc_entity **raw);
int cc_vimc_route(struct cc_media *media,size_t **route,size_t *count);
int cc_vimc_open(cc_camera *camera);
void cc_vimc_close(cc_camera *camera);
int cc_vimc_negotiate(cc_camera *camera,cc_format *format,bool active);
int cc_vimc_enum_format(cc_camera *camera,uint32_t index,cc_format_info *format);
int cc_vimc_enum_size(cc_camera *camera,uint32_t fourcc,uint32_t index,cc_size_range *size);

int cc_sensor_streams_discover(cc_camera *camera, uint32_t which, cc_sensor_streams *streams);
int cc_sensor_streams_image_route(cc_camera *camera, uint32_t which, uint32_t *stream);
int cc_sensor_streams_route(cc_camera *camera, uint32_t which, bool embedded, uint32_t *stream);
size_t cc_camera_pending_fds(const cc_camera *camera, struct pollfd *fds);

static inline int cc_ioctl(const struct cc_io *io, int fd, unsigned long req, void *arg)
{
    return io->ops->ioctl(io->ctx, fd, req, arg);
}

static inline bool cc_multiplanar(const cc_stream *stream)
{
    return stream->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE ||
           stream->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
}

void cc_copy_string(char *dst, size_t capacity, const char *src, size_t length);
#endif
