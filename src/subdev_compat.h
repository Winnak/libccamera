/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/* Linux subdevice UAPI layouts, kept private to support older installed headers.
 * The routing ioctl changed size in Linux 6.10. Try the current ABI first, then
 * the earlier one; neither ABI depends on the running userspace word size. */
#ifndef CCAMERA_SUBDEV_COMPAT_H
#define CCAMERA_SUBDEV_COMPAT_H
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/v4l2-mediabus.h>

/* Stream IDs occupy the former first reserved word, preserving ioctl sizes on
 * older build headers. Use the published layout rather than header-version
 * conditionals or assumptions about the native word size. */
struct cc_subdev_format {
    __u32 which, pad;
    struct v4l2_mbus_framefmt format;
    __u32 stream, reserved[7];
};
struct cc_subdev_mbus_code_enum {
    __u32 pad, index, code, which, flags, stream, reserved[6];
};
struct cc_subdev_frame_size_enum {
    __u32 index, pad, code, min_width, max_width, min_height, max_height, which, stream, reserved[7];
};
struct cc_subdev_frame_interval {
    __u32 pad;
    struct v4l2_fract interval;
    __u32 stream, which, reserved[7];
};
struct cc_subdev_frame_interval_enum {
    __u32 index, pad, code, width, height;
    struct v4l2_fract interval;
    __u32 which, stream, reserved[7];
};
#define CC_SD_G_FRAME_INTERVAL _IOWR('V', 21, struct cc_subdev_frame_interval)
#define CC_SD_S_FRAME_INTERVAL _IOWR('V', 22, struct cc_subdev_frame_interval)
#define CC_SD_ENUM_FRAME_INTERVAL _IOWR('V', 75, struct cc_subdev_frame_interval_enum)
_Static_assert(sizeof(struct cc_subdev_frame_interval)==48,"Linux subdevice frame interval ABI");
_Static_assert(sizeof(struct cc_subdev_frame_interval_enum)==64,"Linux subdevice interval enumeration ABI");

struct cc_subdev_capability {
    __u32 version, capabilities, reserved[14];
};
struct cc_subdev_client_capability { __u64 capabilities; };
struct cc_subdev_route {
    __u32 sink_pad, sink_stream, source_pad, source_stream, flags, reserved[5];
};
struct cc_subdev_routing {
    __u32 which, len_routes;
    __u64 routes;
    __u32 num_routes, reserved[11];
};
struct cc_subdev_routing_legacy {
    __u32 which, num_routes;
    __u64 routes;
    __u32 reserved[6];
};
#define CC_SD_CAP_STREAMS 2U
#define CC_SD_CAP_READONLY 1U
#define CC_SD_ROUTE_ACTIVE 1U
#define CC_SD_QUERYCAP _IOR('V', 0, struct cc_subdev_capability)
#define CC_SD_S_CLIENT_CAP _IOWR('V', 102, struct cc_subdev_client_capability)
#define CC_SD_S_ROUTING _IOWR('V', 39, struct cc_subdev_routing)
#define CC_SD_S_ROUTING_LEGACY _IOWR('V', 39, struct cc_subdev_routing_legacy)
#define CC_SD_G_ROUTING _IOWR('V', 38, struct cc_subdev_routing)
#define CC_SD_G_ROUTING_LEGACY _IOWR('V', 38, struct cc_subdev_routing_legacy)
_Static_assert(sizeof(struct cc_subdev_format) == 88, "Linux subdevice stream format ABI");
_Static_assert(sizeof(struct cc_subdev_mbus_code_enum) == 48, "Linux subdevice stream code ABI");
_Static_assert(sizeof(struct cc_subdev_frame_size_enum) == 64, "Linux subdevice stream size ABI");
_Static_assert(sizeof(struct cc_subdev_route) == 40, "Linux subdevice route ABI");
_Static_assert(sizeof(struct cc_subdev_routing) == 64, "Linux subdevice routing ABI");
_Static_assert(sizeof(struct cc_subdev_routing_legacy) == 40, "Linux legacy routing ABI");
#endif
