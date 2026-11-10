/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_MONITOR_INTERNAL_H
#define CCAMERA_MONITOR_INTERNAL_H
#include "internal.h"
#include <ccamera/monitor.h>

enum cc_notice_type { CC_NOTICE_CHANGE=1, CC_NOTICE_REMOVE=2, CC_NOTICE_LOST=3, CC_NOTICE_TIMER=4 };
struct cc_monitor_notice { enum cc_notice_type type; char path[CC_PATH_MAX]; };
struct cc_node_token { uint64_t device,inode; };
struct cc_monitor_backend_ops {
    int (*next)(void *,struct cc_monitor_notice *); /* 1 notice, 0 drained, negative error. */
    int (*ready)(void *,bool);
    int (*retry)(void *,uint32_t); /* Milliseconds; zero disarms. */
    int (*probe)(void *,const char *,struct cc_node_token *);
    int (*wait)(void *,int);
    int (*now)(void *,uint64_t *);
    void (*close)(void *);
};
struct cc_monitor_backend { const struct cc_monitor_backend_ops *ops; void *ctx; int fd; };
int cc_monitor_native_open(struct cc_monitor_backend *backend);
/* Transfers backend ownership on success only. */
int cc_camera_monitor_create_io(const struct cc_io *io,struct cc_monitor_backend *backend,cc_camera_monitor **out);
int cc_monitor_parse_uevent(const void *data,size_t size,struct cc_monitor_notice *notice);
int cc_monitor_parse_inotify(const void *data,size_t size,int watch,size_t *offset,struct cc_monitor_notice *notice);
int cc_enumerate_state_io(const struct cc_io *io,cc_camera_info **cameras,size_t *count,bool *pending);
#endif
