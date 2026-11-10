/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_MONITOR_H
#define CCAMERA_MONITOR_H
#include <ccamera/ccamera.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct cc_camera_monitor cc_camera_monitor;
enum cc_camera_event_type {
    CC_CAMERA_ADDED=1, CC_CAMERA_REMOVED=2, CC_CAMERA_CHANGED=3, CC_CAMERA_RESYNC=4
};
enum cc_camera_event_flags {
    CC_CAMERA_EVENTS_LOST=1U, CC_CAMERA_EVENT_QUEUE_OVERFLOW=2U
};
typedef struct cc_camera_event {
    enum cc_camera_event_type type;
    uint32_t flags;
    uint64_t generation;
    cc_camera_info camera;   /* Added/current/removed value; zero for RESYNC. */
    cc_camera_info previous; /* Previous value for CHANGED; otherwise zero. */
} cc_camera_event;

/* Caller-driven hotplug registry using kernel uevents, inotify and retry timers.
 * Subscribes before initial enumeration and queues initial ADDED events.
 * No camera ownership, capture threads, callbacks or libudev dependency.
 * Calls on one monitor must be serialized. *monitor is NULL on failure. */
CC_API int cc_camera_monitor_open(cc_camera_monitor **monitor);
CC_API void cc_camera_monitor_close(cc_camera_monitor *monitor);
/* Borrowed CLOEXEC epoll fd, readable for notifications, queued
 * events or deferred discovery retries. Poll it; never read or close it. */
CC_API int cc_camera_monitor_fd(const cc_camera_monitor *monitor);
/* Drain bounded notification batches and reconcile. Does not wait. A failed
 * refresh retains the published registry/events and can be retried. */
CC_API int cc_camera_monitor_dispatch(cc_camera_monitor *monitor);
/* Explicitly reconcile even without a notification, e.g. after changing access
 * permissions or native graph configuration. Same publication guarantees. */
CC_API int cc_camera_monitor_refresh(cc_camera_monitor *monitor);
/* Copy the published registry; free with cc_camera_list_free. Does not dispatch.
 * Outputs stay unchanged on error. Generation changes with published events. */
CC_API int cc_camera_monitor_snapshot(const cc_camera_monitor *monitor, cc_camera_info **cameras,
                                      size_t *count, uint64_t *generation);
/* Dequeue a copied event. timeout_ms: 0 => -EAGAIN, -1 => indefinite, >0 =>
 * bounded wait (-ETIMEDOUT). Outputs stay unchanged on error. A RESYNC event
 * indicates lost notifications/queued events; copy a fresh snapshot. Removals
 * precede additions within a generation, including same-ID reconnections.
 * Existing camera/capture handles remain caller-owned and are never closed. */
CC_API int cc_camera_monitor_dequeue(cc_camera_monitor *monitor, cc_camera_event *event, int timeout_ms);

#ifdef __cplusplus
}
#endif
#endif
