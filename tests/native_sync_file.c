/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Optional native validation. Uses a temporary, initially signaled DRM syncobj;
 * submits no GPU work and changes no display state. No libdrm dependency. */
#include "fence.h"
#include <ccamera/capture.h>
#include <linux/sync_file.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

/* The original DRM syncobj UAPI remains compatible with newer kernels whose
 * handle argument adds a timeline point. Keep zero-extended legacy arguments. */
struct native_sync_create { uint32_t handle, flags; };
struct native_sync_destroy { uint32_t handle, pad; };
struct native_sync_export { uint32_t handle, flags; int32_t fd; uint32_t pad; };
#define NATIVE_SYNC_CREATE _IOWR('d', 0xbf, struct native_sync_create)
#define NATIVE_SYNC_DESTROY _IOWR('d', 0xc0, struct native_sync_destroy)
#define NATIVE_SYNC_EXPORT _IOWR('d', 0xc1, struct native_sync_export)
_Static_assert(CC_SYNC_FILE_INFO == SYNC_IOC_FILE_INFO, "native sync ioctl");
_Static_assert(sizeof(struct cc_sync_file_info) == sizeof(struct sync_file_info), "native sync layout");
_Static_assert(offsetof(struct cc_sync_file_info, status) == offsetof(struct sync_file_info, status), "native status field");

#define CHECK(x) do { if (!(x)) { \
    fprintf(stderr, "%s:%d: %s (errno=%d)\n", __FILE__, __LINE__, #x, errno); \
    result = 1; goto cleanup; \
} } while (0)
int main(int argc, char **argv)
{
    const char *path = argc > 1 ? argv[1] : "/dev/dri/renderD128";
    int device = open(path, O_RDWR | O_CLOEXEC), original = -1, taken = -1, result = 0;
    struct native_sync_create create = {.flags=1}; /* DRM_SYNCOBJ_CREATE_SIGNALED */
    struct cc_fence fence = {0}; cc_request *request = NULL;
    if (device < 0) {
        int error = errno;
        printf("Render node %s unavailable (errno=%d)\n", path, error);
        return error == ENOENT || error == ENODEV || error == ENXIO || error == EACCES || error == EPERM ? 77 : 1;
    }
    if (ioctl(device, NATIVE_SYNC_CREATE, &create) < 0) {
        int error = errno;
        printf("Render-node syncobj creation failed (errno=%d)\n", error);
        close(device);
        return error == ENOTTY || error == EINVAL || error == EOPNOTSUPP || error == EACCES || error == EPERM ? 77 : 1;
    }
    CHECK(create.handle);
    struct native_sync_export exported = {.handle=create.handle,.flags=1,.fd=-1}; /* EXPORT_SYNC_FILE */
    CHECK(ioctl(device, NATIVE_SYNC_EXPORT, &exported) == 0 && exported.fd >= 0);
    original = exported.fd;
    struct sync_file_info info = {0};
    CHECK(ioctl(original, SYNC_IOC_FILE_INFO, &info) == 0 && info.status == 1);
    CHECK(cc_fence_set(&fence, original, &cc_system_io) == 0);
    int owned = fence.fd;
    int flags = fcntl(owned, F_GETFD); CHECK(flags >= 0 && (flags & FD_CLOEXEC));
    CHECK(cc_request_create(1, &request) == 0);
    CHECK(cc_request_set_acquire_fence(request, CC_CAPTURE_RAW, original) == 0);
    CHECK(cc_request_set_acquire_fence(request, CC_CAPTURE_RAW, device) == -EINVAL);
    taken = cc_request_take_acquire_fence(request, CC_CAPTURE_RAW);
    CHECK(taken >= 0); flags = fcntl(taken, F_GETFD); CHECK(flags >= 0 && (flags & FD_CLOEXEC));
    CHECK(close(original) == 0); original = -1;
    struct native_sync_destroy destroy = {.handle=create.handle};
    CHECK(ioctl(device, NATIVE_SYNC_DESTROY, &destroy) == 0); create.handle = 0;
    CHECK(close(device) == 0); device = -1;
    struct pollfd ready = {.fd=owned,.events=POLLIN};
    CHECK(poll(&ready, 1, 0) == 1 && (ready.revents & POLLIN));
    CHECK(cc_fence_update(&fence) == 1 && !fence.valid);
    CHECK(fcntl(owned, F_GETFD) == -1 && errno == EBADF);
    info = (struct sync_file_info){0};
    CHECK(ioctl(taken, SYNC_IOC_FILE_INFO, &info) == 0 && info.status == 1);
    puts("Native DRM sync-file layout, validation, CLOEXEC, polling and one-shot ownership passed");
cleanup:
    cc_fence_clear(&fence);
    if (request) (void)cc_request_destroy(request);
    if (original >= 0) close(original);
    if (taken >= 0) close(taken);
    if (device >= 0) {
        if (create.handle) {
            struct native_sync_destroy release = {.handle=create.handle};
            if (ioctl(device, NATIVE_SYNC_DESTROY, &release) < 0) result = 1;
        }
        close(device);
    }
    return result;
}
