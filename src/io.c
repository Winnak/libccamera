/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "internal.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <unistd.h>

void cc_copy_string(char *dst, size_t capacity, const char *src, size_t length)
{
    if (!capacity)
        return;
    if (length >= capacity)
        length = capacity - 1;
    memcpy(dst, src, length);
    dst[length] = '\0';
}

static int system_open(void *ctx, const char *path, int flags)
{
    (void)ctx;
    int fd;
    do { fd = open(path, flags); } while (fd < 0 && errno == EINTR);
    return fd < 0 ? -errno : fd;
}

static int system_close(void *ctx, int fd)
{
    (void)ctx;
    /* On Linux close consumes the fd even when interrupted. Never retry. */
    return close(fd) < 0 ? -errno : 0;
}

static int system_ioctl(void *ctx, int fd, unsigned long req, void *arg)
{
    (void)ctx;
    int ret;
    do { ret = ioctl(fd, req, arg); } while (ret < 0 && errno == EINTR);
    return ret < 0 ? -errno : ret;
}

static int system_map(void *ctx, int fd, size_t size, off_t offset, void **out)
{
    (void)ctx;
    void *p = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);
    if (p == MAP_FAILED)
        return -errno;
    *out = p;
    return 0;
}

static int system_unmap(void *ctx, void *p, size_t size)
{
    (void)ctx;
    return munmap(p, size) < 0 ? -errno : 0;
}

static int system_lock(void *ctx, int fd)
{
    (void)ctx;
    int ret;
    do { ret = flock(fd, LOCK_EX | LOCK_NB); } while (ret < 0 && errno == EINTR);
    return ret < 0 ? -errno : 0;
}

static int system_wait_many(void *ctx, struct pollfd *fds, size_t count, int timeout)
{
    (void)ctx;
    int ret = poll(fds, (nfds_t)count, timeout);
    if (ret < 0)
        return -errno;
    if (!ret)
        return -ETIMEDOUT;
    bool ready = false;
    for (size_t i = 0; i < count; ++i) {
        if (fds[i].revents & POLLNVAL) return -EBADF;
        if (fds[i].revents & POLLHUP) return -ENODEV;
        if (fds[i].revents & fds[i].events) ready = true;
        else if (fds[i].revents & POLLERR) return -EIO;
    }
    if (ready) return 0;
    return -EAGAIN;
}

static int system_wait(void *ctx, int fd, short events, int timeout)
{
    struct pollfd p = { .fd = fd, .events = events };
    return system_wait_many(ctx, &p, 1, timeout);
}

static int system_duplicate(void *ctx, int fd)
{
    (void)ctx;
    int ret;
    do { ret = fcntl(fd, F_DUPFD_CLOEXEC, 0); } while (ret < 0 && errno == EINTR);
    return ret < 0 ? -errno : ret;
}

static int system_identity(void *ctx, int fd, uint64_t *device, uint64_t *inode)
{
    (void)ctx;
    struct stat st;
    int ret;
    do { ret = fstat(fd, &st); } while (ret < 0 && errno == EINTR);
    if (ret < 0) return -errno;
    *device = (uint64_t)st.st_dev; *inode = (uint64_t)st.st_ino;
    return 0;
}

static int system_now(void *ctx, uint64_t *ns)
{
    (void)ctx;
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) < 0)
        return -errno;
    *ns = (uint64_t)ts.tv_sec * UINT64_C(1000000000) + (uint64_t)ts.tv_nsec;
    return 0;
}

static int system_resolve(void *ctx, unsigned int major_id, unsigned int minor_id,
                          char *path, size_t capacity)
{
    (void)ctx;
    char sysfs[128], line[512];
    snprintf(sysfs, sizeof(sysfs), "/sys/dev/char/%u:%u/uevent", major_id, minor_id);
    int fd = system_open(NULL, sysfs, O_RDONLY | O_CLOEXEC);
    if (fd < 0)
        return fd;
    FILE *f = fdopen(fd, "r");
    if (!f) {
        int saved = errno;
        close(fd);
        return -saved;
    }
    int ret = -ENOENT;
    while (fgets(line, sizeof(line), f)) {
        if (strncmp(line, "DEVNAME=", 8))
            continue;
        char *name = line + 8;
        name[strcspn(name, "\r\n")] = '\0';
        if (!name[0] || name[0] == '/' || strstr(name, "..")) {
            ret = -EINVAL;
            break;
        }
        int n = snprintf(path, capacity, "/dev/%s", name);
        if (n < 0 || (size_t)n >= capacity) {
            ret = -ENAMETOOLONG;
            break;
        }
        struct stat st;
        if (stat(path, &st) < 0) {
            ret = -errno;
            break;
        }
        ret = S_ISCHR(st.st_mode) && major(st.st_rdev) == major_id &&
              minor(st.st_rdev) == minor_id ? 0 : -ENODEV;
        break;
    }
    fclose(f);
    return ret;
}

static const struct cc_io_ops system_ops = {
    .open = system_open, .close = system_close, .ioctl = system_ioctl,
    .map = system_map, .unmap = system_unmap, .lock = system_lock,
    .wait = system_wait, .now = system_now, .resolve = system_resolve,
    .duplicate = system_duplicate, .wait_many = system_wait_many, .identity = system_identity
};
const struct cc_io cc_system_io = { .ops = &system_ops, .ctx = NULL };
