/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Image-relative lookup follows libcamera's source_paths.cpp, adapted to
 * CMake's marker and static-library layout. Reference: Copyright (C) 2021 Google Inc. */
#define _GNU_SOURCE
#include "source_paths.h"
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static const char location_anchor = 0;
static int mapped_image(char **out)
{
    int fd = open("/proc/self/maps", O_RDONLY | O_CLOEXEC);
    if (fd < 0) return errno == ENOMEM || errno == EMFILE || errno == ENFILE ? -errno : 0;
    FILE *file = fdopen(fd, "r");
    if (!file) { int error = errno; close(fd); return -error; }
    char *line = NULL; size_t capacity = 0; int ret = 0;
    errno = 0;
    while (getline(&line, &capacity, file) >= 0) {
        unsigned long long start, end;
        if (sscanf(line, "%llx-%llx", &start, &end) != 2 ||
            (uintptr_t)&location_anchor < start || (uintptr_t)&location_anchor >= end) continue;
        char link[128], filename[4097];
        int length = snprintf(link, sizeof(link), "/proc/self/map_files/%llx-%llx", start, end);
        if (length <= 0 || (size_t)length >= sizeof(link)) break;
        ssize_t size = readlink(link, filename, sizeof(filename) - 1);
        if (size <= 0 || (size_t)size >= sizeof(filename) - 1) break;
        filename[size] = 0;
        *out = realpath(filename, NULL);
        if (!*out && errno == ENOMEM) ret = -ENOMEM;
        break;
    }
    if (!*out && errno == ENOMEM) ret = -ENOMEM;
    free(line); fclose(file);
    return ret;
}
int cc_build_root(char **out)
{
    if (!out) return -EINVAL;
    *out = NULL;
    char *image = NULL;
    int ret = mapped_image(&image);
    if (ret < 0) return ret;
    if (!image) {
        Dl_info info;
        if (!dladdr(&location_anchor, &info) || !info.dli_fname || info.dli_fname[0] != '/') return 0;
        /* Do not resolve relative loader names against a changed working
         * directory when procfs is unavailable. */
        image = realpath(info.dli_fname, NULL);
        if (!image) return errno == ENOMEM ? -ENOMEM : 0;
    }
    char *slash = strrchr(image, '/');
    if (!slash) { free(image); return 0; }
    /* Keep the root slash when the image resides directly under /. */
    if (slash == image) slash[1] = 0;
    else *slash = 0;
    size_t length = strlen(image);
    static const char suffix[] = "/.ccamera-build-tree";
    if (length > SIZE_MAX - sizeof(suffix)) { free(image); return -EOVERFLOW; }
    char *marker = malloc(length + sizeof(suffix));
    if (!marker) { free(image); return -ENOMEM; }
    memcpy(marker, image, length); memcpy(marker + length, suffix, sizeof(suffix));
    int fd = open(marker, O_RDONLY | O_CLOEXEC | O_NONBLOCK);
    free(marker);
    if (fd < 0) {
        int error = errno; free(image);
        return error == ENOMEM || error == EMFILE || error == ENFILE ? -error : 0;
    }
    struct stat status;
    if (fstat(fd, &status) || !S_ISREG(status.st_mode)) { close(fd); free(image); return 0; }
    char content[sizeof(CC_BUILD_TREE_MAGIC)]; size_t count = 0;
    bool failed = false;
    while (count < sizeof(content)) {
        ssize_t n = read(fd, content + count, sizeof(content) - count);
        if (n < 0) { if (errno == EINTR) continue; failed = true; break; }
        if (!n) break;
        count += (size_t)n;
    }
    close(fd);
    /* The generated marker identifies native CMake builds, including static
     * consumers which need no RPATH. It is not installed with the library. */
    if (failed || count != sizeof(CC_BUILD_TREE_MAGIC) - 1 ||
        memcmp(content, CC_BUILD_TREE_MAGIC, sizeof(CC_BUILD_TREE_MAGIC) - 1)) { free(image); return 0; }
    *out = image;
    return 0;
}
