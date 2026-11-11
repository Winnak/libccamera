/* SPDX-License-Identifier: CC0-1.0 */
#ifndef CCAMERA_TEST_NAMESPACE_H
#define CCAMERA_TEST_NAMESPACE_H
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* The proc symlink contains the nsfs inode as "type:[inode]". Reading it
 * preserves the identity check without stat() following the magic link to a
 * disconnected nsfs object, which AppArmor can deny after userns creation. */
static int test_namespace_inode(const char *path, uint64_t *inode)
{
    char target[64];
    ssize_t size = readlink(path, target, sizeof(target));
    if (size < 0) return -errno;
    if ((size_t)size >= sizeof(target)) return -ENAMETOOLONG;
    target[size] = 0;
    const char *start = strchr(target, '[');
    if (!start || start == target || start[-1] != ':' || start[1] < '0' || start[1] > '9') return -EPROTO;
    errno = 0; char *end;
    unsigned long long value = strtoull(start + 1, &end, 10);
    if (errno || !value || end[0] != ']' || end[1]) return -EPROTO;
    *inode = (uint64_t)value; return 0;
}
#endif
