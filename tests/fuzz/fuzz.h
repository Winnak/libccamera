/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_FUZZ_H
#define CCAMERA_FUZZ_H
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Assertions must remain active in Release/RelWithDebInfo builds. */
#define FUZZ_CHECK(x) do { if (!(x)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); abort(); \
} } while (0)

struct fuzz_input { const uint8_t *data; size_t size; };
static inline uint8_t fuzz_u8(struct fuzz_input *in)
{
    if (!in->size) return 0;
    --in->size;
    return *in->data++;
}
static inline uint16_t fuzz_u16(struct fuzz_input *in)
{
    uint16_t lo = fuzz_u8(in), hi = fuzz_u8(in);
    return (uint16_t)(lo | (uint16_t)(hi << 8));
}
static inline uint32_t fuzz_u32(struct fuzz_input *in)
{
    uint32_t lo = fuzz_u16(in), hi = fuzz_u16(in);
    return lo | (hi << 16);
}
static inline uint64_t fuzz_u64(struct fuzz_input *in)
{
    uint64_t lo = fuzz_u32(in), hi = fuzz_u32(in);
    return lo | (hi << 32);
}
static inline void fuzz_bytes(struct fuzz_input *in, void *out, size_t size)
{
    size_t n = in->size < size ? in->size : size;
    if (n) memcpy(out, in->data, n);
    memset((uint8_t *)out + n, 0, size - n);
    in->data += n; in->size -= n;
}
/* Mutate a bounded slice of an otherwise useful configuration. No pointers or
 * bools are ever patched, only integer-only public/UAPI structures. */
static inline void fuzz_patch(struct fuzz_input *in, void *out, size_t size)
{
    size_t offset = fuzz_u32(in) % size;
    size_t n = (size_t)fuzz_u8(in) + 1;
    if (n > size - offset) n = size - offset;
    fuzz_bytes(in, (uint8_t *)out + offset, n);
}
#endif
