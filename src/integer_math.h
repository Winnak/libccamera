/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_INTEGER_MATH_H
#define CCAMERA_INTEGER_MATH_H
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
/* Portable 64x64 product divided by 64, without compiler integer extensions. */
static inline int cc_mul_div_u64(uint64_t a, uint64_t b, uint64_t denominator, bool ceil, uint64_t *out)
{
    if (!denominator) return -EINVAL;
    uint64_t a0=(uint32_t)a, a1=a>>32, b0=(uint32_t)b, b1=b>>32;
    uint64_t p=a0*b0, t=a1*b0+(p>>32), u=a0*b1+(uint32_t)t;
    uint64_t hi=a1*b1+(t>>32)+(u>>32), lo=(u<<32)|(uint32_t)p;
    if (hi>=denominator) return -ERANGE;
    uint64_t quotient=0, remainder=hi;
    for (int i=63;i>=0;--i) {
        bool carry=(remainder>>63)!=0;
        remainder=(remainder<<1)|((lo>>i)&1U);
        if (carry || remainder>=denominator) { remainder-=denominator; quotient|=UINT64_C(1)<<i; }
    }
    if (ceil && remainder) { if (quotient==UINT64_MAX) return -ERANGE; ++quotient; }
    *out=quotient; return 0;
}
#endif
