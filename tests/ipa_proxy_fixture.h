/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef CCAMERA_IPA_PROXY_FIXTURE_H
#define CCAMERA_IPA_PROXY_FIXTURE_H
#include <stdint.h>
#define PROXY_PAYLOAD_SIZE 131072U
struct proxy_fixture_init { uint32_t mode; int32_t parent_pid; uint64_t user_namespace, net_namespace; };
#endif
