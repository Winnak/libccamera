/* SPDX-License-Identifier: LGPL-2.1-or-later */
#define _GNU_SOURCE
#include "ipa_message.h"
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
#define OK(x) CHECK((x) == 0)
#define ERROR(x,e) CHECK((x) == -(e))
static size_t descriptors(void)
{
    DIR *directory = opendir("/proc/self/fd"); CHECK(directory); size_t count = 0; struct dirent *entry;
    while ((entry = readdir(directory))) if (strcmp(entry->d_name, ".") && strcmp(entry->d_name, "..")) ++count;
    OK(closedir(directory)); return count;
}
static void pair(cc_ipc_pipe **a, cc_ipc_pipe **b)
{
    cc_ipc *first = NULL, *second = NULL; int remote; OK(cc_ipc_create(&first, &remote));
    OK(cc_ipc_bind(remote, &second)); OK(close(remote));
    OK(cc_ipc_pipe_open(&first, -1, a)); OK(cc_ipc_pipe_open(&second, -1, b));
}
static void roundtrip(size_t size, int zero)
{
    cc_ipc_pipe *a = NULL, *b = NULL; pair(&a, &b); cc_ipc_message_header header = {123, 456};
    unsigned char *input = malloc(size ? size : 1); CHECK(input);
    for (size_t i = 0; i < size; ++i) input[i] = (unsigned char)(i * 17U + 3U);
    int saved = -1;
    if (zero) { saved = dup(0); (void)close(0); }
    OK(cc_ipa_message_send(a, &header, input, size));
    memset(input, 0, size); free(input); cc_ipc_pipe_close(a);
    cc_ipc_packet *packet = NULL; OK(cc_ipc_pipe_receive(b, &packet, 2000)); cc_ipc_pipe_close(b);
    cc_ipc_message_header received; OK(cc_ipc_packet_header(packet, &received)); CHECK(received.command == 123 && received.cookie == 456);
    uint32_t count; const int *fds = cc_ipc_packet_fds(packet, &count);
    CHECK(count == (size > CC_IPC_MAX_DATA - 20U ? 1U : 0U));
    if (count) {
        CHECK(fds && (fcntl(fds[0], F_GETFD) & FD_CLOEXEC));
        if (zero) CHECK(fds[0] == 0);
        CHECK(ftruncate(fds[0], 0) < 0 && errno == EPERM);
        CHECK(pwrite(fds[0], "x", 1, 0) < 0 && errno == EPERM);
    }
    void *data = NULL; size_t length = 0; OK(cc_ipa_message_decode(packet, &data, &length)); CHECK(length == size);
    for (size_t i = 0; i < size; ++i) CHECK(((unsigned char *)data)[i] == (unsigned char)(i * 17U + 3U));
    if (!size) CHECK(!data);
    cc_ipc_packet_destroy(packet); free(data);
    if (saved >= 0) { CHECK(dup2(saved, 0) == 0); OK(close(saved)); }
}
static void reject(const void *bytes, size_t size, const int *fds, uint32_t num_fds)
{
    cc_ipc_pipe *a = NULL, *b = NULL; pair(&a, &b); cc_ipc_message_header header = {1, 2};
    cc_ipc_payload payload = {bytes, size, fds, num_fds}; OK(cc_ipc_pipe_send(a, &header, &payload));
    cc_ipc_packet *packet = NULL; OK(cc_ipc_pipe_receive(b, &packet, 2000));
    void *data = (void *)(uintptr_t)1; size_t length = 123;
    ERROR(cc_ipa_message_decode(packet, &data, &length), EPROTO); CHECK(!data && !length);
    cc_ipc_packet_destroy(packet); cc_ipc_pipe_close(a); cc_ipc_pipe_close(b);
}
static void malformed(void)
{
    uint32_t envelope[4] = {1, 0, 0, 0};
    for (size_t i = 0; i < 12; ++i) reject(envelope, i, NULL, 0);
    envelope[0] = 2; reject(envelope, 12, NULL, 0); envelope[0] = 1;
    envelope[1] = 2; reject(envelope, 12, NULL, 0); envelope[1] = 0;
    envelope[2] = CC_IPA_WIRE_MAX_DATA + 1U; reject(envelope, 12, NULL, 0);
    envelope[2] = 1; reject(envelope, 12, NULL, 0);
    envelope[2] = 0; reject(envelope, 16, NULL, 0);
    int fd = open("/dev/null", O_RDONLY | O_CLOEXEC); CHECK(fd >= 0);
    reject(envelope, 12, &fd, 1); envelope[1] = 1; reject(envelope, 12, &fd, 1); OK(close(fd));
    reject(envelope, 12, NULL, 0);
    fd = memfd_create("ccamera-test", MFD_CLOEXEC | MFD_ALLOW_SEALING); CHECK(fd >= 0);
    OK(ftruncate(fd, 1)); envelope[2] = 1;
    reject(envelope, 12, &fd, 1); OK(fcntl(fd, F_ADD_SEALS, F_SEAL_SHRINK | F_SEAL_GROW));
    reject(envelope, 12, &fd, 1); OK(fcntl(fd, F_ADD_SEALS, F_SEAL_WRITE | F_SEAL_SEAL));
    envelope[2] = 2; reject(envelope, 12, &fd, 1); envelope[2] = 1;
    reject(envelope, 16, &fd, 1); int both[] = {fd, fd}; reject(envelope, 12, both, 2); OK(close(fd));
}
static void sidecar(size_t size, uint32_t num_handles)
{
    cc_ipc_pipe *a = NULL, *b = NULL; pair(&a, &b); int pipes[2]; OK(pipe2(pipes, O_CLOEXEC | O_NONBLOCK));
    unsigned char *input = malloc(size); CHECK(input); memset(input, 0x5a, size);
    int handles[CC_IPA_WIRE_MAX_FDS]; for (size_t i = 0; i < num_handles; ++i) handles[i] = pipes[1];
    cc_ipc_message_header header = {99, 100}; OK(cc_ipa_message_send_fds(a, &header, input, size, handles, num_handles));
    free(input); OK(close(pipes[1])); cc_ipc_pipe_close(a);
    cc_ipc_packet *packet = NULL; OK(cc_ipc_pipe_receive(b, &packet, 2000)); cc_ipc_pipe_close(b);
    void *data = NULL; size_t length = 0; ERROR(cc_ipa_message_decode(packet, &data, &length), EPROTO); CHECK(!data && !length);
    const int *fds = NULL; uint32_t count = 0; OK(cc_ipa_message_decode_fds(packet, &data, &length, &fds, &count));
    CHECK(length == size && count == num_handles && fds);
    for (size_t i = 0; i < size; ++i) CHECK(((unsigned char *)data)[i] == 0x5a);
    for (size_t i = 0; i < count; ++i) CHECK(fcntl(fds[i], F_GETFD) & FD_CLOEXEC);
    char byte; CHECK(read(pipes[0], &byte, 1) < 0 && errno == EAGAIN);
    cc_ipc_packet_destroy(packet); CHECK(read(pipes[0], &byte, 1) == 0); OK(close(pipes[0])); free(data);
}
int main(void)
{
    size_t before = descriptors();
    roundtrip(0, 0); roundtrip(1, 0); roundtrip(CC_IPC_MAX_DATA - 20U, 0);
    roundtrip(CC_IPC_MAX_DATA - 19U, 0); roundtrip(131072, 1); roundtrip(CC_IPA_WIRE_MAX_DATA, 0);
    sidecar(1, 1); sidecar(131072, 1); sidecar(131072, CC_IPA_WIRE_MAX_FDS);
    malformed(); void *data = (void *)(uintptr_t)1; size_t size = 123;
    ERROR(cc_ipa_message_decode(NULL, &data, &size), EINVAL); CHECK(!data && !size);
    cc_ipc_pipe *a = NULL, *b = NULL; pair(&a, &b); cc_ipc_message_header header = {0}; char byte = 0;
    ERROR(cc_ipa_message_send(a, &header, NULL, 1), EINVAL);
    ERROR(cc_ipa_message_send(a, &header, &byte, CC_IPA_WIRE_MAX_DATA + 1U), E2BIG);
    cc_ipc_pipe_close(a); cc_ipc_pipe_close(b); CHECK(descriptors() == before);
    puts("IPA inline and sealed-memfd snapshots, bounds, fd-zero and immutable data checks passed"); return 0;
}
