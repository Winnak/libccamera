/* SPDX-License-Identifier: LGPL-2.1-or-later */
#define _GNU_SOURCE
#include <ccamera/ipa_vimc.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
#define OK(x) CHECK((x) == 0)
#define ERROR(x,e) do { int actual = (x); if (actual != -(e)) fprintf(stderr, "%s: expected %d, got %d\n", #x, -(e), actual); CHECK(actual == -(e)); } while (0)
static size_t descriptors(void)
{
    DIR *directory = opendir("/proc/self/fd"); CHECK(directory); size_t count = 0; struct dirent *entry;
    while ((entry = readdir(directory))) if (strcmp(entry->d_name, ".") && strcmp(entry->d_name, "..")) ++count;
    OK(closedir(directory)); return count;
}
static void operation(int fd, uint32_t expected)
{
    uint32_t value = UINT32_MAX; CHECK(read(fd, &value, sizeof(value)) == sizeof(value) && value == expected);
}
static void computed(cc_ipa *ipa, uint32_t id)
{
    OK(cc_ipa_vimc_compute_params(ipa, 7, id)); cc_ipa_vimc_event event;
    OK(cc_ipa_vimc_dequeue_event(ipa, &event, 2000)); CHECK(event.buffer_id == id && !event.flags);
}
static void buffers(cc_ipa *ipa)
{
    int fd = memfd_create("ccamera-vimc-test", MFD_CLOEXEC); CHECK(fd >= 0); OK(ftruncate(fd, 8192));
    cc_ipa_vimc_buffer buffer = {.id = 71, .num_planes = 2, .planes = {{fd, 37, 1024}, {fd, 4097, 1024}}};
    OK(cc_ipa_vimc_map_buffers(ipa, &buffer, 1)); CHECK(fcntl(fd, F_GETFD) & FD_CLOEXEC);
    ERROR(cc_ipa_vimc_map_buffers(ipa, &buffer, 1), EEXIST); OK(close(fd)); computed(ipa, 71);
    uint32_t unknown = 999; OK(cc_ipa_vimc_unmap_buffers(ipa, &unknown, 1)); computed(ipa, 71);
    cc_ipa_vimc_event event = {999, 999}, original = event;
    OK(cc_ipa_vimc_compute_params(ipa, 8, unknown)); ERROR(cc_ipa_vimc_dequeue_event(ipa, &event, 2), ETIMEDOUT);
    CHECK(!memcmp(&event, &original, sizeof(event))); computed(ipa, 71);
    OK(cc_ipa_vimc_compute_params(ipa, 9, 71));
    uint32_t id = 71; OK(cc_ipa_vimc_unmap_buffers(ipa, &id, 1));
    OK(cc_ipa_vimc_dequeue_event(ipa, &event, 0)); CHECK(event.buffer_id == 71 && !event.flags);
    OK(cc_ipa_vimc_compute_params(ipa, 8, id)); ERROR(cc_ipa_vimc_dequeue_event(ipa, &event, 2), ETIMEDOUT);
    fd = memfd_create("ccamera-vimc-test", MFD_CLOEXEC); CHECK(fd >= 0); OK(ftruncate(fd, 8192));
    cc_ipa_vimc_buffer pair[2] = {{.id = 100, .num_planes = 1, .planes = {{fd, 0, 128}}},
        {.id = 101, .num_planes = 1, .planes = {{fd, 8190, 128}}}};
    ERROR(cc_ipa_vimc_map_buffers(ipa, pair, 2), EINVAL); pair[1].planes[0].offset = 4096;
    OK(cc_ipa_vimc_map_buffers(ipa, pair, 2)); /* First failed map rolled back fully. */
    computed(ipa, 100); computed(ipa, 101); uint32_t ids[] = {100, 101}; OK(cc_ipa_vimc_unmap_buffers(ipa, ids, 2));
    cc_ipa_vimc_buffer all[CC_IPA_VIMC_MAX_BUFFERS]; uint32_t all_ids[CC_IPA_VIMC_MAX_BUFFERS];
    for (uint32_t i = 0; i < CC_IPA_VIMC_MAX_BUFFERS; ++i) {
        all_ids[i] = i + 1U; all[i] = (cc_ipa_vimc_buffer){.id = i + 1U, .num_planes = 1, .planes = {{fd, 0, 128}}};
    }
    OK(cc_ipa_vimc_map_buffers(ipa, all, CC_IPA_VIMC_MAX_BUFFERS)); ERROR(cc_ipa_vimc_map_buffers(ipa, pair, 1), ENOSPC);
    for (uint32_t i = 0; i < CC_IPA_VIMC_MAX_BUFFERS; ++i) computed(ipa, i + 1U);
    OK(cc_ipa_vimc_unmap_buffers(ipa, all_ids, CC_IPA_VIMC_MAX_BUFFERS));
    for (unsigned i = 0; i < 50; ++i) { OK(cc_ipa_vimc_map_buffers(ipa, pair, 2)); OK(cc_ipa_vimc_unmap_buffers(ipa, ids, 2)); }
    OK(close(fd));
    int saved = dup(0); (void)close(0); fd = memfd_create("ccamera-vimc-zero", MFD_CLOEXEC); CHECK(fd == 0); OK(ftruncate(fd, 8192));
    buffer.planes[0].fd = buffer.planes[1].fd = 0; OK(cc_ipa_vimc_map_buffers(ipa, &buffer, 1)); OK(close(fd));
    computed(ipa, 71); id = 71; OK(cc_ipa_vimc_unmap_buffers(ipa, &id, 1));
    if (saved >= 0) { CHECK(dup2(saved, 0) == 0); OK(close(saved)); }
}
int main(int argc, char **argv)
{
    CHECK(argc == 2); size_t before = descriptors(); int pipes[2]; OK(pipe2(pipes, O_NONBLOCK | O_CLOEXEC));
    const char configuration[] = "vimc mock configuration";
    cc_ipa_vimc_options controller = {configuration, sizeof(configuration), pipes[1], CC_IPA_VIMC_INIT, CC_IPA_VIMC_FLAG2};
    cc_ipa_proxy_options options = {.worker_path = argv[1]}; cc_ipa *ipa = NULL; uint32_t flags = 999;
    int ret = cc_ipa_vimc_create_proxy(&controller, &options, &ipa, &flags);
    if (ret == -ENOSYS || ret == -EPERM || ret == -EACCES) { close(pipes[0]); close(pipes[1]); return 77; }
    if (ret) fprintf(stderr, "VIMC create error: %d\n", ret);
    OK(ret); CHECK(flags == CC_IPA_VIMC_FLAG1 && !cc_ipa_get_capabilities(ipa));
    CHECK(fcntl(pipes[1], F_GETFD) & FD_CLOEXEC); OK(close(pipes[1])); operation(pipes[0], CC_IPA_VIMC_INIT);
    buffers(ipa);
    cc_control controls[] = {{.id = 1, .value = 2}, {.id = 3, .value = 4}};
    OK(cc_ipa_vimc_queue_request(ipa, UINT32_MAX, controls, 2)); OK(cc_ipa_vimc_dispatch(ipa));
    cc_ipa_configuration config = {.pipeline = CC_IPA_VIMC}; OK(cc_ipa_configure(ipa, &config));
    for (unsigned i = 0; i < 3; ++i) {
        OK(cc_ipa_start(ipa)); operation(pipes[0], CC_IPA_VIMC_START);
        cc_ipa_request request = {.sequence = i, .cookie = 123}; cc_ipa_settings settings;
        OK(cc_ipa_prepare(ipa, &request, &settings)); CHECK(!settings.flags && !settings.num_isp_controls && !settings.num_sensor_controls);
        cc_ipa_frame frame = {.sequence = i, .cookie = 123}; cc_ipa_metadata metadata;
        OK(cc_ipa_process(ipa, &frame, &metadata)); CHECK(!metadata.flags);
        cc_ipa_stop(ipa); operation(pipes[0], CC_IPA_VIMC_STOP);
    }
    cc_ipa_destroy(ipa); char byte; CHECK(read(pipes[0], &byte, 1) == 0); OK(close(pipes[0]));
    /* A closed trace reader is a best-effort trace failure, not a dead worker. */
    OK(pipe2(pipes, O_NONBLOCK | O_CLOEXEC)); OK(close(pipes[0])); controller.trace_fd = pipes[1];
    OK(cc_ipa_vimc_create_proxy(&controller, &options, &ipa, &flags)); OK(close(pipes[1]));
    OK(cc_ipa_configure(ipa, &config)); OK(cc_ipa_start(ipa)); cc_ipa_stop(ipa); cc_ipa_destroy(ipa);
    flags = 999; controller.trace_fd = -1; controller.configuration_size = 0;
    ERROR(cc_ipa_vimc_create_proxy(&controller, &options, &ipa, &flags), EINVAL); CHECK(!ipa && flags == 999);
    ERROR(cc_ipa_vimc_dispatch(NULL), EINVAL); ERROR(cc_ipa_vimc_map_buffers(NULL, NULL, 0), EINVAL);
    CHECK(descriptors() == before); int status; CHECK(waitpid(-1, &status, WNOHANG) < 0 && errno == ECHILD);
    puts("VIMC mock lifecycle traces, buffer maps, async queue/params notifications, restart and cleanup passed"); return 0;
}
