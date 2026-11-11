/* SPDX-License-Identifier: LGPL-2.1-or-later */
#define _GNU_SOURCE
#include "ipa_proxy_fixture.h"
#include <ccamera/ipa_proxy.h>
#include <ccamera/capture.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
#define OK(x) CHECK((x) == 0)
#define ERROR(x,e) do { int result_error = (x); if (result_error != -(e)) fprintf(stderr, "%s: expected %d, got %d\n", #x, -(e), result_error); CHECK(result_error == -(e)); } while (0)
static cc_ipa_proxy_options options;
static const char *fixture;
static char trace_path[] = "/tmp/ccamera-proxy-trace-XXXXXX";
static size_t descriptors(void)
{
    DIR *directory = opendir("/proc/self/fd"); CHECK(directory); size_t count = 0; struct dirent *entry;
    while ((entry = readdir(directory))) if (strcmp(entry->d_name, ".") && strcmp(entry->d_name, "..")) ++count;
    OK(closedir(directory)); return count;
}
static struct proxy_fixture_init init(uint32_t mode)
{
    struct stat user, net; OK(stat("/proc/self/ns/user", &user)); OK(stat("/proc/self/ns/net", &net));
    return (struct proxy_fixture_init){mode, getpid(), user.st_ino, net.st_ino};
}
static int create(uint32_t mode, cc_ipa **ipa)
{
    cc_ipa_module *module = NULL; OK(cc_ipa_module_open(fixture, &module));
    struct proxy_fixture_init data = init(mode);
    int ret = cc_ipa_module_create_proxy(module, &data, sizeof(data), &options, ipa);
    cc_ipa_module_close(module); return ret;
}
static int configure(cc_ipa *ipa)
{
    const cc_sensor_model *sensor; OK(cc_sensor_lookup("imx708", &sensor));
    cc_ipa_configuration configuration = {.pipeline = CC_IPA_CFE, .source = {.width = 640}, .sensor = sensor,
        .num_sensor_controls = 1, .sensor_controls = {{.id = 1234}}, .initial_controls = {{.id = 1234, .value = 123}},
        .sensor_delays = {{.id = 1234, .frames = 2}}};
    return cc_ipa_configure(ipa, &configuration);
}
static int prepare(cc_ipa *ipa, cc_ipa_settings *settings)
{
    cc_control control = {.id = 1234, .value = 777}; cc_af_controls af = {.flags = CC_AF_SET_MODE, .mode = CC_AF_CONTINUOUS};
    cc_ipa_request request = {.sequence = UINT64_MAX, .cookie = 456, .sensor_controls = &control, .num_sensor_controls = 1, .af = &af};
    return cc_ipa_prepare(ipa, &request, settings);
}
static int process(cc_ipa *ipa, const cc_ipa_settings *settings, cc_ipa_metadata *metadata)
{
    unsigned char *data = malloc(2U * PROXY_PAYLOAD_SIZE); CHECK(data);
    for (size_t i = 0; i < PROXY_PAYLOAD_SIZE; ++i) { data[i] = (unsigned char)i; data[PROXY_PAYLOAD_SIZE + i] = (unsigned char)(i + 1U); }
    cc_control requested = {.id = 1234, .value = 777}; cc_vc4_tuning vc4;
    cc_sensor_geometry geometry = {.active = {0, 0, 2592, 1944}, .crop = {0, 0, 2592, 1944}, .width = 640, .height = 480};
    OK(cc_vc4_ov5647_build_tuning(&geometry, 4500, 1, &vc4));
    cc_ipa_frame frame = {.sequence = UINT64_MAX, .cookie = 456, .statistics_data = data, .statistics_size = PROXY_PAYLOAD_SIZE,
        .embedded_data = data + PROXY_PAYLOAD_SIZE, .embedded_size = PROXY_PAYLOAD_SIZE,
        .isp_controls = settings->isp_controls, .num_isp_controls = settings->num_isp_controls,
        .backend = &settings->backend, .vc4 = &vc4, .af = settings->af,
        .request_sensor_controls = &requested, .num_request_sensor_controls = 1,
        .settings_sensor_controls = settings->sensor_controls, .num_settings_sensor_controls = settings->num_sensor_controls,
        .sensor = {.count = 1, .controls = {{.id = 1234, .value = 999}}}};
    int ret = cc_ipa_process(ipa, &frame, metadata); free(data); return ret;
}
static void successful(void)
{
    cc_ipa *ipa = NULL; OK(create(0, &ipa)); CHECK(cc_ipa_get_capabilities(ipa) == 7);
    OK(configure(ipa)); OK(cc_ipa_start(ipa));
    cc_ipa_settings settings; OK(prepare(ipa, &settings));
    CHECK(settings.num_isp_controls == 2 && settings.isp_controls[0].size == PROXY_PAYLOAD_SIZE);
    for (size_t i = 0; i < PROXY_PAYLOAD_SIZE; ++i) CHECK(((unsigned char *)settings.isp_controls[0].data)[i] == (unsigned char)(i + 2U));
    struct bcm2835_isp_lens_shading lens; memcpy(&lens, settings.isp_controls[1].data, sizeof(lens));
    char text[19]; CHECK(lens.enabled && lens.dmabuf >= 0 && (fcntl(lens.dmabuf, F_GETFD) & FD_CLOEXEC));
    CHECK(pread(lens.dmabuf, text, sizeof(text), 0) == sizeof(text) && !memcmp(text, "lens shading table", sizeof(text)));
    cc_request *snapshot = NULL; OK(cc_request_create(1, &snapshot)); size_t count = descriptors();
    OK(cc_request_set_isp_controls(snapshot, settings.isp_controls, settings.num_isp_controls)); CHECK(descriptors() == count + 1);
    cc_ipa_metadata metadata; OK(process(ipa, &settings, &metadata));
    CHECK(fcntl(lens.dmabuf, F_GETFD) < 0 && errno == EBADF); OK(cc_request_destroy(snapshot));
    CHECK(metadata.exposure_ns == 1234567 && metadata.red_gain == 1.5 && metadata.af.lens_setting == 500);
    cc_ipa_stop(ipa); OK(configure(ipa)); OK(cc_ipa_start(ipa)); OK(prepare(ipa, &settings)); cc_ipa_destroy(ipa);
}
static void failures(void)
{
    cc_ipa *ipa = (void *)(uintptr_t)1; ERROR(create(1, &ipa), ERANGE); CHECK(!ipa);
    ERROR(create(11, &ipa), EPIPE); CHECK(!ipa); ERROR(create(14, &ipa), ETIMEDOUT); CHECK(!ipa);
    for (uint32_t mode = 2; mode <= 13; ++mode) {
        if (mode == 11) continue;
        OK(create(mode, &ipa));
        if (mode == 2) { ERROR(configure(ipa), EIO); cc_ipa_destroy(ipa); continue; }
        OK(configure(ipa));
        if (mode == 3) { ERROR(cc_ipa_start(ipa), EIO); cc_ipa_destroy(ipa); continue; }
        OK(cc_ipa_start(ipa));
        cc_ipa_settings settings, original; memset(&settings, 0xa5, sizeof(settings)); original = settings;
        int error = mode == 4 ? EIO : mode == 6 ? EPIPE : mode == 7 ? ETIMEDOUT : mode == 8 ? EPROTO : mode == 10 ? E2BIG : 0;
        if (error) {
            ERROR(prepare(ipa, &settings), error); CHECK(!memcmp(&settings, &original, sizeof(settings)));
            if (mode == 6 || mode == 7) ERROR(prepare(ipa, &settings), error);
        } else {
            OK(prepare(ipa, &settings));
            if (mode == 5 || mode == 9) {
                cc_ipa_metadata metadata, old; memset(&metadata, 0xa5, sizeof(metadata)); old = metadata;
                ERROR(process(ipa, &settings, &metadata), mode == 5 ? EIO : EPROTO); CHECK(!memcmp(&metadata, &old, sizeof(old)));
            }
            if (mode == 12) { cc_ipa_stop(ipa); ERROR(cc_ipa_start(ipa), ETIMEDOUT); }
        }
        cc_ipa_destroy(ipa);
    }
}
static void instances_and_fd_zero(void)
{
    cc_ipa *a = NULL, *b = NULL; OK(create(0, &a)); OK(create(0, &b));
    OK(configure(a)); OK(configure(b)); OK(cc_ipa_start(a)); OK(cc_ipa_start(b));
    cc_ipa_settings first, second; OK(prepare(a, &first)); OK(prepare(b, &second));
    CHECK(first.isp_controls[0].data != second.isp_controls[0].data);
    cc_ipa_destroy(b); cc_ipa_metadata metadata; OK(process(a, &first, &metadata)); cc_ipa_destroy(a);
    int saved = dup(0); (void)close(0); OK(create(0, &a)); cc_ipa_destroy(a);
    CHECK(fcntl(0, F_GETFD) < 0 && errno == EBADF);
    if (saved >= 0) { CHECK(dup2(saved, 0) == 0); OK(close(saved)); }
}
static void paths(void)
{
    cc_ipa *ipa = (void *)(uintptr_t)1; cc_ipa_proxy_options saved = options;
    options.timeout_ms = -2; ERROR(create(0, &ipa), EINVAL); CHECK(!ipa); options = saved;
    options.worker_path = "/ccamera-missing-worker"; ERROR(create(0, &ipa), ENOENT); CHECK(!ipa);
    options.worker_path = ""; ERROR(create(0, &ipa), EINVAL); options.worker_path = "/"; ERROR(create(0, &ipa), EINVAL);
    options = saved;
    char temporary[] = "/tmp/ccamera-proxy-path-XXXXXX"; CHECK(mkdtemp(temporary));
    char executable[512]; CHECK(snprintf(executable, sizeof(executable), "%s/ccamera-ipa-worker", temporary) > 0);
    int fd = open(executable, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600); CHECK(fd >= 0);
    CHECK(write(fd, "invalid executable\n", 19) == 19); OK(close(fd));
    options.worker_path = executable; ERROR(create(0, &ipa), EACCES); OK(chmod(executable, 0700));
    ERROR(create(0, &ipa), ENOEXEC); options.worker_path = NULL;
    OK(setenv("CCAMERA_IPA_PROXY_PATH", temporary, 1)); ERROR(create(0, &ipa), ENOEXEC); /* First match does not fall back. */
    OK(unlink(executable)); OK(symlink(saved.worker_path, executable));
    OK(create(0, &ipa)); cc_ipa_destroy(ipa);
    char *oversized = malloc(65538); CHECK(oversized); memset(oversized, 'x', 65537); oversized[65537] = 0;
    OK(setenv("CCAMERA_IPA_PROXY_PATH", oversized, 1)); ERROR(create(0, &ipa), E2BIG); free(oversized);
    OK(unsetenv("CCAMERA_IPA_PROXY_PATH")); OK(unlink(executable)); OK(rmdir(temporary)); options = saved;
}
static void parent_exit(void)
{
    int subreaper = 0; OK(prctl(PR_GET_CHILD_SUBREAPER, &subreaper)); OK(prctl(PR_SET_CHILD_SUBREAPER, 1));
    int pipes[2]; OK(pipe2(pipes, O_CLOEXEC)); pid_t parent = fork(); CHECK(parent >= 0);
    if (!parent) {
        close(pipes[0]); cc_ipa *ipa = NULL; OK(create(0, &ipa));
        /* The most recent constructor trace identifies this parent's worker. */
        FILE *file = fopen(trace_path, "r"); CHECK(file); int pid, worker = 0; char event;
        while (fscanf(file, "%d %c", &pid, &event) == 2) if (event == 'L') worker = pid;
        OK(fclose(file)); CHECK(worker > 0 && worker != getpid());
        CHECK(write(pipes[1], &worker, sizeof(worker)) == sizeof(worker)); _exit(0);
    }
    OK(close(pipes[1])); int worker; CHECK(read(pipes[0], &worker, sizeof(worker)) == sizeof(worker)); OK(close(pipes[0]));
    int status; CHECK(waitpid(parent, &status, 0) == parent && WIFEXITED(status) && !WEXITSTATUS(status));
    pid_t waited = waitpid(worker, &status, 0);
    if (waited != worker || !WIFEXITED(status) || WEXITSTATUS(status))
        fprintf(stderr, "Parent-exit worker: waitpid=%ld status=%#x errno=%d\n", (long)waited, status, errno);
    CHECK(waited == worker && WIFEXITED(status) && !WEXITSTATUS(status));
    OK(prctl(PR_SET_CHILD_SUBREAPER, subreaper));
}
static void traces(void)
{
    FILE *file = fopen(trace_path, "r"); CHECK(file); int pid; char event; unsigned count = 0;
    while (fscanf(file, "%d %c", &pid, &event) == 2) { CHECK(pid > 0 && pid != getpid()); ++count; }
    CHECK(feof(file) && count > 20); OK(fclose(file));
    file = fopen("/proc/self/maps", "r"); CHECK(file); char *line = NULL; size_t capacity = 0;
    while (getline(&line, &capacity, file) >= 0) CHECK(!strstr(line, fixture));
    free(line); OK(fclose(file));
}
static void dependency_modules(const char *origin, const char *unresolved)
{
    cc_ipa_module *module = NULL; cc_ipa *ipa = NULL; uint32_t mode = 7;
    OK(cc_ipa_module_open(origin, &module)); OK(cc_ipa_module_create_proxy(module, &mode, sizeof(mode), &options, &ipa));
    cc_ipa_module_close(module);
    cc_ipa_configuration config = {.pipeline = CC_IPA_VC4, .source = {.width = 97}};
    OK(cc_ipa_configure(ipa, &config)); OK(cc_ipa_start(ipa)); cc_ipa_request request = {0}; cc_ipa_settings settings;
    OK(cc_ipa_prepare(ipa, &request, &settings)); cc_ipa_frame frame = {.statistics = {.luminance = 0.25}}; cc_ipa_metadata metadata;
    OK(cc_ipa_process(ipa, &frame, &metadata)); CHECK(metadata.exposure_ns == 97); cc_ipa_destroy(ipa);
    OK(cc_ipa_module_open(unresolved, &module));
    ERROR(cc_ipa_module_create_proxy(module, &mode, sizeof(mode), &options, &ipa), ENOEXEC); CHECK(!ipa); cc_ipa_module_close(module);
    char temporary[] = "/tmp/ccamera-proxy-stale-XXXXXX"; int fd = mkstemp(temporary); CHECK(fd >= 0);
    FILE *input = fopen(fixture, "rb"); CHECK(input); unsigned char bytes[4096]; size_t size;
    while ((size = fread(bytes, 1, sizeof(bytes), input))) CHECK(write(fd, bytes, size) == (ssize_t)size);
    CHECK(!ferror(input)); OK(fclose(input)); OK(close(fd));
    OK(cc_ipa_module_open(temporary, &module)); fd = open(temporary, O_WRONLY | O_APPEND | O_CLOEXEC); CHECK(fd >= 0);
    CHECK(write(fd, "x", 1) == 1); OK(close(fd)); struct proxy_fixture_init init_data = init(0);
    ERROR(cc_ipa_module_create_proxy(module, &init_data, sizeof(init_data), &options, &ipa), ESTALE);
    CHECK(!ipa); cc_ipa_module_close(module); OK(unlink(temporary));
}
int main(int argc, char **argv)
{
    CHECK(argc == 5); options.worker_path = argv[1]; options.timeout_ms = 500; fixture = argv[2]; size_t before = descriptors();
    int trace = mkstemp(trace_path); CHECK(trace >= 0); OK(close(trace)); OK(setenv("CCAMERA_TEST_PROXY_TRACE", trace_path, 1));
    cc_ipa *probe = NULL; int ret = create(0, &probe);
    if (ret == -ENOSYS || ret == -EPERM || ret == -EACCES) { fprintf(stderr, "Namespace process unavailable: %s\n", strerror(-ret)); return 77; }
    if (ret) fprintf(stderr, "proxy create: %d\n", ret);
    OK(ret); cc_ipa_destroy(probe);
    successful(); failures(); instances_and_fd_zero(); paths(); parent_exit(); traces(); dependency_modules(argv[3], argv[4]);
    OK(unsetenv("CCAMERA_TEST_PROXY_TRACE")); OK(unlink(trace_path)); CHECK(descriptors() == before);
    int status; CHECK(waitpid(-1, &status, WNOHANG) < 0 && errno == ECHILD);
    puts("Isolated module creation, deep callback data, namespace, timeout and reaping checks passed"); return 0;
}
