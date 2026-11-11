/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <ccamera/ipa_proxy.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/filter.h>
#include <linux/seccomp.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
#define OK(x) CHECK((x) == 0)
#define ERROR(x,e) CHECK((x) == -(e))
static char directory[] = "/tmp/ccamera-ipa-signature-XXXXXX";
static char module_path[256], signature_path[256], trace_path[256];
static int read_failure, change_after_read;
static const char *fixture_path;
static void copy(const char *source, const char *destination)
{
    FILE *in = fopen(source, "rb"), *out = fopen(destination, "wb"); CHECK(in && out);
    unsigned char data[4096]; size_t n;
    while ((n = fread(data, 1, sizeof(data), in))) CHECK(fwrite(data, 1, n, out) == n);
    CHECK(!ferror(in)); OK(fclose(in)); OK(fclose(out));
}
static void write_bytes(const char *path, const void *bytes, size_t size)
{
    FILE *file = fopen(path, "wb"); CHECK(file);
    CHECK(fwrite(bytes, 1, size, file) == size); OK(fclose(file));
}
/* Linker syscall seam: change the file after a verification read, ensuring the
 * final identity check rejects both in-place changes and atomic replacement. */
ssize_t __real_pread(int fd, void *data, size_t size, off_t offset);
ssize_t __real_pread64(int fd, void *data, size_t size, off_t offset);
static ssize_t finish_read(ssize_t result)
{
    if (read_failure) { read_failure = 0; errno = EIO; return -1; }
    int change = change_after_read; change_after_read = 0;
    if (change == 1) {
        int fd = open(module_path, O_WRONLY | O_APPEND); CHECK(fd >= 0);
        CHECK(write(fd, "x", 1) == 1); OK(close(fd));
    } else if (change == 2) {
        char replacement[256]; CHECK(snprintf(replacement, sizeof(replacement), "%s/new.so", directory) > 0);
        copy(fixture_path, replacement); OK(rename(replacement, module_path));
    }
    return result;
}
ssize_t __wrap_pread(int fd, void *data, size_t size, off_t offset)
{
    return finish_read(__real_pread(fd, data, size, offset));
}
ssize_t __wrap_pread64(int fd, void *data, size_t size, off_t offset)
{
    return finish_read(__real_pread64(fd, data, size, offset));
}
static void trace_reset(void) { write_bytes(trace_path, "", 0); }
static void trace_check(bool parent, bool loaded)
{
    FILE *file = fopen(trace_path, "r"); CHECK(file);
    long pid; char event; unsigned loads = 0, creates = 0;
    while (fscanf(file, "%ld %c", &pid, &event) == 2) {
        CHECK((pid == (long)getpid()) == parent);
        if (event == 'L') ++loads;
        if (event == 'C') ++creates;
    }
    OK(fclose(file)); CHECK(loads == (loaded ? 1U : 0U)); CHECK(creates == (loaded ? 1U : 0U));
}
static void verify(cc_ipa_module *module, cc_ipa_signature_status expected)
{
    cc_ipa_signature_status status = CC_IPA_SIGNATURE_VALID;
    OK(cc_ipa_module_verify_signature(module, &status)); CHECK(status == expected);
}
static void isolated(cc_ipa_module *module, const char *worker, bool forced)
{
    trace_reset();
    cc_ipa_auto_options options = {.flags = forced ? CC_IPA_AUTO_FORCE_ISOLATION : 0, .proxy = {.worker_path = worker}};
    cc_ipa_execution_mode mode = CC_IPA_EXECUTION_IN_PROCESS;
    cc_ipa *ipa = (void *)(uintptr_t)1;
    int ret = cc_ipa_module_create_auto(module, NULL, 0, &options, &ipa, &mode);
    if (ret == -EPERM || ret == -EACCES || ret == -ENOSYS || ret == -EOPNOTSUPP || ret == -EINVAL) {
        CHECK(!ipa && mode == CC_IPA_EXECUTION_IN_PROCESS); trace_check(false, false);
        puts("Isolated creation denied by namespace policy; checked error propagation and no parent loading");
    } else {
        CHECK(ret == 0 && ipa && mode == CC_IPA_EXECUTION_ISOLATED);
        cc_ipa_configuration config = {.source = {.width = 640}};
        OK(cc_ipa_configure(ipa, &config)); OK(cc_ipa_start(ipa));
        cc_ipa_frame frame = {0}; cc_ipa_metadata metadata;
        OK(cc_ipa_process(ipa, &frame, &metadata)); CHECK(metadata.exposure_ns == 640);
        cc_ipa_destroy(ipa); trace_check(false, true);
    }
}
static void denied_namespaces(cc_ipa_module *module, const char *worker, bool forced)
{
    trace_reset();
    pid_t child = fork(); CHECK(child >= 0);
    if (!child) {
        struct sock_filter filter[] = {
            BPF_STMT(BPF_LD | BPF_W | BPF_ABS, offsetof(struct seccomp_data, nr)),
#ifdef SYS_clone3
            BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, SYS_clone3, 0, 1),
            BPF_STMT(BPF_RET | BPF_K, SECCOMP_RET_ERRNO | EPERM),
#endif
            BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, SYS_clone, 0, 1),
            BPF_STMT(BPF_RET | BPF_K, SECCOMP_RET_ERRNO | EPERM),
            BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, SYS_unshare, 0, 1),
            BPF_STMT(BPF_RET | BPF_K, SECCOMP_RET_ERRNO | EPERM),
            BPF_STMT(BPF_RET | BPF_K, SECCOMP_RET_ALLOW)
        };
        struct sock_fprog program = {(unsigned short)(sizeof(filter) / sizeof(filter[0])), filter};
        if (prctl(PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0) || prctl(PR_SET_SECCOMP, SECCOMP_MODE_FILTER, &program)) {
            if (errno == EINVAL || errno == ENOSYS || errno == EPERM || errno == EACCES) _exit(77);
            CHECK(false);
        }
        cc_ipa_auto_options options = {.flags = forced ? CC_IPA_AUTO_FORCE_ISOLATION : 0, .proxy = {.worker_path = worker}};
        cc_ipa *ipa = (void *)(uintptr_t)1; cc_ipa_execution_mode mode = CC_IPA_EXECUTION_IN_PROCESS;
        ERROR(cc_ipa_module_create_auto(module, NULL, 0, &options, &ipa, &mode), EPERM);
        CHECK(!ipa && mode == CC_IPA_EXECUTION_IN_PROCESS);
        _exit(0);
    }
    int status; CHECK(waitpid(child, &status, 0) == child);
    CHECK(WIFEXITED(status) && (WEXITSTATUS(status) == 0 || WEXITSTATUS(status) == 77));
    trace_check(false, false);
}
int main(int argc, char **argv)
{
    CHECK(argc == 3); CHECK(mkdtemp(directory));
    fixture_path = argv[1];
    CHECK(snprintf(module_path, sizeof(module_path), "%s/module.so", directory) > 0);
    CHECK(snprintf(signature_path, sizeof(signature_path), "%s/module.so.sign", directory) > 0);
    CHECK(snprintf(trace_path, sizeof(trace_path), "%s/trace", directory) > 0);
    OK(setenv("CCAMERA_TEST_MODULE_PID_TRACE", trace_path, 1)); trace_reset();
    copy(argv[1], module_path);
    cc_ipa_module *module = NULL; cc_ipa *ipa = (void *)(uintptr_t)1;
    cc_ipa_signature_status status = CC_IPA_SIGNATURE_VALID;
    ERROR(cc_ipa_module_verify_signature(NULL, &status), EINVAL); CHECK(status == CC_IPA_SIGNATURE_VALID);
    ERROR(cc_ipa_manager_verify_signature(NULL, 0, &status), EINVAL);
    OK(cc_ipa_module_open(module_path, &module));
#ifdef CCAMERA_WITH_IPA_CRYPTO
    verify(module, CC_IPA_SIGNATURE_MISSING);
#else
    verify(module, CC_IPA_SIGNATURE_UNAVAILABLE);
#endif
    ERROR(cc_ipa_module_verify_signature(module, NULL), EINVAL);
    cc_ipa_execution_mode mode = CC_IPA_EXECUTION_IN_PROCESS;
    cc_ipa_auto_options bad = {.flags = 2};
    ERROR(cc_ipa_module_create_auto(module, NULL, 0, &bad, &ipa, &mode), EINVAL); CHECK(!ipa && mode == CC_IPA_EXECUTION_IN_PROCESS);
    bad.flags = 0; bad.proxy.timeout_ms = -2;
    ERROR(cc_ipa_module_create_auto(module, NULL, 0, &bad, &ipa, &mode), EINVAL); CHECK(!ipa);
    ERROR(cc_ipa_module_create_auto(module, NULL, 1, NULL, &ipa, &mode), EINVAL); CHECK(!ipa);
    ERROR(cc_ipa_module_create_auto(NULL, NULL, 0, NULL, &ipa, &mode), EINVAL); CHECK(!ipa);
    isolated(module, argv[2], false); denied_namespaces(module, argv[2], false); cc_ipa_module_close(module);
#ifdef CCAMERA_WITH_IPA_CRYPTO
    char reference[4096]; CHECK(snprintf(reference, sizeof(reference), "%s.reference.sign", argv[1]) > 0);
    copy(reference, signature_path);
    OK(cc_ipa_module_open(module_path, &module)); verify(module, CC_IPA_SIGNATURE_VALID); trace_reset();
    /* Signature data is captured: later sidecar changes cannot change trust. */
    write_bytes(signature_path, "bad", 3); verify(module, CC_IPA_SIGNATURE_VALID); trace_check(true, false);
    mode = CC_IPA_EXECUTION_ISOLATED;
    OK(cc_ipa_module_create_auto(module, NULL, 0, NULL, &ipa, &mode)); CHECK(mode == CC_IPA_EXECUTION_IN_PROCESS);
    cc_ipa_module_close(module); cc_ipa_destroy(ipa); trace_check(true, true);
    copy(reference, signature_path); OK(cc_ipa_module_open(module_path, &module));
    /* A verified factory error must not retry in a worker. */
    uint32_t factory_mode = 5; mode = CC_IPA_EXECUTION_ISOLATED; trace_reset();
    ERROR(cc_ipa_module_create_auto(module, &factory_mode, sizeof(factory_mode), NULL, &ipa, &mode), ERANGE);
    CHECK(!ipa && mode == CC_IPA_EXECUTION_ISOLATED); cc_ipa_module_close(module); trace_check(true, true);
    OK(cc_ipa_module_open(module_path, &module));
    cc_ipa_auto_options force = {.flags = CC_IPA_AUTO_FORCE_ISOLATION, .proxy = {.worker_path = "/nonexistent/ccamera-worker"}};
    ERROR(cc_ipa_module_create_auto(module, NULL, 0, &force, &ipa, &mode), ENOENT); CHECK(!ipa && mode == CC_IPA_EXECUTION_ISOLATED);
    isolated(module, argv[2], true); denied_namespaces(module, argv[2], true); cc_ipa_module_close(module);
    /* Wrong key and malformed lengths, including an empty readable sidecar. */
    CHECK(snprintf(reference, sizeof(reference), "%s.wrong.sign", argv[1]) > 0); copy(reference, signature_path);
    OK(cc_ipa_module_open(module_path, &module)); verify(module, CC_IPA_SIGNATURE_INVALID);
    isolated(module, argv[2], false); cc_ipa_module_close(module);
    for (size_t size = 0; size < 3; ++size) {
        write_bytes(signature_path, "bad", size); OK(cc_ipa_module_open(module_path, &module));
        verify(module, CC_IPA_SIGNATURE_INVALID); cc_ipa_module_close(module);
    }
    unsigned char malformed[256]; memset(malformed, 255, sizeof(malformed));
    write_bytes(signature_path, malformed, sizeof(malformed)); OK(cc_ipa_module_open(module_path, &module));
    verify(module, CC_IPA_SIGNATURE_INVALID); cc_ipa_module_close(module);
    CHECK(snprintf(reference, sizeof(reference), "%s.reference.sign", argv[1]) > 0); copy(reference, signature_path);
    for (int change = 1; change <= 2; ++change) {
        copy(argv[1], module_path); OK(cc_ipa_module_open(module_path, &module));
        status = CC_IPA_SIGNATURE_UNAVAILABLE; change_after_read = change;
        ERROR(cc_ipa_module_verify_signature(module, &status), ESTALE);
        CHECK(status == CC_IPA_SIGNATURE_UNAVAILABLE); cc_ipa_module_close(module);
    }
    copy(argv[1], module_path); OK(cc_ipa_module_open(module_path, &module));
    status = CC_IPA_SIGNATURE_UNAVAILABLE; read_failure = 1;
    ERROR(cc_ipa_module_verify_signature(module, &status), EIO); CHECK(status == CC_IPA_SIGNATURE_UNAVAILABLE);
    read_failure = 1; mode = CC_IPA_EXECUTION_IN_PROCESS;
    ERROR(cc_ipa_module_create_auto(module, NULL, 0, NULL, &ipa, &mode), EIO);
    CHECK(!ipa && mode == CC_IPA_EXECUTION_IN_PROCESS); cc_ipa_module_close(module);
    /* Appending bytes leaves ELF inspection valid but must invalidate the digest. */
    FILE *file = fopen(module_path, "ab"); CHECK(file); CHECK(fputc(1, file) == 1); OK(fclose(file));
    OK(cc_ipa_module_open(module_path, &module)); verify(module, CC_IPA_SIGNATURE_INVALID); cc_ipa_module_close(module);
    copy(argv[1], module_path);
#endif
    trace_reset(); OK(cc_ipa_module_open(module_path, &module));
    OK(truncate(module_path, 20)); status = CC_IPA_SIGNATURE_VALID;
    ERROR(cc_ipa_module_verify_signature(module, &status), ESTALE); CHECK(status == CC_IPA_SIGNATURE_VALID);
    mode = CC_IPA_EXECUTION_IN_PROCESS;
    ERROR(cc_ipa_module_create_auto(module, NULL, 0, NULL, &ipa, &mode), ESTALE); CHECK(!ipa && mode == CC_IPA_EXECUTION_IN_PROCESS);
    cc_ipa_module_close(module); trace_check(true, false); copy(argv[1], module_path);
    OK(cc_ipa_module_open(module_path, &module)); OK(unlink(module_path)); copy(argv[1], module_path);
    ERROR(cc_ipa_module_verify_signature(module, &status), ESTALE); cc_ipa_module_close(module);
    int saved = dup(STDIN_FILENO); (void)close(STDIN_FILENO);
    OK(cc_ipa_module_open(module_path, &module));
#ifdef CCAMERA_WITH_IPA_CRYPTO
    verify(module, CC_IPA_SIGNATURE_VALID);
#else
    verify(module, CC_IPA_SIGNATURE_UNAVAILABLE);
#endif
    cc_ipa_module_close(module); CHECK(fcntl(STDIN_FILENO, F_GETFD) == -1 && errno == EBADF);
    if (saved >= 0) { CHECK(dup2(saved, STDIN_FILENO) == STDIN_FILENO); OK(close(saved)); }
    cc_ipa_module_directory d = {directory, 0}; cc_ipa_manager_options options = {.directories = &d, .num_directories = 1};
    cc_ipa_manager *manager = NULL; OK(cc_ipa_manager_open(&options, &manager)); CHECK(cc_ipa_manager_count(manager) == 1);
    ERROR(cc_ipa_manager_verify_signature(manager, 1, &status), EINVAL);
    OK(cc_ipa_manager_verify_signature(manager, 0, &status));
#ifdef CCAMERA_WITH_IPA_CRYPTO
    CHECK(status == CC_IPA_SIGNATURE_VALID); mode = CC_IPA_EXECUTION_ISOLATED;
    OK(cc_ipa_manager_create_auto(manager, "TestPipeline", 7, 7, NULL, NULL, 0, NULL, &ipa, &mode));
    CHECK(mode == CC_IPA_EXECUTION_IN_PROCESS); cc_ipa_destroy(ipa);
    /* First match must win over a later valid candidate, even on failure. */
    char second[256], second_sign[256];
    CHECK(snprintf(second, sizeof(second), "%s/second.so", directory) > 0);
    CHECK(snprintf(second_sign, sizeof(second_sign), "%s/second.so.sign", directory) > 0);
    copy(module_path, second); copy(signature_path, second_sign);
    cc_ipa_manager_close(manager); OK(cc_ipa_manager_open(&options, &manager)); CHECK(cc_ipa_manager_count(manager) == 2);
    uint32_t failure = 5; mode = CC_IPA_EXECUTION_ISOLATED; trace_reset();
    ERROR(cc_ipa_manager_create_auto(manager, "TestPipeline", 7, 7, NULL, &failure, sizeof(failure), NULL, &ipa, &mode), ERANGE);
    CHECK(!ipa && mode == CC_IPA_EXECUTION_ISOLATED); trace_check(true, true);
    OK(unlink(second)); OK(unlink(second_sign));
#else
    CHECK(status == CC_IPA_SIGNATURE_UNAVAILABLE);
#endif
    ERROR(cc_ipa_manager_create_auto(manager, "absent", 0, 9, NULL, NULL, 0, NULL, &ipa, &mode), ENOENT); CHECK(!ipa);
    cc_ipa_manager_close(manager);
    OK(unlink(module_path)); (void)unlink(signature_path); OK(unlink(trace_path)); OK(rmdir(directory));
    puts("IPA signature status, snapshots, automatic execution and unchanged outputs passed"); return 0;
}
