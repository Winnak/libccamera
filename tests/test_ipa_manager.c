/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include "ipa_manager.h"
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/auxv.h>
#include <sys/stat.h>
#include <unistd.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
#define OK(x) CHECK((x) == 0)
#define ERROR(x,e) CHECK((x) == -(e))
struct manager_vector { unsigned order, depth; uint32_t minimum, maximum; size_t count; const char *selected; };
#include "ipa_manager_vectors.h"
static char root[] = "/tmp/ccamera-ipa-manager-XXXXXX";
static char trace_path[256];
static void path(char out[256], const char *relative)
{
    int n = snprintf(out, 256, "%s/%s", root, relative); CHECK(n > 0 && n < 256);
}
static void directory(const char *relative)
{
    char name[256]; path(name, relative); OK(mkdir(name, 0700));
}
static void copy(const char *source, const char *relative)
{
    char name[256]; path(name, relative);
    FILE *input = fopen(source, "rb"), *output = fopen(name, "wb"); CHECK(input && output);
    unsigned char bytes[4096]; size_t size;
    while ((size = fread(bytes, 1, sizeof(bytes), input))) CHECK(fwrite(bytes, 1, size, output) == size);
    CHECK(!ferror(input)); OK(fclose(input)); OK(fclose(output));
}
static void reset_trace(void) { FILE *f = fopen(trace_path, "wb"); CHECK(f); OK(fclose(f)); }
static void expect_trace(const char *expected)
{
    char bytes[256]; FILE *f = fopen(trace_path, "rb"); CHECK(f);
    size_t n = fread(bytes, 1, sizeof(bytes) - 1, f); CHECK(!ferror(f)); OK(fclose(f)); bytes[n] = 0;
    if (strcmp(bytes, expected)) fprintf(stderr, "Trace: %s, expected: %s\n", bytes, expected);
    CHECK(!strcmp(bytes, expected));
}
static size_t descriptors(void)
{
    DIR *d = opendir("/proc/self/fd"); CHECK(d); size_t count = 0;
    struct dirent *entry;
    while ((entry = readdir(d))) if (strcmp(entry->d_name, ".") && strcmp(entry->d_name, "..")) ++count;
    OK(closedir(d)); return count;
}
static void setup(char **modules)
{
    directory("A"); directory("B"); directory("A/nested"); directory("A/nested/deeper"); directory("A/fake.so");
    copy(modules[0], "A/z-last.so"); copy(modules[1], "A/a-first.so"); copy(modules[2], "A/ignored.so.1");
    copy(modules[2], "A/nested/b-middle.so"); copy(modules[0], "A/nested/deeper/c-deep.so");
    copy(modules[2], "A/fake.so/d.so"); copy(modules[3], "A/00-bad.so");
    copy(modules[2], "B/0-override.so"); copy(modules[1], "B/a-other.so");
    char name[256]; path(name, "A/alias.so"); OK(symlink("z-last.so", name));
    path(name, "A/linked-directory"); OK(symlink("nested", name));
}
static void test_vectors(void)
{
    char a[256], b[256]; path(a, "A"); path(b, "B");
    reset_trace(); size_t before = descriptors();
    for (size_t i = 0; i < sizeof(manager_vectors) / sizeof(manager_vectors[0]); ++i) {
        const struct manager_vector *v = &manager_vectors[i];
        cc_ipa_module_directory directories[2] = {{a, v->depth}, {b, v->depth}};
        if (v->order == 1 || v->order == 3) { directories[0].path = b; directories[1].path = a; }
        cc_ipa_manager_options options = {.directories = directories, .num_directories = v->order < 2 ? 1 : 2};
        cc_ipa_manager *manager = NULL; OK(cc_ipa_manager_open(&options, &manager));
        CHECK(cc_ipa_manager_count(manager) == v->count);
        size_t index = 999;
        int ret = cc_ipa_manager_select(manager, "TestPipeline", v->minimum, v->maximum, NULL, &index);
        if (!v->selected) { CHECK(ret == -ENOENT && index == 999); }
        else {
            CHECK(ret == 0 && index < v->count); char expected[256]; path(expected, v->selected);
            CHECK(!strcmp(cc_ipa_manager_get_path(manager, index), expected));
            const cc_ipa_module_info *info = cc_ipa_manager_get_info(manager, index);
            CHECK(info && info->pipeline_version >= v->minimum && info->pipeline_version <= v->maximum);
        }
        CHECK(!cc_ipa_manager_get_info(manager, v->count) && !cc_ipa_manager_get_path(manager, v->count));
        index = 999; ERROR(cc_ipa_manager_select(manager, "other", 0, UINT32_MAX, NULL, &index), ENOENT); CHECK(index == 999);
        cc_ipa_manager_close(manager); CHECK(descriptors() == before);
    }
    expect_trace("");
}
static void test_selection(const char *module)
{
    char a[256]; path(a, "A"); cc_ipa_module_directory d = {a, 2};
    cc_ipa_manager_options options = {.directories = &d, .num_directories = 1};
    cc_ipa_manager *manager = NULL; reset_trace(); OK(cc_ipa_manager_open(&options, &manager));
    size_t index = 999;
    OK(cc_ipa_manager_select(manager, "TestPipeline", 0, UINT32_MAX, "alternate", &index));
    CHECK(cc_ipa_manager_get_info(manager, index)->pipeline_version == 4);
    ERROR(cc_ipa_manager_select(manager, "TestPipeline", 7, 7, "alternate", &index), ENOENT); CHECK(index == 0);
    ERROR(cc_ipa_manager_select(manager, "TestPipeline", 8, 7, NULL, &index), EINVAL);
    ERROR(cc_ipa_manager_select(manager, "TestPipeline", 0, 9, "", &index), EINVAL);
    ERROR(cc_ipa_manager_select(manager, "", 0, 9, NULL, &index), EINVAL);
    const void *signature = (void *)(uintptr_t)1; size_t signature_size = 1;
    OK(cc_ipa_manager_get_signature(manager, 0, &signature, &signature_size)); CHECK(!signature && !signature_size);
    signature = (void *)(uintptr_t)1; signature_size = 1;
    ERROR(cc_ipa_manager_get_signature(manager, 999, &signature, &signature_size), EINVAL);
    CHECK(signature == (void *)(uintptr_t)1 && signature_size == 1);
    cc_ipa *ipa = NULL; uint32_t mode = 7;
    OK(cc_ipa_manager_create(manager, "TestPipeline", 0, 9, "alternate", &mode, sizeof(mode), &ipa));
    expect_trace("LC"); cc_ipa_manager_close(manager); expect_trace("LC");
    cc_ipa_configuration config = {.source = {.width = 123}}; cc_ipa_request request = {0}; cc_ipa_settings settings;
    OK(cc_ipa_configure(ipa, &config)); OK(cc_ipa_start(ipa)); OK(cc_ipa_prepare(ipa, &request, &settings));
    cc_ipa_destroy(ipa); expect_trace("LCcsptDU");
    /* Duplicated directories/aliases retain their reference ordering. */
    cc_ipa_module_directory repeated[] = {{a, 0}, {a, 0}}; options.directories = repeated; options.num_directories = 2;
    reset_trace(); OK(cc_ipa_manager_open(&options, &manager)); CHECK(cc_ipa_manager_count(manager) == 6);
    cc_ipa_manager_close(manager); expect_trace("");
    /* A manager is a snapshot: later directory arrivals are not added. */
    directory("snapshot"); char snapshot[256]; path(snapshot, "snapshot");
    d = (cc_ipa_module_directory){snapshot, 0}; options.directories = &d; options.num_directories = 1;
    OK(cc_ipa_manager_open(&options, &manager)); copy(module, "snapshot/new.so");
    CHECK(cc_ipa_manager_count(manager) == 0); cc_ipa_manager_close(manager);
    OK(cc_ipa_manager_open(&options, &manager)); CHECK(cc_ipa_manager_count(manager) == 1); cc_ipa_manager_close(manager);
}
static void test_failures(char **modules)
{
    directory("failure"); copy(modules[4], "failure/0-missing.so"); copy(modules[0], "failure/1-valid.so");
    char location[256]; path(location, "failure"); cc_ipa_module_directory d = {location, 0};
    cc_ipa_manager_options options = {.directories = &d, .num_directories = 1};
    cc_ipa_manager *manager = NULL; cc_ipa *ipa = (cc_ipa *)(uintptr_t)1; reset_trace();
    OK(cc_ipa_manager_open(&options, &manager)); CHECK(cc_ipa_manager_count(manager) == 2);
    ERROR(cc_ipa_manager_create(manager, "TestPipeline", 7, 7, NULL, NULL, 0, &ipa), ENOENT); CHECK(!ipa); expect_trace("");
    cc_ipa_manager_close(manager);
    copy(modules[5], "failure/0-missing.so"); OK(cc_ipa_manager_open(&options, &manager));
    ERROR(cc_ipa_manager_create(manager, "TestPipeline", 7, 7, NULL, NULL, 0, &ipa), ENOEXEC); CHECK(!ipa); expect_trace("");
    cc_ipa_manager_close(manager);
    copy(modules[0], "failure/0-missing.so"); OK(cc_ipa_manager_open(&options, &manager));
    uint32_t mode = 5; ERROR(cc_ipa_manager_create(manager, "TestPipeline", 7, 7, NULL, &mode, sizeof(mode), &ipa), ERANGE);
    CHECK(!ipa); cc_ipa_manager_close(manager); expect_trace("LCU");
    reset_trace(); OK(cc_ipa_manager_open(&options, &manager));
    char first[256]; path(first, "failure/0-missing.so"); OK(unlink(first));
    ERROR(cc_ipa_manager_create(manager, "TestPipeline", 7, 7, NULL, NULL, 0, &ipa), ESTALE);
    CHECK(!ipa); cc_ipa_manager_close(manager); expect_trace("");
}
static void test_environment(void)
{
    char a[256], b[256], environment[1024]; path(a, "A"); path(b, "B");
    CHECK(snprintf(environment, sizeof(environment), ":%s::%s:", a, b) > 0);
    cc_ipa_module_directory d = {b, 0}; cc_ipa_manager_options options = {
        .directories = &d, .num_directories = 1, .flags = CC_IPA_MANAGER_ENVIRONMENT};
    cc_ipa_manager *manager = NULL; size_t index = 99;
    OK(cc_ipa_manager_open_environment(&options, environment, false, NULL, NULL, &manager));
    CHECK(cc_ipa_manager_count(manager) == 7);
    OK(cc_ipa_manager_select(manager, "TestPipeline", 0, 9, NULL, &index));
    CHECK(cc_ipa_manager_get_info(manager, index)->pipeline_version == 9); cc_ipa_manager_close(manager);
    options.directories = NULL; options.num_directories = 0;
    OK(cc_ipa_manager_open_environment(&options, environment, true, NULL, NULL, &manager));
    CHECK(!cc_ipa_manager_count(manager)); cc_ipa_manager_close(manager);
    OK(setenv("CCAMERA_IPA_MODULE_PATH", a, 1));
    OK(cc_ipa_manager_open(&options, &manager));
    if (!getauxval(AT_SECURE) && getuid() == geteuid() && getgid() == getegid()) CHECK(cc_ipa_manager_count(manager) == 3);
    else CHECK(!cc_ipa_manager_count(manager));
    cc_ipa_manager_close(manager); OK(unsetenv("CCAMERA_IPA_MODULE_PATH"));
    /* Empty components do not search the working directory. */
    int cwd = open(".", O_RDONLY | O_DIRECTORY | O_CLOEXEC); CHECK(cwd >= 0); OK(chdir(a));
    OK(cc_ipa_manager_open_environment(&options, "::", false, NULL, NULL, &manager)); CHECK(!cc_ipa_manager_count(manager));
    cc_ipa_manager_close(manager); OK(fchdir(cwd)); OK(close(cwd));
    char *large = malloc(65538); CHECK(large); memset(large, 'x', 65537); large[65537] = 0;
    ERROR(cc_ipa_manager_open_environment(&options, large, false, NULL, NULL, &manager), E2BIG); CHECK(!manager);
    OK(cc_ipa_manager_open_environment(&options, large, true, NULL, NULL, &manager)); CHECK(!cc_ipa_manager_count(manager));
    cc_ipa_manager_close(manager); free(large);
    char many[256]; memset(many, 'x', sizeof(many)); for (size_t i = 1; i < 130; i += 2) many[i] = ':'; many[130] = 0;
    ERROR(cc_ipa_manager_open_environment(&options, many, false, NULL, NULL, &manager), E2BIG); CHECK(!manager);
    /* Default opening with an environment path also appends the system source. */
    OK(cc_ipa_manager_open_environment(NULL, a, false, b, NULL, &manager)); CHECK(cc_ipa_manager_count(manager) == 5);
    OK(cc_ipa_manager_select(manager, "TestPipeline", 0, 9, NULL, &index));
    CHECK(cc_ipa_manager_get_info(manager, index)->pipeline_version == 4);
    cc_ipa_manager_close(manager);
    /* Build modules follow environment paths, recurse to depth 2, and precede
     * the installed path. The default policy enables all three sources. */
    OK(cc_ipa_manager_open_environment(NULL, b, false, b, a, &manager));
    CHECK(cc_ipa_manager_count(manager) == 10);
    OK(cc_ipa_manager_select(manager, "TestPipeline", 0, 9, NULL, &index));
    CHECK(cc_ipa_manager_get_info(manager, index)->pipeline_version == 9);
    OK(cc_ipa_manager_select(manager, "TestPipeline", 7, 7, NULL, &index));
    CHECK(strstr(cc_ipa_manager_get_path(manager, index), "/A/z-last.so"));
    cc_ipa_manager_close(manager);
}
static void test_limits(void)
{
    char a[256], missing[256]; path(a, "A"); path(missing, "absent");
    cc_ipa_module_directory d = {a, 0}; cc_ipa_manager_options options = {.directories = &d, .num_directories = 1, .max_modules = 2};
    cc_ipa_manager *manager = NULL; size_t before = descriptors();
    ERROR(cc_ipa_manager_open(&options, &manager), E2BIG); CHECK(!manager && descriptors() == before);
    options.max_modules = CC_IPA_MANAGER_MAX_MODULES + 1;
    ERROR(cc_ipa_manager_open(&options, &manager), EINVAL); options.max_modules = 0;
    d.max_depth = CC_IPA_MANAGER_MAX_DEPTH + 1; ERROR(cc_ipa_manager_open(&options, &manager), EINVAL); d.max_depth = 0;
    options.num_directories = CC_IPA_MANAGER_MAX_DIRECTORIES + 1; ERROR(cc_ipa_manager_open(&options, &manager), EINVAL);
    options.num_directories = 1; options.directories = NULL; ERROR(cc_ipa_manager_open(&options, &manager), EINVAL);
    options.directories = &d; options.flags = UINT32_MAX; ERROR(cc_ipa_manager_open(&options, &manager), EINVAL); options.flags = 0;
    d.path = NULL; ERROR(cc_ipa_manager_open(&options, &manager), EINVAL);
    d.path = missing; OK(cc_ipa_manager_open(&options, &manager)); CHECK(!cc_ipa_manager_count(manager)); cc_ipa_manager_close(manager);
    directory("too-many"); char many[256]; path(many, "too-many"); d.path = many;
    for (unsigned i = 0; i < 4097; ++i) {
        char name[300]; CHECK(snprintf(name, sizeof(name), "%s/%04u.so", many, i) > 0); OK(symlink("absent", name));
    }
    ERROR(cc_ipa_manager_open(&options, &manager), E2BIG); CHECK(!manager && descriptors() == before);
    d.path = a;
    int saved = dup(STDIN_FILENO); (void)close(STDIN_FILENO);
    OK(cc_ipa_manager_open(&options, &manager)); int flags = fcntl(STDIN_FILENO, F_GETFD);
    CHECK(flags >= 0 && (flags & FD_CLOEXEC)); cc_ipa_manager_close(manager);
    CHECK(fcntl(STDIN_FILENO, F_GETFD) == -1 && errno == EBADF);
    if (saved >= 0) { CHECK(dup2(saved, STDIN_FILENO) == STDIN_FILENO); OK(close(saved)); }
}
static void cleanup(const char *name)
{
    DIR *directory = opendir(name); CHECK(directory); struct dirent *entry;
    while ((entry = readdir(directory))) {
        if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, "..")) continue;
        char child[512]; int n = snprintf(child, sizeof(child), "%s/%s", name, entry->d_name); CHECK(n > 0 && n < 512);
        struct stat status; OK(lstat(child, &status));
        if (S_ISDIR(status.st_mode)) cleanup(child); else OK(unlink(child));
    }
    OK(closedir(directory)); OK(rmdir(name));
}
int main(int argc, char **argv)
{
    CHECK(argc == 7); CHECK(mkdtemp(root)); path(trace_path, "trace"); OK(setenv("CCAMERA_TEST_MODULE_TRACE", trace_path, 1));
    cc_ipa_manager *manager = NULL; cc_ipa *ipa = NULL; size_t index = 1;
    ERROR(cc_ipa_manager_open(NULL, NULL), EINVAL); cc_ipa_manager_close(NULL);
    CHECK(!cc_ipa_manager_count(NULL) && !cc_ipa_manager_get_info(NULL, 0) && !cc_ipa_manager_get_path(NULL, 0));
    ERROR(cc_ipa_manager_select(NULL, "TestPipeline", 0, 9, NULL, &index), EINVAL); CHECK(index == 1);
    ERROR(cc_ipa_manager_create(NULL, "TestPipeline", 0, 9, NULL, NULL, 0, &ipa), EINVAL); CHECK(!ipa);
    cc_ipa_manager_options empty = {0}; OK(cc_ipa_manager_open(&empty, &manager)); CHECK(!cc_ipa_manager_count(manager));
    cc_ipa_manager_close(manager);
    setup(argv + 1); test_vectors(); test_selection(argv[1]); test_failures(argv + 1); test_environment(); test_limits();
    cleanup(root);
    puts("IPA discovery: 60 archived policies, precedence, bounded traversal, environment and retained instances passed");
    return 0;
}
