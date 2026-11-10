/* SPDX-License-Identifier: LGPL-2.1-or-later */
#define _GNU_SOURCE
#include "source_paths.h"
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <spawn.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
#define OK(x) CHECK((x) == 0)
extern char **environ;
static char temporary[] = "/tmp/ccamera-runtime-XXXXXX";
static bool proxy;
static void remove_tree(const char *directory);
static void path(char *out, size_t capacity, const char *directory, const char *name)
{
    int n = snprintf(out, capacity, "%s/%s", directory, name); CHECK(n > 0 && (size_t)n < capacity);
}
static void copy(const char *source, const char *destination)
{
    FILE *input = fopen(source, "rb"), *output = fopen(destination, "wb"); CHECK(input && output);
    unsigned char bytes[4096]; size_t size;
    while ((size = fread(bytes, 1, sizeof(bytes), input))) CHECK(fwrite(bytes, 1, size, output) == size);
    CHECK(!ferror(input)); OK(fclose(input)); OK(fclose(output)); OK(chmod(destination, 0700));
}
static void write_marker(const char *directory, const char *text)
{
    char file[512]; path(file, sizeof(file), directory, ".ccamera-build-tree");
    FILE *output = fopen(file, "wb"); CHECK(output);
    CHECK(fwrite(text, 1, strlen(text), output) == strlen(text)); OK(fclose(output));
}
static void run(const char *program, const char *count, const char *selected, const char *environment, const char *stdin_mode)
{
    char *arguments[] = {(char *)program, (char *)count, (char *)selected, (char *)environment, (char *)stdin_mode, NULL};
    pid_t pid; OK(posix_spawn(&pid, program, NULL, NULL, arguments, environ));
    int status; pid_t ret;
    do { ret = waitpid(pid, &status, 0); } while (ret < 0 && errno == EINTR);
    if (ret == pid && WIFEXITED(status) && WEXITSTATUS(status) == 77 && proxy) { remove_tree(temporary); exit(77); }
    CHECK(ret == pid && WIFEXITED(status) && !WEXITSTATUS(status));
}
static void remove_tree(const char *directory)
{
    DIR *d = opendir(directory); CHECK(d); struct dirent *entry;
    while ((entry = readdir(d))) {
        if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, "..")) continue;
        char child[1024]; path(child, sizeof(child), directory, entry->d_name);
        struct stat status; OK(lstat(child, &status));
        if (S_ISDIR(status.st_mode)) remove_tree(child); else OK(unlink(child));
    }
    OK(closedir(d)); OK(rmdir(directory));
}
int main(int argc, char **argv)
{
    CHECK((argc == 5 || argc == 6) && mkdtemp(temporary));
    proxy = argc == 6;
    OK(unsetenv("CCAMERA_IPA_PROXY_PATH"));
    if (proxy) OK(setenv("CCAMERA_TEST_RUNTIME_PROXY", "1", 1));
    else OK(unsetenv("CCAMERA_TEST_RUNTIME_PROXY"));
    /* Force the relocated executable to resolve its own core through $ORIGIN,
     * rather than an externally supplied copy in the test environment. */
    OK(unsetenv("LD_LIBRARY_PATH")); OK(unsetenv("CCAMERA_IPA_MODULE_PATH"));
    char stage[512], relocated[512], program[512], selected[512], marker[512];
    path(stage, sizeof(stage), temporary, "stage"); OK(mkdir(stage, 0700));
    path(program, sizeof(program), stage, "probe"); copy(argv[1], program);
    if (proxy) { char worker[512]; path(worker, sizeof(worker), stage, "ccamera-ipa-worker"); copy(argv[5], worker); }
    if (strcmp(argv[3], "-")) {
        char library[512]; path(library, sizeof(library), stage, argv[4]); copy(argv[3], library);
    }
    char ipa[512], one[512], two[512], three[512], deep[512];
    path(ipa, sizeof(ipa), stage, "ipa"); OK(mkdir(ipa, 0700));
    path(one, sizeof(one), ipa, "one"); OK(mkdir(one, 0700));
    path(two, sizeof(two), one, "two"); OK(mkdir(two, 0700));
    path(selected, sizeof(selected), two, "module.so"); copy(argv[2], selected);
    path(three, sizeof(three), two, "three"); OK(mkdir(three, 0700));
    path(deep, sizeof(deep), three, "too-deep.so"); copy(argv[2], deep);
    run(program, "0", "-", "build", "normal"); /* A copied image needs its build marker. */
    write_marker(stage, "not-a-ccamera-build\n"); run(program, "0", "-", "build", "normal");
    write_marker(stage, CC_BUILD_TREE_MAGIC "extra\n"); run(program, "0", "-", "build", "normal");
    write_marker(stage, CC_BUILD_TREE_MAGIC); run(program, "1", selected, "build", "normal");
    run(program, "1", selected, "build", "fd0");
    int cwd = open(".", O_RDONLY | O_DIRECTORY | O_CLOEXEC); CHECK(cwd >= 0);
    OK(chdir(stage)); run("./probe", "1", selected, "build", "cwd"); OK(fchdir(cwd)); OK(close(cwd));
    /* Environment sources precede the detected build tree. */
    char environment[512], preferred[512]; path(environment, sizeof(environment), temporary, "environment");
    OK(mkdir(environment, 0700)); path(preferred, sizeof(preferred), environment, "preferred.so"); copy(argv[2], preferred);
    OK(setenv("CCAMERA_IPA_MODULE_PATH", environment, 1)); run(program, "2", preferred, "environment", "normal");
    OK(unsetenv("CCAMERA_IPA_MODULE_PATH"));
    /* Relative image origins survive moving the entire tree. */
    path(relocated, sizeof(relocated), temporary, "relocated"); OK(rename(stage, relocated));
    path(program, sizeof(program), relocated, "probe");
    CHECK(snprintf(selected, sizeof(selected), "%s/ipa/one/two/module.so", relocated) > 0);
    run(program, "1", selected, "build", "normal");
    char alias[512]; path(alias, sizeof(alias), temporary, "alias"); OK(symlink(program, alias));
    run(alias, "1", selected, "build", "normal");
    path(marker, sizeof(marker), relocated, ".ccamera-build-tree"); OK(unlink(marker));
    run(program, "0", "-", "build", "normal");
    remove_tree(temporary);
    puts("Runtime build lookup, relocation, marker validation, source priority and fd zero passed");
    return 0;
}
