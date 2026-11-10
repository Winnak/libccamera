/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Directory precedence, sorted discovery and first-match policy follow
 * libcamera's ipa_manager.cpp. Copyright (C) 2019 Google Inc. */
#include "ipa_manager.h"
#include <ccamera/ipa_proxy.h>
#include "source_paths.h"
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/auxv.h>
#include <sys/stat.h>
#include <unistd.h>

#ifndef CCAMERA_IPA_MODULE_DIR
#define CCAMERA_IPA_MODULE_DIR "/usr/local/lib/ccamera/ipa"
#endif
#define CANDIDATE_LIMIT 4096U
#define ENTRY_LIMIT 65536U
#define ENVIRONMENT_LIMIT (64U * 1024U)
struct cc_ipa_manager {
    cc_ipa_module **modules;
    size_t count, maximum;
};
struct search {
    cc_ipa_module_directory directories[CC_IPA_MANAGER_MAX_DIRECTORIES];
    size_t count, entries, candidates;
};
struct files { char **paths; size_t count; };
static bool unavailable(int error)
{
    return error == ENOENT || error == EACCES || error == EPERM || error == ENOTDIR || error == ELOOP;
}
static int add_path(struct search *search, const char *path, size_t length, uint32_t depth)
{
    if (!length) return 0;
    if (search->count == CC_IPA_MANAGER_MAX_DIRECTORIES) return -E2BIG;
    if (length == SIZE_MAX) return -EOVERFLOW;
    char *copy = malloc(length + 1);
    if (!copy) return -ENOMEM;
    memcpy(copy, path, length); copy[length] = 0;
    search->directories[search->count++] = (cc_ipa_module_directory){copy, depth};
    return 0;
}
static void free_paths(struct search *search)
{
    for (size_t i = 0; i < search->count; ++i) free((char *)search->directories[i].path);
}
static int options_valid(const cc_ipa_manager_options *options)
{
    if (!options) return 0;
    if ((options->flags & ~(uint32_t)(CC_IPA_MANAGER_ENVIRONMENT | CC_IPA_MANAGER_BUILD_TREE | CC_IPA_MANAGER_INSTALLED)) ||
        options->num_directories > CC_IPA_MANAGER_MAX_DIRECTORIES ||
        (options->num_directories && !options->directories) ||
        options->max_modules > CC_IPA_MANAGER_MAX_MODULES) return -EINVAL;
    for (size_t i = 0; i < options->num_directories; ++i)
        if (!options->directories[i].path || options->directories[i].max_depth > CC_IPA_MANAGER_MAX_DEPTH) return -EINVAL;
    return 0;
}
static int search_paths(const cc_ipa_manager_options *options, const char *environment,
                        bool privileged, const char *installed, const char *build, struct search *search)
{
    cc_ipa_manager_options defaults = {.flags = CC_IPA_MANAGER_ENVIRONMENT | CC_IPA_MANAGER_BUILD_TREE | CC_IPA_MANAGER_INSTALLED};
    if (!options) options = &defaults;
    int validation = options_valid(options);
    if (validation < 0) return validation;
    for (size_t i = 0; i < options->num_directories; ++i) {
        const cc_ipa_module_directory *d = &options->directories[i];
        int ret = add_path(search, d->path, strlen(d->path), d->max_depth);
        if (ret < 0) return ret;
    }
    if ((options->flags & CC_IPA_MANAGER_ENVIRONMENT) && !privileged && environment) {
        size_t size = strnlen(environment, ENVIRONMENT_LIMIT + 1U);
        if (size > ENVIRONMENT_LIMIT) return -E2BIG;
        size_t start = 0;
        for (size_t i = 0; i <= size; ++i) if (environment[i] == ':' || !environment[i]) {
            int ret = add_path(search, environment + start, i - start, 0);
            if (ret < 0) return ret;
            start = i + 1;
        }
    }
    if ((options->flags & CC_IPA_MANAGER_BUILD_TREE) && build) {
        int ret = add_path(search, build, strlen(build), 2);
        if (ret < 0) return ret;
    }
    if (options->flags & CC_IPA_MANAGER_INSTALLED) {
        if (!installed) installed = CCAMERA_IPA_MODULE_DIR;
        return add_path(search, installed, strlen(installed), 0);
    }
    return 0;
}
void cc_ipa_manager_free_paths(cc_ipa_module_directory *directories, size_t count)
{
    for (size_t i = 0; i < count; ++i) free((char *)directories[i].path);
    free(directories);
}
int cc_ipa_manager_search_paths(const cc_ipa_manager_options *options, const char *environment,
                               bool privileged, const char *installed, const char *build,
                               cc_ipa_module_directory **directories, size_t *count)
{
    if (!directories || !count) return -EINVAL;
    struct search search = {0};
    int ret = search_paths(options, environment, privileged, installed, build, &search);
    if (ret < 0) { free_paths(&search); return ret; }
    cc_ipa_module_directory *result = NULL;
    if (search.count) {
        result = malloc(search.count * sizeof(*result));
        if (!result) { free_paths(&search); return -ENOMEM; }
        memcpy(result, search.directories, search.count * sizeof(*result));
    }
    *directories = result; *count = search.count;
    return 0;
}
static char *join(const char *directory, const char *name)
{
    size_t a = strlen(directory), b = strlen(name);
    if (a > SIZE_MAX - b - 2) return NULL;
    char *path = malloc(a + b + 2);
    if (!path) return NULL;
    memcpy(path, directory, a); path[a] = '/'; memcpy(path + a + 1, name, b + 1);
    return path;
}
static int scan_directory(struct search *search, const char *path, uint32_t depth, struct files *files, int source_fd)
{
    DIR *directory = source_fd >= 0 ? fdopendir(source_fd) : opendir(path);
    if (!directory) {
        int error = errno;
        if (source_fd >= 0) close(source_fd);
        return unavailable(error) ? 0 : -error;
    }
    int ret = 0;
    for (;;) {
        errno = 0;
        struct dirent *entry = readdir(directory);
        if (!entry) { if (errno) ret = -errno; break; }
        if (++search->entries > ENTRY_LIMIT) { ret = -E2BIG; break; }
        if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, "..")) continue;
        struct stat status;
        if (fstatat(dirfd(directory), entry->d_name, &status, AT_SYMLINK_NOFOLLOW)) {
            if (unavailable(errno)) continue;
            ret = -errno; break;
        }
        size_t length = strlen(entry->d_name);
        bool is_directory = S_ISDIR(status.st_mode);
        bool candidate = !is_directory && length >= 3 && !strcmp(entry->d_name + length - 3, ".so") &&
            (S_ISREG(status.st_mode) || S_ISLNK(status.st_mode));
        if ((!is_directory || !depth) && !candidate) continue;
        char *child = join(path, entry->d_name);
        if (!child) { ret = -ENOMEM; break; }
        if (is_directory) {
            /* Enforce the nested-symlink policy even if an entry changes
             * between fstatat and descent. fdopendir takes ownership. */
            int fd = openat(dirfd(directory), entry->d_name, O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
            ret = fd < 0 ? (unavailable(errno) ? 0 : -errno) :
                scan_directory(search, child, depth - 1, files, fd);
            free(child);
            if (ret < 0) break;
        } else {
            if (++search->candidates > CANDIDATE_LIMIT) { free(child); ret = -E2BIG; break; }
            char **paths = realloc(files->paths, (files->count + 1) * sizeof(*paths));
            if (!paths) { free(child); ret = -ENOMEM; break; }
            files->paths = paths; files->paths[files->count++] = child;
        }
    }
    closedir(directory);
    return ret;
}
static int compare_paths(const void *a, const void *b)
{
    return strcmp(*(const char *const *)a, *(const char *const *)b);
}
static int add_directory(cc_ipa_manager *manager, struct search *search, const cc_ipa_module_directory *directory)
{
    struct files files = {0};
    int ret = scan_directory(search, directory->path, directory->max_depth, &files, -1);
    if (ret < 0) goto done;
    if (files.count > 1) qsort(files.paths, files.count, sizeof(*files.paths), compare_paths);
    for (size_t i = 0; i < files.count; ++i) {
        cc_ipa_module *module = NULL;
        ret = cc_ipa_module_open(files.paths[i], &module);
        if (ret < 0) {
            if (unavailable(-ret) || ret == -ENOEXEC || ret == -EINVAL || ret == -EPROTONOSUPPORT ||
                ret == -EFBIG || ret == -ESTALE) { ret = 0; continue; }
            break;
        }
        if (manager->count == manager->maximum) { cc_ipa_module_close(module); ret = -E2BIG; break; }
        manager->modules[manager->count++] = module;
    }
done:
    for (size_t i = 0; i < files.count; ++i) free(files.paths[i]);
    free(files.paths);
    return ret;
}
void cc_ipa_manager_close(cc_ipa_manager *manager)
{
    if (!manager) return;
    for (size_t i = 0; i < manager->count; ++i) cc_ipa_module_close(manager->modules[i]);
    free(manager->modules); free(manager);
}
int cc_ipa_manager_open_environment(const cc_ipa_manager_options *options, const char *environment,
                                   bool privileged, const char *installed, const char *build, cc_ipa_manager **out)
{
    if (!out) return -EINVAL;
    *out = NULL;
    struct search search = {0};
    int ret = search_paths(options, environment, privileged, installed, build, &search);
    if (ret < 0) { free_paths(&search); return ret; }
    cc_ipa_manager *manager = calloc(1, sizeof(*manager));
    if (!manager) { free_paths(&search); return -ENOMEM; }
    manager->maximum = options && options->max_modules ? options->max_modules : CC_IPA_MANAGER_MAX_MODULES;
    manager->modules = calloc(manager->maximum, sizeof(*manager->modules));
    if (!manager->modules) { ret = -ENOMEM; goto done; }
    for (size_t i = 0; i < search.count; ++i) {
        ret = add_directory(manager, &search, &search.directories[i]);
        if (ret < 0) break;
    }
done:
    free_paths(&search);
    if (ret < 0) { cc_ipa_manager_close(manager); return ret; }
    *out = manager; return 0;
}
int cc_ipa_manager_open(const cc_ipa_manager_options *options, cc_ipa_manager **out)
{
    if (!out) return -EINVAL;
    *out = NULL;
    int validation = options_valid(options);
    if (validation < 0) return validation;
    bool privileged = getauxval(AT_SECURE) != 0 || getuid() != geteuid() || getgid() != getegid();
    char *root = NULL, *build = NULL;
    int ret = 0;
    if (!options || (options->flags & CC_IPA_MANAGER_BUILD_TREE)) {
        ret = cc_build_root(&root);
        if (ret < 0) return ret;
        if (root) { build = join(root, "ipa"); free(root); if (!build) return -ENOMEM; }
    }
    ret = cc_ipa_manager_open_environment(options, getenv("CCAMERA_IPA_MODULE_PATH"), privileged, NULL, build, out);
    free(build);
    return ret;
}
size_t cc_ipa_manager_count(const cc_ipa_manager *manager) { return manager ? manager->count : 0; }
const cc_ipa_module_info *cc_ipa_manager_get_info(const cc_ipa_manager *manager, size_t index)
{
    return manager && index < manager->count ? cc_ipa_module_get_info(manager->modules[index]) : NULL;
}
const char *cc_ipa_manager_get_path(const cc_ipa_manager *manager, size_t index)
{
    return manager && index < manager->count ? cc_ipa_module_get_path(manager->modules[index]) : NULL;
}
int cc_ipa_manager_get_signature(const cc_ipa_manager *manager, size_t index, const void **data, size_t *size)
{
    if (!manager || index >= manager->count) return -EINVAL;
    return cc_ipa_module_get_signature(manager->modules[index], data, size);
}
int cc_ipa_manager_select(const cc_ipa_manager *manager, const char *pipeline, uint32_t minimum,
                          uint32_t maximum, const char *name, size_t *index)
{
    if (!manager || !pipeline || !pipeline[0] || minimum > maximum || (name && !name[0]) || !index) return -EINVAL;
    for (size_t i = 0; i < manager->count; ++i) {
        const cc_ipa_module_info *info = cc_ipa_module_get_info(manager->modules[i]);
        if (cc_ipa_module_match(manager->modules[i], pipeline, minimum, maximum) == 1 &&
            (!name || !strcmp(info->name, name))) { *index = i; return 0; }
    }
    return -ENOENT;
}
int cc_ipa_manager_create(cc_ipa_manager *manager, const char *pipeline, uint32_t minimum,
                          uint32_t maximum, const char *name, const void *data, size_t size, cc_ipa **out)
{
    if (!out) return -EINVAL;
    *out = NULL;
    if (size && !data) return -EINVAL;
    size_t index;
    int ret = cc_ipa_manager_select(manager, pipeline, minimum, maximum, name, &index);
    if (ret < 0) return ret;
    return cc_ipa_module_create(manager->modules[index], data, size, out);
}
int cc_ipa_manager_create_proxy(cc_ipa_manager *manager, const char *pipeline, uint32_t minimum,
                                uint32_t maximum, const char *name, const void *data, size_t size,
                                const cc_ipa_proxy_options *options, cc_ipa **out)
{
    if (!out) return -EINVAL;
    *out = NULL;
    if (size && !data) return -EINVAL;
    size_t index;
    int ret = cc_ipa_manager_select(manager, pipeline, minimum, maximum, name, &index);
    if (ret < 0) return ret;
    return cc_ipa_module_create_proxy(manager->modules[index], data, size, options, out);
}
