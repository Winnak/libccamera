/* SPDX-License-Identifier: LGPL-2.1-or-later */
#define _XOPEN_SOURCE 700
#include "ipa_module.h"
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>
#ifdef CCAMERA_WITH_IPA_CRYPTO
#include <openssl/evp.h>
#include <openssl/rsa.h>
#include <openssl/x509.h>
#include "cc_ipa_public_key.h"
#endif

struct cc_ipa_module {
    char *path;
    int fd;
    struct stat snapshot;
    cc_ipa_module_info info;
    void *signature;
    size_t signature_size, references;
    bool factory, signature_present;
    void *handle;
    cc_ipa_module_factory create;
};
static bool same_file(const struct stat *a, const struct stat *b)
{
    return a->st_dev == b->st_dev && a->st_ino == b->st_ino && a->st_size == b->st_size &&
        a->st_mtim.tv_sec == b->st_mtim.tv_sec && a->st_mtim.tv_nsec == b->st_mtim.tv_nsec &&
        a->st_ctim.tv_sec == b->st_ctim.tv_sec && a->st_ctim.tv_nsec == b->st_ctim.tv_nsec;
}
static int read_file(int fd, size_t maximum, void **out, size_t *size, struct stat *snapshot)
{
    struct stat status;
    if (fstat(fd, &status)) return -errno;
    if (!S_ISREG(status.st_mode) || status.st_size < 0) return -EINVAL;
    if ((uint64_t)status.st_size > maximum) return -EFBIG;
    size_t length = (size_t)status.st_size;
    unsigned char *data = malloc(length ? length : 1);
    if (!data) return -ENOMEM;
    size_t offset = 0;
    int ret = 0;
    while (offset < length) {
        ssize_t n = pread(fd, data + offset, length - offset, (off_t)offset);
        if (n < 0) { if (errno == EINTR) continue; ret = -errno; break; }
        if (!n) { ret = -ESTALE; break; }
        offset += (size_t)n;
    }
    struct stat after;
    if (!ret && fstat(fd, &after)) ret = -errno;
    if (!ret && !same_file(&status, &after)) ret = -ESTALE;
    if (ret) { free(data); return ret; }
    *out = data; *size = length;
    if (snapshot) *snapshot = status;
    return 0;
}
static int read_signature(cc_ipa_module *m)
{
    size_t length = strlen(m->path);
    if (length > SIZE_MAX - 6) return -EOVERFLOW;
    char *path = malloc(length + 6);
    if (!path) return -ENOMEM;
    memcpy(path, m->path, length);
    memcpy(path + length, ".sign", 6);
    int fd = open(path, O_RDONLY | O_CLOEXEC | O_NONBLOCK);
    free(path);
    if (fd < 0) return 0; /* Signatures are optional, as in the reference. */
    int ret = read_file(fd, 1024U * 1024U, &m->signature, &m->signature_size, NULL);
    close(fd);
    if (!ret) m->signature_present = true;
    return ret == -ENOMEM || ret == -EFBIG ? ret : 0;
}
void cc_ipa_module_close(cc_ipa_module *m)
{
    if (!m || --m->references) return;
    if (m->handle) dlclose(m->handle);
    if (m->fd >= 0) close(m->fd);
    free(m->signature);
    free(m->path);
    free(m);
}
int cc_ipa_module_open(const char *path, cc_ipa_module **out)
{
    if (!out) return -EINVAL;
    *out = NULL;
    if (!path || !path[0]) return -EINVAL;
    cc_ipa_module *m = calloc(1, sizeof(*m));
    if (!m) return -ENOMEM;
    m->fd = -1; m->references = 1;
    m->path = realpath(path, NULL);
    int ret = 0;
    if (!m->path) { ret = -errno; goto fail; }
    m->fd = open(m->path, O_RDONLY | O_CLOEXEC | O_NONBLOCK);
    if (m->fd < 0) { ret = -errno; goto fail; }
    void *data = NULL; size_t size = 0;
    ret = read_file(m->fd, 64U * 1024U * 1024U, &data, &size, &m->snapshot);
    if (!ret) ret = cc_ipa_module_elf(data, size, &m->info, &m->factory);
    free(data);
    if (ret < 0) goto fail;
    ret = read_signature(m);
    if (ret < 0) goto fail;
    *out = m;
    return 0;
fail:
    cc_ipa_module_close(m);
    return ret;
}
const cc_ipa_module_info *cc_ipa_module_get_info(const cc_ipa_module *m) { return m ? &m->info : NULL; }
const char *cc_ipa_module_get_path(const cc_ipa_module *m) { return m ? m->path : NULL; }
int cc_ipa_module_match(const cc_ipa_module *m, const char *name, uint32_t minimum, uint32_t maximum)
{
    if (!m || !name || minimum > maximum) return -EINVAL;
    return m->info.pipeline_version >= minimum && m->info.pipeline_version <= maximum &&
        !strcmp(m->info.pipeline_name, name);
}
int cc_ipa_module_get_signature(const cc_ipa_module *m, const void **data, size_t *size)
{
    if (!m || !data || !size) return -EINVAL;
    *data = m->signature_size ? m->signature : NULL;
    *size = m->signature_size;
    return 0;
}
static int unchanged(const cc_ipa_module *m)
{
    struct stat file, path;
    if (fstat(m->fd, &file)) return -errno;
    if (!same_file(&m->snapshot, &file)) return -ESTALE;
    if (stat(m->path, &path))
        return errno == ENOENT || errno == ENOTDIR || errno == ELOOP ? -ESTALE : -errno;
    return same_file(&m->snapshot, &path) ? 0 : -ESTALE;
}
int cc_ipa_module_verify_signature(const cc_ipa_module *m, cc_ipa_signature_status *status)
{
    if (!m || !status) return -EINVAL;
    int ret = unchanged(m);
    if (ret < 0) return ret;
    cc_ipa_signature_status result = CC_IPA_SIGNATURE_UNAVAILABLE;
#ifdef CCAMERA_WITH_IPA_CRYPTO
    if (!m->signature_present) result = CC_IPA_SIGNATURE_MISSING;
    else {
        const unsigned char *der = cc_ipa_public_key;
        EVP_PKEY *key = d2i_PUBKEY(NULL, &der, (long)sizeof(cc_ipa_public_key));
        if (!key) return -EIO;
        EVP_MD_CTX *context = EVP_MD_CTX_new();
        if (!context) { EVP_PKEY_free(key); return -ENOMEM; }
        EVP_PKEY_CTX *key_context = NULL;
        if (!EVP_PKEY_is_a(key, "RSA") ||
            EVP_DigestVerifyInit(context, &key_context, EVP_sha256(), NULL, key) != 1 ||
            EVP_PKEY_CTX_set_rsa_padding(key_context, RSA_PKCS1_PADDING) <= 0) ret = -EIO;
        unsigned char bytes[16384];
        off_t offset = 0;
        while (!ret && offset < m->snapshot.st_size) {
            off_t remaining = m->snapshot.st_size - offset;
            size_t length = remaining < (off_t)sizeof(bytes) ? (size_t)remaining : sizeof(bytes);
            ssize_t count = pread(m->fd, bytes, length, offset);
            if (count < 0) { if (errno == EINTR) continue; ret = -errno; break; }
            if (!count) { ret = -ESTALE; break; }
            if (EVP_DigestVerifyUpdate(context, bytes, (size_t)count) != 1) ret = -EIO;
            offset += count;
        }
        if (!ret) {
            /* Reject malformed lengths before Final: providers may report
             * decoding errors as negative values for malformed RSA signatures. */
            int expected = EVP_PKEY_get_size(key);
            if (expected <= 0) ret = -EIO;
            else if (m->signature_size != (size_t)expected) result = CC_IPA_SIGNATURE_INVALID;
            else {
                int verified = EVP_DigestVerifyFinal(context, m->signature, m->signature_size);
                if (verified < 0) ret = -EIO;
                else result = verified == 1 ? CC_IPA_SIGNATURE_VALID : CC_IPA_SIGNATURE_INVALID;
            }
        }
        EVP_MD_CTX_free(context); EVP_PKEY_free(key);
    }
#endif
    /* Staleness takes precedence over a digest/signature failure. */
    int identity = unchanged(m);
    if (identity < 0) return identity;
    if (ret < 0) return ret;
    *status = result;
    return 0;
}
static struct cc_ipa_module_transfer transfer_stamp(const struct stat *status, const cc_ipa_module_info *info)
{
    struct cc_ipa_module_transfer transfer = {0};
    transfer.info = *info; transfer.device = status->st_dev; transfer.inode = status->st_ino;
    transfer.size = (uint64_t)status->st_size;
    transfer.mtime_seconds = status->st_mtim.tv_sec; transfer.ctime_seconds = status->st_ctim.tv_sec;
    transfer.mtime_nanoseconds = (int32_t)status->st_mtim.tv_nsec;
    transfer.ctime_nanoseconds = (int32_t)status->st_ctim.tv_nsec;
    return transfer;
}
static bool same_stamp(const struct cc_ipa_module_transfer *a, const struct cc_ipa_module_transfer *b)
{
    return !memcmp(&a->info, &b->info, sizeof(a->info)) && a->device == b->device && a->inode == b->inode &&
        a->size == b->size && a->mtime_seconds == b->mtime_seconds && a->ctime_seconds == b->ctime_seconds &&
        a->mtime_nanoseconds == b->mtime_nanoseconds && a->ctime_nanoseconds == b->ctime_nanoseconds;
}
int cc_ipa_module_export(cc_ipa_module *module, int *fd, struct cc_ipa_module_transfer *transfer)
{
    if (fd) *fd = -1;
    if (!module || !fd || !transfer) return -EINVAL;
    int ret = unchanged(module); if (ret < 0) return ret;
    if (!module->factory) return -ENOENT;
    int duplicate = fcntl(module->fd, F_DUPFD_CLOEXEC, 0); if (duplicate < 0) return -errno;
    *transfer = transfer_stamp(&module->snapshot, &module->info); *fd = duplicate; return 0;
}
int cc_ipa_module_import(const char *path, int fd, const struct cc_ipa_module_transfer *transfer, cc_ipa_module **out)
{
    if (!out) return -EINVAL;
    *out = NULL;
    if (!path || !path[0] || fd < 0 || !transfer) return -EINVAL;
    cc_ipa_module *module = calloc(1, sizeof(*module)); if (!module) return -ENOMEM;
    module->fd = -1; module->references = 1;
    module->path = strdup(path);
    int ret = module->path ? 0 : -ENOMEM;
    if (ret < 0) goto done;
    module->fd = fcntl(fd, F_DUPFD_CLOEXEC, 0);
    if (module->fd < 0) { ret = -errno; goto done; }
    void *data = NULL; size_t size = 0;
    ret = read_file(module->fd, 64U * 1024U * 1024U, &data, &size, &module->snapshot);
    if (!ret) ret = cc_ipa_module_elf(data, size, &module->info, &module->factory);
    free(data);
    if (!ret) {
        struct cc_ipa_module_transfer actual = transfer_stamp(&module->snapshot, &module->info);
        if (!same_stamp(&actual, transfer)) ret = -ESTALE;
    }
    if (!ret) ret = unchanged(module);
done:
    if (ret < 0) { cc_ipa_module_close(module); return ret; }
    *out = module; return 0;
}
static int loaded_identity(const cc_ipa_module *m, const void *symbol)
{
    /* dlopen may return an older object cached under this path even after an
     * atomic file replacement. Check the mapped inode, not just its metadata. */
    /* Btrfs subvolume st_dev values differ from /proc/maps' superblock device.
     * Compare against a read-only mapping of the retained fd, in the same
     * kernel namespace, instead of comparing maps directly with stat(). */
    void *probe = mmap(NULL, 1, PROT_READ, MAP_PRIVATE, m->fd, 0);
    if (probe == MAP_FAILED) return -errno;
    int fd = open("/proc/self/maps", O_RDONLY | O_CLOEXEC);
    if (fd < 0) { int ret = -errno; munmap(probe, 1); return ret; }
    FILE *file = fdopen(fd, "r");
    if (!file) { int ret = -errno; close(fd); munmap(probe, 1); return ret; }
    char *line = NULL; size_t capacity = 0;
    int ret = -ESTALE;
    struct mapping { bool found; unsigned int major, minor; unsigned long long inode; } mappings[2] = {{0}};
    const uintptr_t addresses[2] = {(uintptr_t)symbol, (uintptr_t)probe};
    while (getline(&line, &capacity, file) >= 0) {
        unsigned long long start, end, offset, inode;
        unsigned int device_major, device_minor;
        char permissions[5];
        if (sscanf(line, "%llx-%llx %4s %llx %x:%x %llu", &start, &end, permissions,
                   &offset, &device_major, &device_minor, &inode) != 7) continue;
        for (size_t i = 0; i < 2; ++i)
            if (addresses[i] >= start && addresses[i] < end)
                mappings[i] = (struct mapping){true, device_major, device_minor, inode};
        if (mappings[0].found && mappings[1].found) {
            ret = mappings[0].major == mappings[1].major && mappings[0].minor == mappings[1].minor &&
                mappings[0].inode == mappings[1].inode ? 0 : -ESTALE;
            break;
        }
    }
    if (ferror(file)) ret = errno ? -errno : -EIO;
    free(line); fclose(file); munmap(probe, 1);
    return ret;
}
static int load_module(cc_ipa_module *m)
{
    if (m->handle) return 0;
    if (!m->factory) return -ENOENT;
    int ret = unchanged(m);
    if (ret < 0) return ret;
    void *handle = dlopen(m->path, RTLD_NOW | RTLD_LOCAL);
    if (!handle) return -ENOEXEC;
    const cc_ipa_module_info *info = dlsym(handle, "cc_ipa_module_descriptor");
    void *symbol = dlsym(handle, "cc_ipa_module_create_v1");
    ret = unchanged(m);
    if (!ret && (!info || memcmp(info, &m->info, sizeof(m->info)))) ret = -ESTALE;
    if (!ret) ret = loaded_identity(m, info);
    if (!ret && !symbol) ret = -ENOENT;
    if (ret < 0) { dlclose(handle); return ret; }
    _Static_assert(sizeof(m->create) == sizeof(symbol), "POSIX dlsym pointer representation");
    memcpy(&m->create, &symbol, sizeof(symbol));
    m->handle = handle;
    return 0;
}
static void release_module(void *module) { cc_ipa_module_close(module); }
int cc_ipa_module_create(cc_ipa_module *m, const void *data, size_t size, cc_ipa **out)
{
    if (!out) return -EINVAL;
    *out = NULL;
    if (!m || (size && !data)) return -EINVAL;
    if (m->references == SIZE_MAX) return -EOVERFLOW;
    int ret = load_module(m);
    if (ret < 0) return ret;
    cc_ipa_ops ops = {0}; void *context = NULL;
    ret = m->create(CC_IPA_ABI_VERSION, sizeof(ops), data, size, &ops, &context);
    if (ret) return ret < 0 ? ret : -EPROTO;
    ret = cc_ipa_create(&ops, context, out);
    if (ret < 0) { if (ops.destroy) ops.destroy(context); return ret; }
    ++m->references;
    cc_ipa_set_owner(*out, m, release_module);
    return 0;
}
