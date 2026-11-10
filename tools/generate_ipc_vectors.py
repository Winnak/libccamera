#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
"""Compile unmodified archived IPC send/message methods with recording syscalls."""
import hashlib
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile

SHA = "ce4d60131c5fd218153f99cf1271375271815cc183a9ac7b270f18d3588015d5"
archive = Path(sys.argv[1])
if hashlib.sha256(archive.read_bytes()).hexdigest() != SHA:
    raise SystemExit("Reference archive SHA256 mismatch")


def method(source, signature):
    start = source.index(signature)
    opening = source.index("{", start)
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


with tarfile.open(archive) as tar:
    socket = tar.extractfile("libcamera/src/libcamera/ipc_unixsocket.cpp").read().decode()
    pipe = tar.extractfile("libcamera/src/libcamera/ipc_pipe.cpp").read().decode()
methods = "\n".join([
    method(socket, "int IPCUnixSocket::send(const Payload &payload)"),
    method(socket, "int IPCUnixSocket::sendData("),
    method(pipe, "IPCUnixSocket::Payload IPCMessage::payload() const"),
])
program = r'''
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <sys/socket.h>
#include <vector>
struct Sink { template<typename T> Sink &operator<<(const T &) { return *this; } };
#define LOG(...) Sink{}
class IPCUnixSocket {
    struct FD { int get() const { return 42; } } fd_;
    struct Header { uint32_t data; uint8_t fds; };
    int sendData(const void *, size_t, const int32_t *, unsigned);
public:
    struct Payload { std::vector<uint8_t> data; std::vector<int32_t> fds; };
    bool isBound() const { return true; }
    int send(const Payload &);
};
struct SharedFD { int value; int get() const { return value; } };
class IPCMessage {
public:
    struct Header { uint32_t cmd, cookie; } header_;
    std::vector<uint8_t> data_; std::vector<SharedFD> fds_;
    IPCUnixSocket::Payload payload() const;
};
static std::vector<uint8_t> wire_header, wire_body;
static std::vector<int> wire_fds;
extern "C" ssize_t __wrap_send(int, const void *data, size_t size, int) {
    wire_header.assign(static_cast<const uint8_t *>(data), static_cast<const uint8_t *>(data) + size);
    return size;
}
extern "C" ssize_t __wrap_sendmsg(int, const struct msghdr *message, int) {
    wire_body.clear(); wire_fds.clear();
    for (size_t i = 0; i < message->msg_iovlen; ++i) {
        const auto &v = message->msg_iov[i];
        const auto *p = static_cast<const uint8_t *>(v.iov_base);
        if (v.iov_len) wire_body.insert(wire_body.end(), p, p + v.iov_len);
    }
    for (auto *c = CMSG_FIRSTHDR(message); c; c = CMSG_NXTHDR(const_cast<msghdr *>(message), c)) {
        if (c->cmsg_level != SOL_SOCKET || c->cmsg_type != SCM_RIGHTS) return -1;
        size_t count = (c->cmsg_len - CMSG_LEN(0)) / sizeof(int);
        const int *p = reinterpret_cast<const int *>(CMSG_DATA(c));
        wire_fds.insert(wire_fds.end(), p, p + count);
    }
    return wire_body.size();
}
''' + methods + r'''
static void run(bool typed, uint32_t command, uint32_t cookie, unsigned size, unsigned fds) {
    wire_header.clear(); wire_body.clear(); wire_fds.clear();
    std::vector<uint8_t> data(size);
    for (unsigned i = 0; i < size; ++i) data[i] = static_cast<uint8_t>(i * 17 + 3);
    IPCUnixSocket::Payload payload;
    if (typed) {
        IPCMessage message; message.header_ = {command, cookie}; message.data_ = data;
        for (unsigned i = 0; i < fds; ++i) message.fds_.push_back({static_cast<int>(123 + i)});
        payload = message.payload();
    } else {
        payload.data = data;
        for (unsigned i = 0; i < fds; ++i) payload.fds.push_back(123 + i);
    }
    IPCUnixSocket socket; int status = socket.send(payload);
    uint64_t hash = UINT64_C(14695981039346656037);
    size_t prefix = typed ? 8 : 0;
    for (size_t i = prefix; i < wire_body.size(); ++i) { hash ^= wire_body[i]; hash *= UINT64_C(1099511628211); }
    printf("    {%u,0x%08xU,0x%08xU,%u,%u,%d,UINT64_C(0x%016llx),{", typed, command, cookie, size, fds, status,
           static_cast<unsigned long long>(hash));
    for (unsigned i = 0; i < 8; ++i) printf("%s%u", i ? "," : "", i < wire_header.size() ? wire_header[i] : 0);
    printf("},{");
    for (unsigned i = 0; i < 8; ++i) printf("%s%u", i ? "," : "", typed && i < wire_body.size() ? wire_body[i] : 0);
    printf("}},\n");
}
int main() {
    for (unsigned size : {0U,1U,8U,64U,65536U}) for (unsigned fds : {0U,1U,4U,253U}) run(false,0,0,size,fds);
    for (uint32_t command : {0U,1U,UINT32_MAX}) for (uint32_t cookie : {0U,42U,UINT32_MAX})
    for (unsigned size : {0U,1U,13U,4096U,65528U}) for (unsigned fds : {0U,1U,4U}) run(true,command,cookie,size,fds);
}
'''
with tempfile.TemporaryDirectory() as directory:
    root = Path(directory)
    (root / "vectors.cpp").write_text(program)
    subprocess.run(["c++", "-std=c++17", str(root / "vectors.cpp"), "-Wl,--wrap=send", "-Wl,--wrap=sendmsg",
                    "-o", str(root / "vectors")], check=True)
    output = subprocess.check_output([str(root / "vectors")], text=True)
destination = Path(__file__).resolve().parents[1] / "tests/ipc_vectors.h"
destination.write_text("/* SPDX-License-Identifier: LGPL-2.1-or-later */\n"
    "/* Compiled, unmodified archived IPCUnixSocket::send/sendData and\n"
    " * IPCMessage::payload with recording syscalls. Native fields generated on\n"
    " * a little-endian host; tests normalize their byte order. Reference SHA256:\n"
    f" * {SHA}. */\nstatic const struct ipc_vector ipc_vectors[]={{\n" + output + "};\n")
print(f"Wrote {len(output.splitlines())} IPC wire fixtures to {destination}")
