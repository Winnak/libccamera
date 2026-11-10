#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
"""Compile archived directory/selection policies; module metadata is a text stub.

The extracted parseDir/addDir/module/match methods are unmodified. The fixture
scope is traversal, sorting, precedence and first-match selection, not ELF or
dynamic linking (those are covered using actual compiled C modules).
"""
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
    depth = 1
    end = opening + 1
    while depth:
        if source[end] == "{":
            depth += 1
        elif source[end] == "}":
            depth -= 1
        end += 1
    return source[start:end]


with tarfile.open(archive) as tar:
    def read(suffix):
        member = next(m for m in tar.getmembers() if m.name.endswith("/" + suffix))
        return tar.extractfile(member).read().decode()

    manager = read("src/libcamera/ipa_manager.cpp")
    module = read("src/libcamera/ipa_module.cpp")
policies = "\n".join([
    method(manager, "void IPAManager::parseDir("),
    method(manager, "unsigned int IPAManager::addDir("),
    method(manager, "IPAModule *IPAManager::module("),
    method(module, "bool IPAModule::match("),
])
program = r'''
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
struct Sink { template<typename T> Sink &operator<<(const T &) { return *this; } };
#define LOG(...) Sink{}
class PipelineHandler {
public: const char *name() const { return "TestPipeline"; }
};
class IPAModule {
    struct { uint32_t pipelineVersion = 0; char pipelineName[256] = {}; } info_;
    std::string path_; bool valid_ = false;
public:
    explicit IPAModule(const std::string &path) : path_(path) {
        std::ifstream input(path); std::string name;
        if (input >> name >> info_.pipelineVersion) {
            std::strncpy(info_.pipelineName, name.c_str(), sizeof(info_.pipelineName) - 1);
            valid_ = true;
        }
    }
    bool isValid() const { return valid_; }
    const std::string &path() const { return path_; }
    bool match(PipelineHandler *, uint32_t, uint32_t) const;
};
class IPAManager {
    void parseDir(const char *, unsigned, std::vector<std::string> &);
    std::vector<std::unique_ptr<IPAModule>> modules_;
public:
    unsigned addDir(const char *, unsigned = 0);
    IPAModule *module(PipelineHandler *, uint32_t, uint32_t);
    size_t count() const { return modules_.size(); }
};
''' + policies + r'''
int main(int argc, char **argv) {
    if (argc != 2) return 1;
    std::string root = argv[1]; PipelineHandler pipe;
    const uint32_t minimum[] = {0, 7, 8, 4, 10}, maximum[] = {UINT32_MAX, 7, 9, 6, 11};
    for (unsigned order = 0; order < 4; ++order) for (unsigned depth = 0; depth < 3; ++depth)
    for (unsigned interval = 0; interval < 5; ++interval) {
        IPAManager manager;
        if (order == 0 || order == 2) manager.addDir((root + "/A").c_str(), depth);
        if (order == 1 || order == 2 || order == 3) manager.addDir((root + "/B").c_str(), depth);
        if (order == 3) manager.addDir((root + "/A").c_str(), depth);
        IPAModule *module = manager.module(&pipe, minimum[interval], maximum[interval]);
        std::string path = module ? module->path().substr(root.size() + 1) : "";
        printf("%u %u %u %u %zu %s\n", order, depth, minimum[interval], maximum[interval],
               manager.count(), module ? path.c_str() : "-");
    }
}
'''
with tempfile.TemporaryDirectory() as directory:
    root = Path(directory)
    for path, version in [
        ("A/z-last.so", 7), ("A/a-first.so", 4), ("A/00-bad.so", None),
        ("A/ignored.so.1", 9), ("A/nested/b-middle.so", 9),
        ("A/nested/deeper/c-deep.so", 7), ("A/fake.so/d.so", 9),
        ("B/0-override.so", 9), ("B/a-other.so", 4),
    ]:
        file = root / path
        file.parent.mkdir(parents=True, exist_ok=True)
        file.write_text(f"TestPipeline {version}" if version else "invalid")
    (root / "A/alias.so").symlink_to("z-last.so")
    (root / "A/linked-directory").symlink_to("nested", target_is_directory=True)
    (root / "vectors.cpp").write_text(program)
    subprocess.run(["c++", "-std=c++17", str(root / "vectors.cpp"), "-o", str(root / "vectors")], check=True)
    output = subprocess.check_output([str(root / "vectors"), str(root)], text=True)
    rows = []
    for line in output.splitlines():
        order, depth, minimum, maximum, count, path = line.split()
        selected = "NULL" if path == "-" else '"' + (root / path).resolve().relative_to(root).as_posix() + '"'
        rows.append(f"    {{{order},{depth},{minimum}U,{maximum}U,{count},{selected}}},\n")
destination = Path(__file__).resolve().parents[1] / "tests/ipa_manager_vectors.h"
destination.write_text("/* SPDX-License-Identifier: LGPL-2.1-or-later */\n"
    "/* Generated from compiled, unmodified archived parseDir/addDir/module/match\n"
    " * methods, with text metadata stubs. Reference SHA256:\n"
    f" * {SHA}. */\n"
    "static const struct manager_vector manager_vectors[]={\n" + "".join(rows) + "};\n")
print(f"Wrote {len(rows)} IPA discovery fixtures to {destination}")
