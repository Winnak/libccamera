#!/usr/bin/env python3
# SPDX-License-Identifier: CC0-1.0
"""Native packaging tests; signing is always performed by a host executable."""
import hashlib
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile

source, build, openssl, compiler = map(Path, sys.argv[1:5])
root = Path(tempfile.mkdtemp(prefix="ipa-crypto-build-", dir=build))
cmake = shutil.which("cmake")
ninja = shutil.which("ninja")


def run(*args, expected=0, env=None):
    result = subprocess.run([str(a) for a in args], text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, env=env, timeout=120)
    if result.returncode != expected:
        raise AssertionError(f"{args}: expected {expected}, got {result.returncode}\n{result.stdout}")
    return result.stdout


def configure(directory, *args):
    return run(cmake, "-S", source, "-B", directory, "-G", "Ninja",
               f"-DCMAKE_C_COMPILER={compiler}", "-DCCAMERA_BUILD_TESTS=OFF",
               "-DCCAMERA_BUILD_EXAMPLES=OFF", *args)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def public(key, path):
    run(openssl, "pkey", "-in", key, "-pubout", "-out", path)


def cli(module, key, expected=0):
    run(openssl, "dgst", "-sha256", "-verify", key, "-signature", str(module) + ".sign", module, expected=expected)


try:
    # Disabled mode must never discover OpenSSL or create any key.
    disabled = root / "disabled"
    configure(disabled, "-DCCAMERA_WITH_IPA_CRYPTO=OFF", "-DCMAKE_DISABLE_FIND_PACKAGE_OpenSSL=TRUE")
    assert not (disabled / "ipa-keys").exists()
    for options in (("-DCCAMERA_IPA_PRIVATE_KEY=missing",), ("-DCCAMERA_IPA_PUBLIC_KEY=missing",)):
        run(cmake, "-S", source, "-B", root / "reject-off", *options, expected=1)
    native = root / "native"
    configure(native, "-DCCAMERA_WITH_IPA_CRYPTO=ON")
    key = native / "ipa-keys/development.pem"
    original = key.read_bytes()
    assert key.stat().st_mode & 0o777 == 0o600
    configure(native, "-DCCAMERA_WITH_IPA_CRYPTO=ON")
    assert key.read_bytes() == original
    old_header = (native / "ipa-crypto/cc_ipa_public_key.h").read_bytes()
    new_key = root / "replacement.pem"
    run(openssl, "genpkey", "-algorithm", "RSA", "-pkeyopt", "rsa_keygen_bits:2048", "-out", new_key)
    shutil.copyfile(new_key, key)
    # Building must notice key replacement and reconfigure the embedded root.
    run(cmake, "--build", native, "-j", "4")
    assert (native / "ipa-crypto/cc_ipa_public_key.h").read_bytes() != old_header
    pub = root / "public.pem"
    public(key, pub)
    public_only = root / "public-only"
    configure(public_only, "-DCCAMERA_WITH_IPA_CRYPTO=ON", f"-DCCAMERA_IPA_PUBLIC_KEY={pub}")
    assert not (public_only / "ipa-keys/development.pem").exists()
    assert (public_only / "ipa-crypto/cc_ipa_public_key.h").read_bytes() == (native / "ipa-crypto/cc_ipa_public_key.h").read_bytes()
    run(cmake, "--build", public_only, "-j", "4")
    # Reject missing, encrypted, EC and restricted RSA-PSS keys without prompts.
    ec = root / "ec.pem"
    pss = root / "pss.pem"
    encrypted = root / "encrypted.pem"
    run(openssl, "genpkey", "-algorithm", "EC", "-pkeyopt", "ec_paramgen_curve:P-256", "-out", ec)
    run(openssl, "genpkey", "-algorithm", "RSA-PSS", "-pkeyopt", "rsa_keygen_bits:2048", "-out", pss)
    run(openssl, "pkey", "-in", key, "-aes-256-cbc", "-passout", "pass:test-password", "-out", encrypted)
    for bad in (ec, pss, encrypted, root / "missing.pem"):
        run(cmake, "-S", source, "-B", root / "reject-key", "-DCCAMERA_WITH_IPA_CRYPTO=ON",
            f"-DCCAMERA_IPA_PRIVATE_KEY={bad}", expected=1)
    run(cmake, "-S", source, "-B", root / "reject-both", "-DCCAMERA_WITH_IPA_CRYPTO=ON",
        f"-DCCAMERA_IPA_PRIVATE_KEY={key}", f"-DCCAMERA_IPA_PUBLIC_KEY={pub}", expected=1)
    prefix = root / "package"
    run(cmake, "--install", native, "--prefix", prefix)
    # Installed exports contain no private key material or build-tree paths.
    for path in prefix.rglob("*"):
        if path.is_file():
            content = path.read_bytes()
            assert b"PRIVATE KEY-----" not in content, path
            if path.suffix in (".cmake", ".pc"):
                assert str(native).encode() not in content, path
    signing_source = root / "signing-source"
    signing_source.mkdir()
    (signing_source / "CMakeLists.txt").write_text('''cmake_minimum_required(VERSION 3.18)
project(signing_consumer C)
find_package(ccamera CONFIG REQUIRED)
add_library(ipa MODULE "${FIXTURE}")
target_include_directories(ipa PRIVATE "${HEADERS}")
set_target_properties(ipa PROPERTIES BUILD_RPATH "/build-only-path" INSTALL_RPATH "$ORIGIN/deps")
ccamera_sign_ipa(TARGET ipa PRIVATE_KEY "${KEY}" INSTALL_DESTINATION "${DESTINATION}" COMPONENT ipa)
add_executable(consumer "${CONSUMER}")
target_link_libraries(consumer PRIVATE ccamera::ccamera)
''')
    # An installed consumer cannot silently reuse a build's private key.
    no_key_source = root / "no-key-source"
    no_key_source.mkdir()
    (no_key_source / "CMakeLists.txt").write_text('''cmake_minimum_required(VERSION 3.18)
project(no_key C)
find_package(ccamera CONFIG REQUIRED)
add_library(ipa MODULE dummy.c)
ccamera_sign_ipa(TARGET ipa)
''')
    (no_key_source / "dummy.c").write_text("int dummy;\n")
    run(cmake, "-S", no_key_source, "-B", root / "no-key-build", f"-DCMAKE_PREFIX_PATH={prefix}", expected=1)
    for generator in ("Ninja", "Ninja Multi-Config"):
        directory = root / generator.replace(" ", "-")
        run(cmake, "-S", signing_source, "-B", directory, "-G", generator,
            f"-DCMAKE_PREFIX_PATH={prefix}", f"-DKEY={key}", "-DDESTINATION=lib/ipa",
            f"-DFIXTURE={source / 'tests/ipa_module_fixture.c'}", f"-DHEADERS={source / 'include'}",
            f"-DCONSUMER={source / 'tests/consumer/main.c'}")
        configs = ("Debug", "Release") if generator.endswith("Multi-Config") else ("",)
        for config in configs:
            run(cmake, "--build", directory, "--config", config, "-j", "4")
            module = directory / config / "libipa.so"
            cli(module, pub)
            run(native / "ccamera-ipa-verify", module)
            run(public_only / "ccamera-ipa-verify", module)
            run(directory / config / "consumer")
            # Key-only replacement must update the signature without relinking.
            before = digest(module)
            key_original = key.read_bytes()
            replacement = root / "second-rsa.pem"
            run(openssl, "genpkey", "-algorithm", "RSA", "-pkeyopt", "rsa_keygen_bits:2048", "-out", replacement)
            shutil.copyfile(replacement, key)
            run(cmake, "--build", directory, "--config", config, "-j", "4")
            assert digest(module) == before
            new_pub = root / "second-public.pem"
            public(key, new_pub)
            cli(module, new_pub)
            run(native / "ccamera-ipa-verify", module, expected=1)
            key.write_bytes(key_original)
            run(cmake, "--build", directory, "--config", config, "-j", "4")
            cli(module, pub)
            destdir = root / ("stage-" + generator.replace(" ", "-") + config)
            environment = dict(os.environ, DESTDIR=str(destdir))
            # --prefix, DESTDIR, component, strip and RPATH changes all apply.
            run(cmake, "--install", directory, "--config", config, "--prefix", "/overridden",
                "--component", "ipa", "--strip", env=environment)
            installed = destdir / "overridden/lib/ipa/libipa.so"
            assert digest(installed) != before
            cli(installed, pub)
            run(native / "ccamera-ipa-verify", installed)
            run(public_only / "ccamera-ipa-verify", installed)
            assert "$ORIGIN/deps" in run("readelf", "-d", installed)
            assert "/build-only-path" not in run("readelf", "-d", installed)
            signature = Path(str(installed) + ".sign")
            signature.unlink()
            run(native / "ccamera-ipa-verify", installed, expected=1)
            # Installation signing errors must be fatal.
            key.write_text("not a key")
            run(cmake, "--install", directory, "--config", config, "--prefix", "/overridden",
                "--component", "ipa", env=environment, expected=1)
            key.write_bytes(key_original)
    run(native / "ccamera-ipa-verify", expected=2)
    run(native / "ccamera-ipa-verify", root / "absent.so", expected=2)
    # pkg-config must carry OpenSSL's own private dependencies for static links.
    pkgconfig = shutil.which("pkg-config")
    if pkgconfig:
        environment = dict(os.environ, PKG_CONFIG_PATH=str(next(prefix.rglob("ccamera.pc")).parent))
        flags = shlex.split(run(pkgconfig, "--static", "--cflags", "--libs", "ccamera", env=environment))
        assert "-lcrypto" in flags
        crypto_flags = shlex.split(run(pkgconfig, "--static", "--libs", "libcrypto", env=environment))
        assert all(flag in flags for flag in crypto_flags)
        run(compiler, source / "tests/consumer/main.c", "-o", root / "pkg-consumer", *flags)
        run(root / "pkg-consumer")
        # Resolve a fully static libcrypto, including zlib/dl/pthread, if installed.
        archive = Path(run(compiler, "-print-file-name=libcrypto.a").strip())
        if archive.is_file():
            run(compiler, "-static", source / "tests/consumer/main.c", "-o", root / "pkg-consumer-static", *flags)
            run(root / "pkg-consumer-static")
    # Absolute install destinations still honor DESTDIR and ignore --prefix.
    absolute = root / "absolute-build"
    run(cmake, "-S", signing_source, "-B", absolute, "-G", "Ninja",
        f"-DCMAKE_PREFIX_PATH={prefix}", f"-DKEY={key}", "-DDESTINATION=/absolute/ipa",
        f"-DFIXTURE={source / 'tests/ipa_module_fixture.c'}", f"-DHEADERS={source / 'include'}",
        f"-DCONSUMER={source / 'tests/consumer/main.c'}")
    run(cmake, "--build", absolute, "--target", "ipa-ipa-sign")
    absolute_stage = root / "absolute-stage"
    run(cmake, "--install", absolute, "--component", "unrelated", env=dict(os.environ, DESTDIR=str(absolute_stage)))
    assert not (absolute_stage / "absolute/ipa/libipa.so").exists()
    run(cmake, "--install", absolute, "--component", "ipa", "--prefix", "/unused-prefix",
        env=dict(os.environ, DESTDIR=str(absolute_stage)))
    cli(absolute_stage / "absolute/ipa/libipa.so", pub)
    # Cross signing must find the HOST tool even with target-only program lookup.
    cross_source = root / "cross-source"
    cross_source.mkdir()
    target_root = root / "target-root"
    (target_root / "usr/bin").mkdir(parents=True)
    poison = target_root / "usr/bin/openssl"
    poison.write_text("#!/bin/sh\necho TARGET_OPENSSL_MUST_NOT_RUN >&2\nexit 99\n")
    poison.chmod(0o755)
    (cross_source / "CMakeLists.txt").write_text(f'''cmake_minimum_required(VERSION 3.18)
project(cross_sign C)
include("{prefix / 'lib/cmake/ccamera/ccameraIpaSigning.cmake'}")
add_library(ipa MODULE dummy.c)
ccamera_sign_ipa(TARGET ipa PRIVATE_KEY "${{KEY}}")
''')
    # GNUInstallDirs may choose lib64.
    helper = next(prefix.rglob("ccameraIpaSigning.cmake"))
    text = (cross_source / "CMakeLists.txt").read_text().replace(str(prefix / "lib/cmake/ccamera/ccameraIpaSigning.cmake"), str(helper))
    (cross_source / "CMakeLists.txt").write_text(text)
    (cross_source / "dummy.c").write_text("int dummy;\n")
    cross_compiler = shutil.which("aarch64-none-linux-gnu-gcc") or shutil.which("aarch64-linux-gnu-gcc") or str(compiler)
    cross = root / "cross"
    run(cmake, "-S", cross_source, "-B", cross, "-G", "Ninja", "-DCMAKE_SYSTEM_NAME=Linux",
        "-DCMAKE_SYSTEM_PROCESSOR=aarch64", f"-DCMAKE_C_COMPILER={cross_compiler}", f"-DCMAKE_MAKE_PROGRAM={ninja}",
        f"-DCMAKE_FIND_ROOT_PATH={target_root}", "-DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=ONLY", f"-DKEY={key}")
    run(cmake, "--build", cross)
    cli(cross / "libipa.so", pub)
    print("IPA key lifecycle, unsupported keys, public-only builds, exported signing, installation and static consumers passed")
except Exception:
    print(f"Retained failing build test in {root}", file=sys.stderr)
    raise
else:
    shutil.rmtree(root)
