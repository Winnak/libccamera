# SPDX-License-Identifier: CC0-1.0
# The Arm GNU Linux toolchain supplies its own headers and libc. Put its bin
# directory on PATH; no target filesystem or extra libraries are needed.
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER aarch64-none-linux-gnu-gcc)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
