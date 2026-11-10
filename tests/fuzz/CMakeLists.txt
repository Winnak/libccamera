if(NOT CMAKE_C_COMPILER_ID STREQUAL "Clang")
    message(FATAL_ERROR "CCAMERA_BUILD_FUZZERS requires Clang with libFuzzer, ASan and UBSan; configure a fresh build with -DCMAKE_C_COMPILER=clang")
endif()

include(CheckCSourceCompiles)
include(CMakePushCheckState)
cmake_push_check_state(RESET)
set(CMAKE_REQUIRED_FLAGS "-fsanitize=fuzzer,address,undefined")
check_c_source_compiles("#include <stddef.h>
    #include <stdint.h>
    int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
        (void)data; (void)size; return 0;
    }" CCAMERA_HAS_LIBFUZZER)
cmake_pop_check_state()
if(NOT CCAMERA_HAS_LIBFUZZER)
    message(FATAL_ERROR "Clang's libFuzzer, ASan and UBSan runtimes are required (install the matching compiler-rt development package)")
endif()

# Instrument the library itself, while leaving the installed ccamera target and
# its consumers free of sanitizer/runtime dependencies.
list(TRANSFORM CCAMERA_SOURCES PREPEND "${libccamera_SOURCE_DIR}/")
add_library(ccamera-fuzz-support STATIC ${CCAMERA_SOURCES})
ccamera_compile_settings(ccamera-fuzz-support)
target_include_directories(ccamera-fuzz-support PUBLIC "${libccamera_SOURCE_DIR}/include")
target_compile_options(ccamera-fuzz-support PRIVATE
    -g -O1 -fno-omit-frame-pointer -fno-sanitize-recover=all
    -fsanitize=fuzzer-no-link,address,undefined)

foreach(name pisp cfe sensor delayed stream controls media ipa ipa_module ipa_manager ipc ipa_wire af embedded routing buffers fences uvc monitor configuration)
    set(target ccamera-fuzz-${name})
    add_executable(${target} fuzz_${name}.c)
    ccamera_compile_settings(${target})
    target_link_libraries(${target} PRIVATE ccamera-fuzz-support)
    target_compile_options(${target} PRIVATE
        -g -O1 -fno-omit-frame-pointer -fno-sanitize-recover=all
        -fsanitize=fuzzer-no-link,address,undefined)
    target_link_options(${target} PRIVATE -fsanitize=fuzzer,address,undefined)
    set_target_properties(${target} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/fuzz")

    # Write discoveries/artifacts under the build directory, never into the
    # checked-in seed corpus. A fresh writable corpus is not required each run.
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/corpus/${name}"
                        "${CMAKE_CURRENT_BINARY_DIR}/artifacts/${name}")
    set(max_len 4096)
    if(name STREQUAL "ipa_wire")
        set(max_len 65536)
    endif()
    add_test(NAME fuzz-${name}-smoke COMMAND ${target}
        "${CMAKE_CURRENT_BINARY_DIR}/corpus/${name}"
        "${CMAKE_CURRENT_SOURCE_DIR}/corpus/${name}"
        -runs=1000 -seed=1 -max_len=${max_len} -timeout=5 -rss_limit_mb=512
        "-artifact_prefix=${CMAKE_CURRENT_BINARY_DIR}/artifacts/${name}/")
    set_tests_properties(fuzz-${name}-smoke PROPERTIES LABELS fuzz TIMEOUT 60)
endforeach()
