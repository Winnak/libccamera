# SPDX-License-Identifier: CC0-1.0
set(prefix "${CC_BUILD_DIR}/test-install")
set(consumer "${CC_BUILD_DIR}/test-consumer")

function(run)
    execute_process(COMMAND ${ARGV} RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "Command failed (${result}): ${ARGV}\n${output}\n${error}")
    endif()
endfunction()

run("${CMAKE_COMMAND}" --install "${CC_BUILD_DIR}" --prefix "${prefix}" --config "${CC_CONFIG}")
set(toolchain_args)
if(CC_TOOLCHAIN_FILE)
    list(APPEND toolchain_args "-DCMAKE_TOOLCHAIN_FILE=${CC_TOOLCHAIN_FILE}")
endif()
if(CC_SYSROOT)
    list(APPEND toolchain_args "-DCMAKE_SYSROOT=${CC_SYSROOT}")
endif()
run("${CMAKE_COMMAND}" -S "${CC_SOURCE_DIR}/tests/consumer" -B "${consumer}"
    -G "${CC_GENERATOR}"
    "-DCMAKE_PREFIX_PATH=${prefix}" "-DCMAKE_C_COMPILER=${CC_COMPILER}"
    "-DCMAKE_C_FLAGS=${CC_C_FLAGS}" "-DCMAKE_EXE_LINKER_FLAGS=${CC_LINKER_FLAGS}"
    "-DCMAKE_BUILD_TYPE=${CC_CONFIG}" ${toolchain_args})
run("${CMAKE_COMMAND}" --build "${consumer}" --config "${CC_CONFIG}")
if(CC_CROSSCOMPILING AND NOT CC_EMULATOR)
    message(FATAL_ERROR "Consumer cross-built successfully, but running it requires CMAKE_CROSSCOMPILING_EMULATOR (or execution on the target).")
endif()
if(EXISTS "${consumer}/${CC_CONFIG}/consumer")
    run(${CC_EMULATOR} "${consumer}/${CC_CONFIG}/consumer" "${prefix}/${CC_WORKER_INSTALL_DIR}/ccamera-ipa-worker")
else()
    run(${CC_EMULATOR} "${consumer}/consumer" "${prefix}/${CC_WORKER_INSTALL_DIR}/ccamera-ipa-worker")
endif()
