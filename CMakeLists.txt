cmake_minimum_required(VERSION 3.16)
project(libccamera VERSION 0.1.0 LANGUAGES C)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "libccamera currently requires Linux media and V4L2 APIs")
endif()

option(BUILD_SHARED_LIBS "Build a shared library instead of a static archive" OFF)
option(CCAMERA_BUILD_EXAMPLES "Build the capture examples" ON)
option(CCAMERA_BUILD_TESTS "Build hardware-independent tests" ON)
option(CCAMERA_BUILD_FUZZERS "Build Clang libFuzzer targets with ASan and UBSan" OFF)
option(CCAMERA_BUILD_DRM_FENCE_TEST "Build the optional real DRM sync-file test" OFF)
option(CCAMERA_WITH_IPA_CRYPTO "Verify native C IPA signatures with OpenSSL Crypto" OFF)
set(CCAMERA_IPA_PRIVATE_KEY "" CACHE FILEPATH "PEM RSA private signing key (never installed)")
set(CCAMERA_IPA_PUBLIC_KEY "" CACHE FILEPATH "PEM RSA public verification key for verification-only builds")
if(CCAMERA_IPA_PRIVATE_KEY AND CCAMERA_IPA_PUBLIC_KEY)
    message(FATAL_ERROR "CCAMERA_IPA_PRIVATE_KEY and CCAMERA_IPA_PUBLIC_KEY are mutually exclusive")
endif()
if(NOT CCAMERA_WITH_IPA_CRYPTO AND (CCAMERA_IPA_PRIVATE_KEY OR CCAMERA_IPA_PUBLIC_KEY))
    message(FATAL_ERROR "IPA key options require CCAMERA_WITH_IPA_CRYPTO=ON")
endif()
if(CCAMERA_WITH_IPA_CRYPTO)
    cmake_minimum_required(VERSION 3.18)
    find_package(OpenSSL REQUIRED COMPONENTS Crypto)
    include(cmake/ccameraOpenSSL.cmake)
    ccamera_ipa_openssl_requirements()
    include(cmake/ccameraIpaKey.cmake)
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/ipa-crypto")
    ccamera_ipa_prepare_key("${CCAMERA_IPA_PRIVATE_KEY}" "${CCAMERA_IPA_PUBLIC_KEY}"
        "${CMAKE_CURRENT_BINARY_DIR}/ipa-keys" "${CMAKE_CURRENT_BINARY_DIR}/ipa-crypto/cc_ipa_public_key.h" signing_key)
    set_property(GLOBAL PROPERTY CCAMERA_IPA_BUILD_SIGNING_KEY "${signing_key}")
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CheckSymbolExists)
set(CCAMERA_IPA_MODULE_DIR "${CMAKE_INSTALL_FULL_LIBDIR}/ccamera/ipa" CACHE PATH "Installed C IPA module directory")
set(CCAMERA_IPA_WORKER_DIR "${CMAKE_INSTALL_FULL_LIBEXECDIR}/ccamera")
check_symbol_exists(dlopen "dlfcn.h" CCAMERA_DLOPEN_IN_LIBC)
set(CCAMERA_LOADER_LIBRARIES)
if(NOT CCAMERA_DLOPEN_IN_LIBC)
    set(CCAMERA_LOADER_LIBRARIES ${CMAKE_DL_LIBS})
endif()
set(CCAMERA_SOURCES
    src/io.c src/media.c src/media_topology.c src/controls.c src/stream.c src/bayer.c
    src/orientation.c src/camera.c src/configuration.c src/camera_properties.c src/camera_id.c
    src/sensor_mode.c src/vimc.c src/uvc.c src/discovery.c src/monitor.c src/monitor_native.c
    src/capture.c src/still.c src/request_buffers.c src/fence.c src/delayed_controls.c
    src/sensor.c src/sensor_timing.c src/ipa.c src/photo.c src/photo_processing.c
    src/ipa_module.c src/ipa_module_elf.c src/ipa_manager.c src/ipa_proxy.c src/ipa_vimc.c
    src/source_paths.c src/ipc.c src/ipc_pipe.c src/process.c src/ipa_wire.c src/ipa_message.c
    src/ipa_statistics.c src/ipa_reference.c src/af.c src/embedded.c src/embedded_capture.c
    src/sensor_streams.c src/frame_interval.c src/vc4_tuning.c src/vc4_profiles.c
    src/pisp_tuning.c src/pisp_native.c src/pisp_profiles.c src/ipa_vc4.c src/vc4_isp.c
    src/cfe.c src/cfe_config.c src/pisp_format.c src/pisp_processing.c src/pisp_resize.c
    src/pisp_tiling.c src/pisp_be_config.c src/pisp_be.c
)
list(TRANSFORM CCAMERA_SOURCES PREPEND "${libccamera_SOURCE_DIR}/")

function(ccamera_compile_settings target)
    if(CCAMERA_WITH_IPA_CRYPTO)
        target_link_libraries(${target} PRIVATE OpenSSL::Crypto)
        target_compile_definitions(${target} PRIVATE CCAMERA_WITH_IPA_CRYPTO=1)
        target_include_directories(${target} PRIVATE "${libccamera_BINARY_DIR}/ipa-crypto")
    endif()
    if(CCAMERA_LOADER_LIBRARIES)
        target_link_libraries(${target} PRIVATE ${CCAMERA_LOADER_LIBRARIES})
    endif()
    # CMake learned the C_STANDARD=17 spelling in 3.21. Older supported
    # versions can still drive C17-capable GCC/Clang explicitly.
    if(CMAKE_VERSION VERSION_LESS 3.21 AND CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE -std=c17)
        set_target_properties(${target} PROPERTIES C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
    else()
        set_target_properties(${target} PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
    endif()
    set_target_properties(${target} PROPERTIES BUILD_RPATH_USE_ORIGIN TRUE)
    target_compile_definitions(${target} PRIVATE _POSIX_C_SOURCE=200809L _FILE_OFFSET_BITS=64
        "CCAMERA_VERSION=\"${PROJECT_VERSION}\"")
    target_compile_definitions(${target} PRIVATE "CCAMERA_IPA_MODULE_DIR=\"${CCAMERA_IPA_MODULE_DIR}\"")
    target_compile_definitions(${target} PRIVATE "CCAMERA_IPA_WORKER_DIR=\"${CCAMERA_IPA_WORKER_DIR}\"")
    target_include_directories(${target} PRIVATE ${libccamera_SOURCE_DIR}/src)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
endfunction()

add_library(ccamera ${CCAMERA_SOURCES})
add_library(ccamera::ccamera ALIAS ccamera)
ccamera_compile_settings(ccamera)
target_include_directories(ccamera PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
set_target_properties(ccamera PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION ${PROJECT_VERSION_MAJOR}
    C_VISIBILITY_PRESET hidden POSITION_INDEPENDENT_CODE YES)
file(GENERATE OUTPUT "$<TARGET_FILE_DIR:ccamera>/.ccamera-build-tree" CONTENT "ccamera-build-tree-v1\n")

# Keep the exec worker self-contained in static and shared installations.
# Its module loader still resolves module $ORIGIN dependencies normally.
add_executable(ccamera-ipa-worker src/ipa_worker.c ${CCAMERA_SOURCES})
ccamera_compile_settings(ccamera-ipa-worker)
target_include_directories(ccamera-ipa-worker PRIVATE include)
set_target_properties(ccamera-ipa-worker PROPERTIES RUNTIME_OUTPUT_DIRECTORY "$<TARGET_FILE_DIR:ccamera>")
if(CCAMERA_WITH_IPA_CRYPTO)
    add_executable(ccamera-ipa-verify src/ipa_verify.c)
    ccamera_compile_settings(ccamera-ipa-verify)
    target_link_libraries(ccamera-ipa-verify PRIVATE ccamera::ccamera)
    install(TARGETS ccamera-ipa-verify RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

if(CCAMERA_BUILD_EXAMPLES)
    add_executable(ccamera-photo examples/photo.c)
    ccamera_compile_settings(ccamera-photo)
    target_link_libraries(ccamera-photo PRIVATE ccamera::ccamera)
    add_executable(ccamera-still examples/still.c)
    ccamera_compile_settings(ccamera-still)
    target_link_libraries(ccamera-still PRIVATE ccamera::ccamera)
    add_executable(ccamera-capture examples/capture.c)
    ccamera_compile_settings(ccamera-capture)
    target_link_libraries(ccamera-capture PRIVATE ccamera::ccamera)
    add_executable(ccamera-cfe-capture examples/cfe_capture.c)
    ccamera_compile_settings(ccamera-cfe-capture)
    target_link_libraries(ccamera-cfe-capture PRIVATE ccamera::ccamera)
    add_executable(ccamera-watch examples/watch.c)
    ccamera_compile_settings(ccamera-watch)
    target_link_libraries(ccamera-watch PRIVATE ccamera::ccamera)
endif()

if(CCAMERA_BUILD_TESTS OR CCAMERA_BUILD_FUZZERS OR CCAMERA_BUILD_DRM_FENCE_TEST)
    enable_testing()
    add_subdirectory(tests)
endif()

install(TARGETS ccamera EXPORT ccameraTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(TARGETS ccamera-ipa-worker RUNTIME DESTINATION ${CMAKE_INSTALL_LIBEXECDIR}/ccamera)
install(DIRECTORY include/ccamera DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES LICENSE DESTINATION ${CMAKE_INSTALL_DATADIR}/licenses/ccamera)
install(DIRECTORY LICENSES/ DESTINATION ${CMAKE_INSTALL_DATADIR}/licenses/ccamera
    FILES_MATCHING PATTERN "*.txt")
install(EXPORT ccameraTargets NAMESPACE ccamera:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ccamera)
configure_package_config_file(cmake/ccameraConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/ccameraConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ccamera)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/ccameraConfigVersion.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMinorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ccameraConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/ccameraConfigVersion.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ccamera)
install(FILES cmake/ccameraIpaSigning.cmake cmake/ccameraIpaSign.cmake cmake/ccameraOpenSSL.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ccamera)
file(RELATIVE_PATH CCAMERA_PC_PREFIX "${CMAKE_INSTALL_FULL_LIBDIR}/pkgconfig" "${CMAKE_INSTALL_PREFIX}")
set(CCAMERA_PC_PRIVATE_LIBS "")
set(CCAMERA_PC_PRIVATE_REQUIRES "")
if(CCAMERA_WITH_IPA_CRYPTO)
    set(CCAMERA_PC_PRIVATE_REQUIRES "libcrypto >= 3")
endif()
foreach(library IN LISTS CCAMERA_LOADER_LIBRARIES)
    string(APPEND CCAMERA_PC_PRIVATE_LIBS " -l${library}")
endforeach()
configure_file(cmake/ccamera.pc.in ccamera.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ccamera.pc DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
