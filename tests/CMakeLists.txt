# Resolve test output paths from the library build directory.
foreach(kind RUNTIME LIBRARY)
    set(output_variable "CMAKE_${kind}_OUTPUT_DIRECTORY")
    set(output_path "${${output_variable}}")
    if(output_path STREQUAL "")
        set(${output_variable} "${libccamera_BINARY_DIR}")
    elseif(NOT IS_ABSOLUTE "${output_path}" AND NOT output_path MATCHES "\\$<")
        set(${output_variable} "${libccamera_BINARY_DIR}/${output_path}")
    endif()
endforeach()

function(ccamera_add_test_executable target)
    add_executable(${target} ${ARGN})
    ccamera_compile_settings(${target})
endfunction()

if(CCAMERA_BUILD_TESTS)
    # Compile the private syscall seam into tests, without exposing it in the API.

    ccamera_add_test_executable(ccamera-test test_camera.c ${CCAMERA_SOURCES})
    target_include_directories(ccamera-test PRIVATE "${libccamera_SOURCE_DIR}/include")
    add_test(NAME camera-lifecycle COMMAND ccamera-test)
    add_test(NAME cfe-frontend COMMAND ccamera-test cfe)
    add_test(NAME capture-requests COMMAND ccamera-test requests)
    add_test(NAME sensor-delays COMMAND ccamera-test delays)
    add_test(NAME request-sensor-controls COMMAND ccamera-test request-controls)
    add_test(NAME sensor-defaults COMMAND ccamera-test sensor)
    add_test(NAME cfe-ipa-feedback COMMAND ccamera-test ipa)
    add_test(NAME uvc-pipeline COMMAND ccamera-test uvc)
    add_test(NAME oriented-capture COMMAND ccamera-test orientation)
    add_test(NAME identity-discovery COMMAND ccamera-test identities)
    add_test(NAME media-hotplug COMMAND ccamera-test hotplug)
    add_test(NAME cfe-try-configuration COMMAND ccamera-test try-configuration)
    add_test(NAME raw-stream-roles COMMAND ccamera-test roles)
    add_test(NAME native-still-setup COMMAND ccamera-test still)

    ccamera_add_test_executable(ccamera-uvc-test test_uvc.c)
    target_link_libraries(ccamera-uvc-test PRIVATE ccamera::ccamera)
    add_test(NAME uvc-controls COMMAND ccamera-uvc-test)

    ccamera_add_test_executable(ccamera-stream-test test_stream.c ${CCAMERA_SOURCES})
    target_include_directories(ccamera-stream-test PRIVATE "${libccamera_SOURCE_DIR}/include")
    add_test(NAME stream-lifecycle COMMAND ccamera-stream-test)

    ccamera_add_test_executable(ccamera-interval-test test_frame_interval.c ${CCAMERA_SOURCES})
    target_include_directories(ccamera-interval-test PRIVATE "${libccamera_SOURCE_DIR}/include")
    add_test(NAME frame-intervals COMMAND ccamera-interval-test)

    ccamera_add_test_executable(ccamera-vimc-test test_vimc.c ${CCAMERA_SOURCES})
    target_include_directories(ccamera-vimc-test PRIVATE "${libccamera_SOURCE_DIR}/include")
    add_test(NAME vimc-pipeline COMMAND ccamera-vimc-test)

    ccamera_add_test_executable(ccamera-vc4-test test_vc4.c ${CCAMERA_SOURCES})
    target_include_directories(ccamera-vc4-test PRIVATE "${libccamera_SOURCE_DIR}/include")
    add_test(NAME vc4-isp COMMAND ccamera-vc4-test)
    add_test(NAME pisp-backend COMMAND ccamera-vc4-test pisp)
    add_test(NAME isp-try-configuration COMMAND ccamera-vc4-test try-configuration)
    add_test(NAME processed-stream-roles COMMAND ccamera-vc4-test roles)
    add_test(NAME pi-still-setup COMMAND ccamera-vc4-test still)
    add_test(NAME pipeline-requests COMMAND ccamera-vc4-test requests)
    add_test(NAME request-image-buffers COMMAND ccamera-vc4-test buffers)
    add_test(NAME acquire-fences COMMAND ccamera-vc4-test fences)
    add_test(NAME vc4-ipa-feedback COMMAND ccamera-vc4-test ipa)
    add_test(NAME cfe-pisp-feedback COMMAND ccamera-vc4-test cfe-be)
    add_test(NAME vc4-photo-graphs COMMAND ccamera-vc4-test vc4-photo)
    add_test(NAME ipa-proxy-vc4-photo-graphs COMMAND ccamera-vc4-test vc4-photo $<TARGET_FILE:ccamera-ipa-worker>)
    set_tests_properties(ipa-proxy-vc4-photo-graphs PROPERTIES TIMEOUT 60 SKIP_RETURN_CODE 77)
    add_test(NAME pisp-photo-graphs COMMAND ccamera-vc4-test photo)
    add_test(NAME pisp-native-photo-graphs COMMAND ccamera-vc4-test native-photo)
    add_test(NAME ipa-proxy-pisp-native-photo-graphs COMMAND ccamera-vc4-test native-photo $<TARGET_FILE:ccamera-ipa-worker>)
    set_tests_properties(ipa-proxy-pisp-native-photo-graphs PROPERTIES TIMEOUT 60 SKIP_RETURN_CODE 77)
    add_test(NAME ipa-proxy-pisp-photo-graphs COMMAND ccamera-vc4-test photo $<TARGET_FILE:ccamera-ipa-worker>)
    set_tests_properties(ipa-proxy-pisp-photo-graphs PROPERTIES TIMEOUT 60 SKIP_RETURN_CODE 77)
    add_test(NAME sensor-timing-graphs COMMAND ccamera-vc4-test timing)
    add_test(NAME autofocus-graphs COMMAND ccamera-vc4-test af)
    add_test(NAME embedded-autofocus-graph COMMAND ccamera-vc4-test embedded)
    add_test(NAME media-v2-pipeline-graphs COMMAND ccamera-vc4-test topology)

    ccamera_add_test_executable(ccamera-delayed-test test_delayed_controls.c "${libccamera_SOURCE_DIR}/src/delayed_controls.c")
    target_include_directories(ccamera-delayed-test PRIVATE "${libccamera_SOURCE_DIR}/include")
    add_test(NAME delayed-controls COMMAND ccamera-delayed-test)

    ccamera_add_test_executable(ccamera-sensor-test test_sensor.c)
    target_link_libraries(ccamera-sensor-test PRIVATE ccamera::ccamera)
    add_test(NAME sensor-helpers COMMAND ccamera-sensor-test)

    ccamera_add_test_executable(ccamera-embedded-test test_embedded.c)
    target_link_libraries(ccamera-embedded-test PRIVATE ccamera::ccamera)
    add_test(NAME sensor-embedded-data COMMAND ccamera-embedded-test)

    ccamera_add_test_executable(ccamera-sensor-streams-test test_sensor_streams.c ${CCAMERA_SOURCES})
    target_include_directories(ccamera-sensor-streams-test PRIVATE "${libccamera_SOURCE_DIR}/include")
    add_test(NAME sensor-stream-routing COMMAND ccamera-sensor-streams-test)

    ccamera_add_test_executable(ccamera-sensor-mode-test test_sensor_mode.c ${CCAMERA_SOURCES})
    target_include_directories(ccamera-sensor-mode-test PRIVATE "${libccamera_SOURCE_DIR}/include")
    add_test(NAME sensor-mode-selection COMMAND ccamera-sensor-mode-test)
    add_test(NAME rpi-sensor-mode-selection COMMAND ccamera-sensor-mode-test rpi)

    ccamera_add_test_executable(ccamera-properties-test test_camera_properties.c ${CCAMERA_SOURCES})
    target_include_directories(ccamera-properties-test PRIVATE "${libccamera_SOURCE_DIR}/include")
    add_test(NAME camera-properties COMMAND ccamera-properties-test)

    ccamera_add_test_executable(ccamera-orientation-test test_orientation.c ${CCAMERA_SOURCES})
    target_include_directories(ccamera-orientation-test PRIVATE "${libccamera_SOURCE_DIR}/include")
    add_test(NAME camera-orientation COMMAND ccamera-orientation-test)

    ccamera_add_test_executable(ccamera-id-test test_camera_id.c ${CCAMERA_SOURCES})
    target_include_directories(ccamera-id-test PRIVATE "${libccamera_SOURCE_DIR}/include")
    add_test(NAME camera-identities COMMAND ccamera-id-test)

    ccamera_add_test_executable(ccamera-monitor-test test_monitor.c ${CCAMERA_SOURCES})
    target_include_directories(ccamera-monitor-test PRIVATE "${libccamera_SOURCE_DIR}/include")
    add_test(NAME camera-hotplug COMMAND ccamera-monitor-test)
    add_test(NAME linux-notifications COMMAND ccamera-monitor-test native)

    ccamera_add_test_executable(ccamera-configuration-test test_configuration.c ${CCAMERA_SOURCES})
    target_include_directories(ccamera-configuration-test PRIVATE "${libccamera_SOURCE_DIR}/include")
    add_test(NAME stream-roles COMMAND ccamera-configuration-test)

    ccamera_add_test_executable(ccamera-embedded-capture-test test_embedded_capture.c ${CCAMERA_SOURCES})
    target_include_directories(ccamera-embedded-capture-test PRIVATE "${libccamera_SOURCE_DIR}/include")
    add_test(NAME embedded-capture COMMAND ccamera-embedded-capture-test)

    ccamera_add_test_executable(ccamera-pisp-config-test test_pisp_config.c)
    target_link_libraries(ccamera-pisp-config-test PRIVATE ccamera::ccamera)
    add_test(NAME pisp-configuration COMMAND ccamera-pisp-config-test)

    ccamera_add_test_executable(ccamera-pisp-processing-test test_pisp_processing.c)
    target_link_libraries(ccamera-pisp-processing-test PRIVATE ccamera::ccamera)
    add_test(NAME pisp-processing COMMAND ccamera-pisp-processing-test)

    ccamera_add_test_executable(ccamera-photo-test test_photo.c)
    target_link_libraries(ccamera-photo-test PRIVATE ccamera::ccamera)
    add_dependencies(ccamera-photo-test ccamera-ipa-worker)
    add_test(NAME photographic-controls COMMAND ccamera-photo-test)
    add_test(NAME ipa-proxy-photographic-controls COMMAND ccamera-photo-test $<TARGET_FILE:ccamera-ipa-worker>)
    set_tests_properties(ipa-proxy-photographic-controls PROPERTIES TIMEOUT 60 SKIP_RETURN_CODE 77)

    ccamera_add_test_executable(ccamera-vc4-tuning-test test_vc4_tuning.c)
    target_link_libraries(ccamera-vc4-tuning-test PRIVATE ccamera::ccamera)
    add_test(NAME vc4-module-profiles COMMAND ccamera-vc4-tuning-test)

    ccamera_add_test_executable(ccamera-pisp-tuning-test test_pisp_tuning.c)
    target_link_libraries(ccamera-pisp-tuning-test PRIVATE ccamera::ccamera)
    add_dependencies(ccamera-pisp-tuning-test ccamera-ipa-worker)
    add_test(NAME pisp-photo-tuning COMMAND ccamera-pisp-tuning-test)

    add_test(NAME ipa-proxy-pisp-controller COMMAND ccamera-pisp-tuning-test $<TARGET_FILE:ccamera-ipa-worker>)
    set_tests_properties(ipa-proxy-pisp-controller PROPERTIES TIMEOUT 60 SKIP_RETURN_CODE 77)
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND)
        add_test(NAME sensor-timing-analysis COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_sensor_timing_analysis.py)
    endif()

    ccamera_add_test_executable(ccamera-ipa-test test_ipa.c)
    target_link_libraries(ccamera-ipa-test PRIVATE ccamera::ccamera)
    add_test(NAME ipa-algorithms COMMAND ccamera-ipa-test)
    foreach(variant valid no_info bad_api bad_abi bad_name bad_pipeline no_factory unresolved origin manager_v4 manager_v9)
        add_library(ccamera-ipa-fixture-${variant} MODULE ipa_module_fixture.c)
        ccamera_compile_settings(ccamera-ipa-fixture-${variant})
        target_include_directories(ccamera-ipa-fixture-${variant} PRIVATE "${libccamera_SOURCE_DIR}/include")
        string(TOUPPER "${variant}" definition)
        target_compile_definitions(ccamera-ipa-fixture-${variant} PRIVATE "CC_IPA_FIXTURE_${definition}=1")
        set_target_properties(ccamera-ipa-fixture-${variant} PROPERTIES C_VISIBILITY_PRESET hidden)
    endforeach()
    add_library(ccamera-ipa-fixture-helper SHARED ipa_module_dependency.c)
    ccamera_compile_settings(ccamera-ipa-fixture-helper)
    set_target_properties(ccamera-ipa-fixture-helper PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${libccamera_BINARY_DIR}/ipa-dependencies" C_VISIBILITY_PRESET hidden)
    target_link_libraries(ccamera-ipa-fixture-origin PRIVATE ccamera-ipa-fixture-helper)
    set_target_properties(ccamera-ipa-fixture-origin PROPERTIES BUILD_RPATH_USE_ORIGIN TRUE
        BUILD_RPATH "$ORIGIN/ipa-dependencies")

    ccamera_add_test_executable(ccamera-ipa-module-test test_ipa_module.c ${CCAMERA_SOURCES})
    target_include_directories(ccamera-ipa-module-test PRIVATE "${libccamera_SOURCE_DIR}/include")
    add_test(NAME ipa-modules COMMAND ccamera-ipa-module-test
        $<TARGET_FILE:ccamera-ipa-fixture-valid> $<TARGET_FILE:ccamera-ipa-fixture-no_info>
        $<TARGET_FILE:ccamera-ipa-fixture-bad_api> $<TARGET_FILE:ccamera-ipa-fixture-bad_abi>
        $<TARGET_FILE:ccamera-ipa-fixture-bad_name> $<TARGET_FILE:ccamera-ipa-fixture-bad_pipeline>
        $<TARGET_FILE:ccamera-ipa-fixture-no_factory> $<TARGET_FILE:ccamera-ipa-fixture-unresolved>
        $<TARGET_FILE:ccamera-ipa-fixture-origin>)
    add_library(ccamera-ipa-auto-fixture MODULE ipa_module_fixture.c)
    ccamera_compile_settings(ccamera-ipa-auto-fixture)
    target_include_directories(ccamera-ipa-auto-fixture PRIVATE "${libccamera_SOURCE_DIR}/include")
    set_target_properties(ccamera-ipa-auto-fixture PROPERTIES C_VISIBILITY_PRESET hidden)

    ccamera_add_test_executable(ccamera-ipa-signature-test test_ipa_signature.c ${CCAMERA_SOURCES})
    target_include_directories(ccamera-ipa-signature-test PRIVATE "${libccamera_SOURCE_DIR}/include")
    target_link_options(ccamera-ipa-signature-test PRIVATE "LINKER:--wrap=pread" "LINKER:--wrap=pread64")
    add_dependencies(ccamera-ipa-signature-test ccamera-ipa-worker ccamera-ipa-auto-fixture)
    if(CCAMERA_WITH_IPA_CRYPTO)
        # Test trust is independent of production keys, including public-only builds.
        ccamera_ipa_prepare_key("" "" "${libccamera_BINARY_DIR}/ipa-test-keys"
            "${libccamera_BINARY_DIR}/ipa-test-keys/cc_ipa_public_key.h" test_key)
        ccamera_ipa_prepare_key("" "" "${libccamera_BINARY_DIR}/ipa-wrong-keys"
            "${libccamera_BINARY_DIR}/ipa-wrong-keys/cc_ipa_public_key.h" wrong_key)
        target_include_directories(ccamera-ipa-signature-test BEFORE PRIVATE "${libccamera_BINARY_DIR}/ipa-test-keys")
        ccamera_sign_ipa(TARGET ccamera-ipa-auto-fixture PRIVATE_KEY "${test_key}")
        ccamera_ipa_host_openssl(host_openssl)
        add_custom_target(ccamera-ipa-reference-signatures ALL
            COMMAND "${host_openssl}" dgst -sha256 -sign "${test_key}" -passin pass:
                -out "$<TARGET_FILE:ccamera-ipa-auto-fixture>.reference.sign" "$<TARGET_FILE:ccamera-ipa-auto-fixture>"
            COMMAND "${host_openssl}" dgst -sha256 -sign "${wrong_key}" -passin pass:
                -out "$<TARGET_FILE:ccamera-ipa-auto-fixture>.wrong.sign" "$<TARGET_FILE:ccamera-ipa-auto-fixture>"
            DEPENDS ccamera-ipa-auto-fixture VERBATIM)
        add_dependencies(ccamera-ipa-signature-test ccamera-ipa-auto-fixture-ipa-sign ccamera-ipa-reference-signatures)
    endif()
    add_test(NAME ipa-signatures COMMAND ccamera-ipa-signature-test
        $<TARGET_FILE:ccamera-ipa-auto-fixture> $<TARGET_FILE:ccamera-ipa-worker>)
    set_tests_properties(ipa-signatures PROPERTIES TIMEOUT 30)
    add_library(ccamera-ipa-auto-manager-fixture MODULE ipa_module_fixture.c)
    ccamera_compile_settings(ccamera-ipa-auto-manager-fixture)
    target_include_directories(ccamera-ipa-auto-manager-fixture PRIVATE "${libccamera_SOURCE_DIR}/include")
    target_compile_definitions(ccamera-ipa-auto-manager-fixture PRIVATE CC_IPA_FIXTURE_MANAGER_V4=1)
    set_target_properties(ccamera-ipa-auto-manager-fixture PROPERTIES C_VISIBILITY_PRESET hidden)

    ccamera_add_test_executable(ccamera-ipa-auto-graph-test test_vc4.c ${CCAMERA_SOURCES})
    target_include_directories(ccamera-ipa-auto-graph-test PRIVATE "${libccamera_SOURCE_DIR}/include")
    add_dependencies(ccamera-ipa-auto-graph-test ccamera-ipa-worker ccamera-ipa-auto-fixture ccamera-ipa-auto-manager-fixture)
    if(CCAMERA_WITH_IPA_CRYPTO)
        target_include_directories(ccamera-ipa-auto-graph-test BEFORE PRIVATE "${libccamera_BINARY_DIR}/ipa-test-keys")
        ccamera_sign_ipa(TARGET ccamera-ipa-auto-manager-fixture PRIVATE_KEY "${test_key}")
        add_dependencies(ccamera-ipa-auto-graph-test ccamera-ipa-auto-fixture-ipa-sign ccamera-ipa-auto-manager-fixture-ipa-sign)
    endif()
    add_test(NAME ipa-auto-module-graphs COMMAND ccamera-ipa-auto-graph-test auto-modules
        $<TARGET_FILE:ccamera-ipa-auto-fixture> $<TARGET_FILE:ccamera-ipa-worker>)
    add_test(NAME ipa-auto-manager-graphs COMMAND ccamera-ipa-auto-graph-test auto-manager
        $<TARGET_FILE:ccamera-ipa-auto-manager-fixture> $<TARGET_FILE:ccamera-ipa-worker>)
    set_tests_properties(ipa-auto-module-graphs ipa-auto-manager-graphs PROPERTIES TIMEOUT 60 SKIP_RETURN_CODE 77)
    if(CCAMERA_WITH_IPA_CRYPTO AND Python3_Interpreter_FOUND AND NOT CMAKE_CROSSCOMPILING)
        add_test(NAME ipa-crypto-build COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/test_ipa_crypto_build.py"
            "${libccamera_SOURCE_DIR}" "${libccamera_BINARY_DIR}" "${host_openssl}" "${CMAKE_C_COMPILER}")
        set_tests_properties(ipa-crypto-build PROPERTIES TIMEOUT 180)
    endif()
    add_test(NAME ipa-module-graphs COMMAND ccamera-vc4-test modules $<TARGET_FILE:ccamera-ipa-fixture-valid>)

    ccamera_add_test_executable(ccamera-ipa-manager-test test_ipa_manager.c ${CCAMERA_SOURCES})
    target_include_directories(ccamera-ipa-manager-test PRIVATE "${libccamera_SOURCE_DIR}/include")
    add_test(NAME ipa-discovery COMMAND ccamera-ipa-manager-test
        $<TARGET_FILE:ccamera-ipa-fixture-valid> $<TARGET_FILE:ccamera-ipa-fixture-manager_v4>
        $<TARGET_FILE:ccamera-ipa-fixture-manager_v9> $<TARGET_FILE:ccamera-ipa-fixture-bad_api>
        $<TARGET_FILE:ccamera-ipa-fixture-no_factory> $<TARGET_FILE:ccamera-ipa-fixture-unresolved>)
    add_test(NAME ipa-manager-graphs COMMAND ccamera-vc4-test manager $<TARGET_FILE:ccamera-ipa-fixture-manager_v4>)

    ccamera_add_test_executable(ccamera-ipc-test test_ipc.c "${libccamera_SOURCE_DIR}/src/ipc.c")
    target_include_directories(ccamera-ipc-test PRIVATE "${libccamera_SOURCE_DIR}/include")
    add_test(NAME ipc-transport COMMAND ccamera-ipc-test)
    add_test(NAME ipc-malformed COMMAND ccamera-ipc-test malformed)
    set_tests_properties(ipc-transport ipc-malformed PROPERTIES TIMEOUT 30)
    find_package(Threads REQUIRED)

    ccamera_add_test_executable(ccamera-process-helper process_helper.c)

    ccamera_add_test_executable(ccamera-process-test test_process.c "${libccamera_SOURCE_DIR}/src/process.c")
    target_include_directories(ccamera-process-test PRIVATE "${libccamera_SOURCE_DIR}/include")
    target_link_libraries(ccamera-process-test PRIVATE Threads::Threads)
    add_test(NAME process-lifecycle COMMAND ccamera-process-test $<TARGET_FILE:ccamera-process-helper>)
    add_test(NAME process-namespaces COMMAND ccamera-process-test $<TARGET_FILE:ccamera-process-helper> namespaces)
    set_tests_properties(process-lifecycle process-namespaces PROPERTIES TIMEOUT 60 SKIP_RETURN_CODE 77)

    ccamera_add_test_executable(ccamera-ipc-pipe-test test_ipc_pipe.c
        "${libccamera_SOURCE_DIR}/src/ipc.c"
        "${libccamera_SOURCE_DIR}/src/ipc_pipe.c"
        "${libccamera_SOURCE_DIR}/src/process.c")
    target_include_directories(ccamera-ipc-pipe-test PRIVATE "${libccamera_SOURCE_DIR}/include")
    add_test(NAME ipc-rpc COMMAND ccamera-ipc-pipe-test)
    set_tests_properties(ipc-rpc PROPERTIES TIMEOUT 30 SKIP_RETURN_CODE 77)

    ccamera_add_test_executable(ccamera-ipa-wire-test test_ipa_wire.c)
    target_link_libraries(ccamera-ipa-wire-test PRIVATE ccamera::ccamera)
    # The codec is private; shared builds compile its implementation into this test.
    if(BUILD_SHARED_LIBS)
        target_sources(ccamera-ipa-wire-test PRIVATE "${libccamera_SOURCE_DIR}/src/ipa_wire.c")
    endif()
    add_test(NAME ipa-wire COMMAND ccamera-ipa-wire-test)

    ccamera_add_test_executable(ccamera-ipa-message-test test_ipa_message.c)
    target_link_libraries(ccamera-ipa-message-test PRIVATE ccamera::ccamera)
    if(BUILD_SHARED_LIBS)
        target_sources(ccamera-ipa-message-test PRIVATE "${libccamera_SOURCE_DIR}/src/ipa_message.c")
    endif()
    add_test(NAME ipa-message COMMAND ccamera-ipa-message-test)
    add_library(ccamera-ipa-proxy-fixture MODULE ipa_proxy_fixture.c)
    ccamera_compile_settings(ccamera-ipa-proxy-fixture)
    target_link_libraries(ccamera-ipa-proxy-fixture PRIVATE ccamera::ccamera)
    set_target_properties(ccamera-ipa-proxy-fixture PROPERTIES C_VISIBILITY_PRESET hidden)

    ccamera_add_test_executable(ccamera-ipa-proxy-test test_ipa_proxy.c)
    target_link_libraries(ccamera-ipa-proxy-test PRIVATE ccamera::ccamera)
    add_dependencies(ccamera-ipa-proxy-test ccamera-ipa-worker ccamera-ipa-proxy-fixture ccamera-ipa-fixture-origin ccamera-ipa-fixture-unresolved)
    add_test(NAME ipa-proxy COMMAND ccamera-ipa-proxy-test $<TARGET_FILE:ccamera-ipa-worker> $<TARGET_FILE:ccamera-ipa-proxy-fixture>
        $<TARGET_FILE:ccamera-ipa-fixture-origin> $<TARGET_FILE:ccamera-ipa-fixture-unresolved>)
    set_tests_properties(ipa-proxy PROPERTIES TIMEOUT 30 SKIP_RETURN_CODE 77)
    add_dependencies(ccamera-ipa-test ccamera-ipa-worker)
    add_dependencies(ccamera-vc4-tuning-test ccamera-ipa-worker)
    add_test(NAME ipa-proxy-algorithms COMMAND ccamera-ipa-test $<TARGET_FILE:ccamera-ipa-worker>)
    add_test(NAME ipa-proxy-vc4-controller COMMAND ccamera-vc4-tuning-test $<TARGET_FILE:ccamera-ipa-worker>)
    set_tests_properties(ipa-proxy-algorithms ipa-proxy-vc4-controller PROPERTIES TIMEOUT 60 SKIP_RETURN_CODE 77)
    add_dependencies(ccamera-test ccamera-ipa-worker)
    add_dependencies(ccamera-vc4-test ccamera-ipa-worker)
    add_test(NAME ipa-proxy-graphs COMMAND ccamera-vc4-test proxy-feedback $<TARGET_FILE:ccamera-ipa-worker>)
    add_test(NAME ipa-proxy-cfe COMMAND ccamera-test proxy-feedback $<TARGET_FILE:ccamera-ipa-worker>)
    add_test(NAME ipa-proxy-module-graphs COMMAND ccamera-vc4-test proxy-modules
        $<TARGET_FILE:ccamera-ipa-fixture-valid> $<TARGET_FILE:ccamera-ipa-worker>)
    add_test(NAME ipa-proxy-manager-graphs COMMAND ccamera-vc4-test proxy-manager
        $<TARGET_FILE:ccamera-ipa-fixture-manager_v4> $<TARGET_FILE:ccamera-ipa-worker>)
    set_tests_properties(ipa-proxy-graphs ipa-proxy-cfe ipa-proxy-module-graphs ipa-proxy-manager-graphs
        PROPERTIES TIMEOUT 60 SKIP_RETURN_CODE 77)

    ccamera_add_test_executable(ccamera-ipa-vimc-test test_ipa_vimc.c)
    target_link_libraries(ccamera-ipa-vimc-test PRIVATE ccamera::ccamera)
    add_dependencies(ccamera-ipa-vimc-test ccamera-ipa-worker)
    add_test(NAME ipa-vimc-ipc COMMAND ccamera-ipa-vimc-test $<TARGET_FILE:ccamera-ipa-worker>)
    set_tests_properties(ipa-vimc-ipc PROPERTIES TIMEOUT 30 SKIP_RETURN_CODE 77)
    add_dependencies(ccamera-vimc-test ccamera-ipa-worker)
    add_test(NAME ipa-vimc-graphs COMMAND ccamera-vimc-test $<TARGET_FILE:ccamera-ipa-worker>)
    set_tests_properties(ipa-vimc-graphs PROPERTIES TIMEOUT 60 SKIP_RETURN_CODE 77)
    add_library(ccamera-ipa-runtime-fixture MODULE ipa_module_fixture.c)
    ccamera_compile_settings(ccamera-ipa-runtime-fixture)
    target_include_directories(ccamera-ipa-runtime-fixture PRIVATE "${libccamera_SOURCE_DIR}/include")
    set_target_properties(ccamera-ipa-runtime-fixture PROPERTIES C_VISIBILITY_PRESET hidden
        LIBRARY_OUTPUT_DIRECTORY "$<TARGET_FILE_DIR:ccamera>/ipa/test/nested")

    ccamera_add_test_executable(ccamera-runtime-probe runtime_path_probe.c)
    target_link_libraries(ccamera-runtime-probe PRIVATE ccamera::ccamera)

    ccamera_add_test_executable(ccamera-runtime-test test_runtime_paths.c)
    add_test(NAME ipa-build-tree COMMAND ccamera-runtime-probe 1 $<TARGET_FILE:ccamera-ipa-runtime-fixture> build normal)
    if(BUILD_SHARED_LIBS)
        add_test(NAME ipa-relocated-tree COMMAND ccamera-runtime-test
            $<TARGET_FILE:ccamera-runtime-probe> $<TARGET_FILE:ccamera-ipa-runtime-fixture>
            $<TARGET_FILE:ccamera> $<TARGET_SONAME_FILE_NAME:ccamera>)
        add_test(NAME ipa-proxy-relocated-tree COMMAND ccamera-runtime-test
            $<TARGET_FILE:ccamera-runtime-probe> $<TARGET_FILE:ccamera-ipa-runtime-fixture>
            $<TARGET_FILE:ccamera> $<TARGET_SONAME_FILE_NAME:ccamera> $<TARGET_FILE:ccamera-ipa-worker>)
    else()
        add_test(NAME ipa-relocated-tree COMMAND ccamera-runtime-test
            $<TARGET_FILE:ccamera-runtime-probe> $<TARGET_FILE:ccamera-ipa-runtime-fixture> - -)
        add_test(NAME ipa-proxy-relocated-tree COMMAND ccamera-runtime-test
            $<TARGET_FILE:ccamera-runtime-probe> $<TARGET_FILE:ccamera-ipa-runtime-fixture> - - $<TARGET_FILE:ccamera-ipa-worker>)
    endif()
    set_tests_properties(ipa-relocated-tree PROPERTIES TIMEOUT 30)
    set_tests_properties(ipa-proxy-relocated-tree PROPERTIES TIMEOUT 30 SKIP_RETURN_CODE 77)

    ccamera_add_test_executable(ccamera-af-test test_af.c)
    target_link_libraries(ccamera-af-test PRIVATE ccamera::ccamera)
    add_test(NAME autofocus COMMAND ccamera-af-test)

    ccamera_add_test_executable(ccamera-lens-test test_lens.c ${CCAMERA_SOURCES})
    target_include_directories(ccamera-lens-test PRIVATE "${libccamera_SOURCE_DIR}/include")
    add_test(NAME lens-control COMMAND ccamera-lens-test)

    ccamera_add_test_executable(ccamera-media-topology-test test_media_topology.c
        "${libccamera_SOURCE_DIR}/src/media.c"
        "${libccamera_SOURCE_DIR}/src/media_topology.c"
        "${libccamera_SOURCE_DIR}/src/io.c")
    target_include_directories(ccamera-media-topology-test PRIVATE "${libccamera_SOURCE_DIR}/include")
    add_test(NAME media-v2-topology COMMAND ccamera-media-topology-test)
    add_test(NAME installed-consumer COMMAND ${CMAKE_COMMAND}
        -DCC_BUILD_DIR=${libccamera_BINARY_DIR}
        -DCC_SOURCE_DIR=${libccamera_SOURCE_DIR}
        -DCC_VERSION=${PROJECT_VERSION}
        "-DCC_GENERATOR=${CMAKE_GENERATOR}"
        -DCC_CONFIG=$<CONFIG>
        -DCC_COMPILER=${CMAKE_C_COMPILER}
        -DCC_WORKER_INSTALL_DIR=${CMAKE_INSTALL_LIBEXECDIR}/ccamera
        -DCC_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
        -DCC_SYSROOT=${CMAKE_SYSROOT}
        -DCC_CROSSCOMPILING=${CMAKE_CROSSCOMPILING}
        "-DCC_EMULATOR=${CMAKE_CROSSCOMPILING_EMULATOR}"
        "-DCC_C_FLAGS=${CMAKE_C_FLAGS}"
        "-DCC_LINKER_FLAGS=${CMAKE_EXE_LINKER_FLAGS}"
        -P ${CMAKE_CURRENT_SOURCE_DIR}/install_consumer.cmake)
endif()

if(CCAMERA_BUILD_DRM_FENCE_TEST)
    set(CCAMERA_DRM_RENDER_NODE "/dev/dri/renderD128" CACHE FILEPATH "DRM render node for the native fence test")

    ccamera_add_test_executable(ccamera-native-fence-test native_sync_file.c
        "${libccamera_SOURCE_DIR}/src/fence.c" "${libccamera_SOURCE_DIR}/src/io.c")
    target_link_libraries(ccamera-native-fence-test PRIVATE ccamera::ccamera)
    add_test(NAME native-sync-file COMMAND ccamera-native-fence-test "${CCAMERA_DRM_RENDER_NODE}")
    set_tests_properties(native-sync-file PROPERTIES SKIP_RETURN_CODE 77)
endif()

# Resolve relative test paths from the library build directory.
get_property(ccamera_tests DIRECTORY PROPERTY TESTS)
if(ccamera_tests)
    set_tests_properties(${ccamera_tests} PROPERTIES WORKING_DIRECTORY "${libccamera_BINARY_DIR}")
endif()

if(CCAMERA_BUILD_FUZZERS)
    add_subdirectory(fuzz)
endif()
