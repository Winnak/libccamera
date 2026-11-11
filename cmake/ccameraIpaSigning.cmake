# SPDX-License-Identifier: CC0-1.0
include_guard(GLOBAL)

# Always find a HOST tool, independently of the target OpenSSL::Crypto library.
function(ccamera_ipa_host_openssl output)
    find_program(CCAMERA_HOST_OPENSSL NAMES openssl NO_CMAKE_FIND_ROOT_PATH)
    if(NOT CCAMERA_HOST_OPENSSL)
        message(FATAL_ERROR "IPA signing requires a host openssl executable")
    endif()
    execute_process(COMMAND "${CCAMERA_HOST_OPENSSL}" version
        RESULT_VARIABLE result OUTPUT_VARIABLE version ERROR_VARIABLE error)
    if(NOT result EQUAL 0 OR NOT version MATCHES "^OpenSSL ([0-9]+)\\.")
        message(FATAL_ERROR "Cannot run host openssl: ${error}")
    endif()
    if(CMAKE_MATCH_1 LESS 3)
        message(FATAL_ERROR "IPA signing requires host OpenSSL 3 or newer")
    endif()
    set(${output} "${CCAMERA_HOST_OPENSSL}" PARENT_SCOPE)
endfunction()

# This helper is exported without any configured key or build-tree paths.
function(ccamera_sign_ipa)
    if(CMAKE_VERSION VERSION_LESS 3.18)
        message(FATAL_ERROR "ccamera_sign_ipa requires CMake 3.18 or newer")
    endif()
    cmake_parse_arguments(IPA "" "TARGET;PRIVATE_KEY;INSTALL_DESTINATION;COMPONENT" "" ${ARGN})
    if(IPA_UNPARSED_ARGUMENTS OR IPA_KEYWORDS_MISSING_VALUES OR NOT IPA_TARGET OR NOT TARGET "${IPA_TARGET}")
        message(FATAL_ERROR "ccamera_sign_ipa(TARGET target [PRIVATE_KEY file] [INSTALL_DESTINATION dir] [COMPONENT name])")
    endif()
    get_target_property(type "${IPA_TARGET}" TYPE)
    if(NOT type MATCHES "^(MODULE_LIBRARY|SHARED_LIBRARY)$")
        message(FATAL_ERROR "ccamera_sign_ipa requires a module or shared library target")
    endif()
    if(TARGET "${IPA_TARGET}-ipa-sign")
        message(FATAL_ERROR "ccamera_sign_ipa already called for ${IPA_TARGET}")
    endif()
    if(NOT IPA_PRIVATE_KEY)
        get_property(IPA_PRIVATE_KEY GLOBAL PROPERTY CCAMERA_IPA_BUILD_SIGNING_KEY)
    endif()
    if(NOT IPA_PRIVATE_KEY)
        message(FATAL_ERROR "ccamera_sign_ipa requires PRIVATE_KEY (no configured private signing key)")
    endif()
    get_filename_component(IPA_PRIVATE_KEY "${IPA_PRIVATE_KEY}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    if(NOT EXISTS "${IPA_PRIVATE_KEY}" OR IS_DIRECTORY "${IPA_PRIVATE_KEY}")
        message(FATAL_ERROR "IPA private key is not a file: ${IPA_PRIVATE_KEY}")
    endif()
    ccamera_ipa_host_openssl(openssl)
    set(script "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/ccameraIpaSign.cmake")
    # A target is intentional: it runs on every build, even if only the key was
    # replaced without a relink. The script skips an up-to-date signature using
    # content hashes, including changes with preserved timestamps.
    add_custom_target("${IPA_TARGET}-ipa-sign" ALL
        COMMAND "${CMAKE_COMMAND}" "-DMODULE=$<TARGET_FILE:${IPA_TARGET}>"
            "-DPRIVATE_KEY=${IPA_PRIVATE_KEY}" "-DOPENSSL=${openssl}" -P "${script}"
        DEPENDS "${IPA_TARGET}" "${IPA_PRIVATE_KEY}" "${script}" VERBATIM)
    if(IPA_INSTALL_DESTINATION)
        if(NOT IPA_COMPONENT)
            set(IPA_COMPONENT Unspecified)
        endif()
        install(TARGETS "${IPA_TARGET}" LIBRARY DESTINATION "${IPA_INSTALL_DESTINATION}" COMPONENT "${IPA_COMPONENT}")
        # install(TARGETS) above performs strip/RPATH edits first. Resolve prefix
        # and DESTDIR at install time, never at configuration time.
        set(code "set(destination [==[${IPA_INSTALL_DESTINATION}]==])\n")
        string(APPEND code "if(NOT IS_ABSOLUTE \"\${destination}\")\n  set(destination \"\${CMAKE_INSTALL_PREFIX}/\${destination}\")\nendif()\n")
        string(APPEND code "set(MODULE \"\$ENV{DESTDIR}\${destination}/$<TARGET_FILE_NAME:${IPA_TARGET}>\")\n")
        string(APPEND code "set(PRIVATE_KEY [==[${IPA_PRIVATE_KEY}]==])\nset(OPENSSL [==[${openssl}]==])\nset(FORCE_SIGN TRUE)\ninclude([==[${script}]==])\n")
        set(install_script "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/${IPA_TARGET}-ipa-install-$<CONFIG>.cmake")
        file(GENERATE OUTPUT "${install_script}" CONTENT "${code}")
        install(SCRIPT "${install_script}" COMPONENT "${IPA_COMPONENT}")
    endif()
endfunction()
