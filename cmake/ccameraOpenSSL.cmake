# SPDX-License-Identifier: CC0-1.0
# FindOpenSSL before CMake 3.20 cannot parse OpenSSL 3's version macros, and
# older releases omit compression dependencies from static imported targets.
# Keep OpenSSL::Crypto as the canonical target while filling those gaps.
function(ccamera_ipa_openssl_requirements)
    file(STRINGS "${OPENSSL_INCLUDE_DIR}/openssl/opensslv.h" major_line
        REGEX "^#[ \t]*define[ \t]+OPENSSL_VERSION_MAJOR[ \t]+[0-9]+")
    if(NOT major_line MATCHES "OPENSSL_VERSION_MAJOR[ \t]+([0-9]+)" OR CMAKE_MATCH_1 LESS 3)
        message(FATAL_ERROR "IPA verification requires OpenSSL 3 or newer")
    endif()
    if(NOT OPENSSL_CRYPTO_LIBRARY MATCHES "\\.a$")
        return()
    endif()
    get_target_property(added OpenSSL::Crypto CCAMERA_IPA_STATIC_DEPS_ADDED)
    if(added)
        return()
    endif()
    get_filename_component(directory "${OPENSSL_CRYPTO_LIBRARY}" DIRECTORY)
    set(metadata "${directory}/pkgconfig/libcrypto.pc")
    if(EXISTS "${metadata}")
        find_package(PkgConfig QUIET)
        if(PKG_CONFIG_FOUND)
            # Query metadata beside the selected TARGET library, rather than a
            # potentially unrelated host openssl.pc. Respect cross pkg-config
            # sysroot/LIBDIR settings supplied by the toolchain.
            execute_process(COMMAND "${CMAKE_COMMAND}" -E env
                "PKG_CONFIG_PATH=${directory}/pkgconfig:$ENV{PKG_CONFIG_PATH}"
                "${PKG_CONFIG_EXECUTABLE}" --static --libs libcrypto
                RESULT_VARIABLE result OUTPUT_VARIABLE flags ERROR_VARIABLE error)
            if(NOT result EQUAL 0)
                message(FATAL_ERROR "Cannot resolve static libcrypto dependencies: ${error}")
            endif()
        else()
            # pkg-config is optional. Standard OpenSSL puts dl, pthread and
            # compression libraries directly in Libs.private.
            file(STRINGS "${metadata}" flags REGEX "^Libs.private:")
            string(REGEX REPLACE "^Libs.private:[ \t]*" "" flags "${flags}")
            file(STRINGS "${metadata}" requires REGEX "^Requires.private:[ \t]*[^ \t]")
            if(requires)
                message(FATAL_ERROR "This static libcrypto requires pkg-config to resolve ${requires}")
            endif()
        endif()
        separate_arguments(flags UNIX_COMMAND "${flags}")
        list(REMOVE_ITEM flags -lcrypto)
        set_property(TARGET OpenSSL::Crypto APPEND PROPERTY INTERFACE_LINK_LIBRARIES ${flags})
    endif()
    set_property(TARGET OpenSSL::Crypto PROPERTY CCAMERA_IPA_STATIC_DEPS_ADDED TRUE)
endfunction()
