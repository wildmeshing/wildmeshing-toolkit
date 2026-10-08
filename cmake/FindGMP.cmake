# Try to find the GNU Multiple Precision Arithmetic Library (GMP)
# See http://gmplib.org/

Message(STATUS "GMP_DIR = ${GMP_DIR}")
Message(STATUS "GMP_DIR (env var) = $ENV{GMP_DIR}")

# Homebrew links every formula's headers into one shared directory (/opt/homebrew/include, or
# /usr/local/include on Intel). Found there, GMP's include directory puts every library Homebrew
# has installed on the include path of everything that links GMP -- as a system directory, ahead
# of the copies this build fetches. A Homebrew Abseil then shadows the Abseil that ipc-toolkit
# fetches for topological_offset: the code compiles against one version and links the other, and
# the link fails on an undefined absl::lts_<version>::hash_internal::MixingHashState::kSeed. Each
# formula also has a prefix of its own (`brew --prefix gmp`) that holds its headers alone; look
# there first.
set(_WMTK_GMP_HOMEBREW_PREFIXES "")
if(APPLE)
    foreach(_prefix "$ENV{HOMEBREW_PREFIX}" /opt/homebrew /usr/local)
        if(_prefix AND EXISTS "${_prefix}/opt/gmp/include/gmp.h")
            list(APPEND _WMTK_GMP_HOMEBREW_PREFIXES "${_prefix}/opt/gmp")
        endif()
    endforeach()
    # A cache from before this lookup existed holds the shared directory; find it again.
    if(_WMTK_GMP_HOMEBREW_PREFIXES AND GMP_INCLUDES MATCHES "^(/opt/homebrew|/usr/local)/include/?$")
        unset(GMP_INCLUDES CACHE)
    endif()
endif()

find_path(GMP_INCLUDES
    NAMES
        gmp.h
    HINTS
        ${GMP_DIR}
        ENV GMP_DIR
        ${_WMTK_GMP_HOMEBREW_PREFIXES}
    PATHS
        ${INCLUDE_INSTALL_DIR}
    PATH_SUFFIXES
        include
)

find_library(GMP_LIBRARIES
    NAMES
        gmp
        libgmp-10
        libgmp
    HINTS
        ${GMP_DIR}
        ENV GMP_DIR
        ${_WMTK_GMP_HOMEBREW_PREFIXES}
    PATHS
        ${LIB_INSTALL_DIR}
    PATH_SUFFIXES
        lib
)

set(GMP_EXTRA_VARS "")
if(WIN32)
    # Find dll file and set IMPORTED_LOCATION to the .dll file
    find_file(GMP_RUNTIME_LIB
        NAMES
            gmp.dll
            gmp-10.dll
            libgmp-10.dll
        HINTS
            ${GMP_DIR}
            ENV GMP_DIR
        PATHS
            ${LIB_INSTALL_DIR}
        PATH_SUFFIXES
            bin
            lib
    )
    list(APPEND GMP_EXTRA_VARS GMP_RUNTIME_LIB)

    message(STATUS "Windows GMP Paths: \n  ${GMP_INCLUDES}\n  ${GMP_LIBRARIES}\n  ${GMP_RUNTIME_LIB}")
else()
    message(STATUS "GMP Paths: \n  ${GMP_INCLUDES}\n  ${GMP_LIBRARIES}")
endif()


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GMP
    REQUIRED_VARS
        GMP_INCLUDES
        GMP_LIBRARIES
        ${GMP_EXTRA_VARS}
    REASON_FAILURE_MESSAGE
        "GMP is not installed on your system. Either install GMP using your preferred package manager, or disable libigl modules that depend on GMP, such as CORK and CGAL. See LibiglOptions.cmake.sample for configuration options. Do not forget to delete your <build>/CMakeCache.txt for the changes to take effect."
)
mark_as_advanced(GMP_INCLUDES GMP_LIBRARIES)

if(GMP_INCLUDES AND GMP_LIBRARIES AND NOT TARGET gmp::gmp)
    if(GMP_RUNTIME_LIB)
        add_library(gmp::gmp SHARED IMPORTED)
    else()
        add_library(gmp::gmp UNKNOWN IMPORTED)
    endif()

    # Set public header location and link language
    set_target_properties(gmp::gmp PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "C"
        INTERFACE_INCLUDE_DIRECTORIES "${GMP_INCLUDES}"
    )

    # Set lib location. On Windows we specify both the .lib and the .dll paths
    if(GMP_RUNTIME_LIB)
        set_target_properties(gmp::gmp PROPERTIES
            IMPORTED_IMPLIB "${GMP_LIBRARIES}"
            IMPORTED_LOCATION "${GMP_RUNTIME_LIB}"
        )
    else()
        set_target_properties(gmp::gmp PROPERTIES
            IMPORTED_LOCATION "${GMP_LIBRARIES}"
        )
    endif()
endif()