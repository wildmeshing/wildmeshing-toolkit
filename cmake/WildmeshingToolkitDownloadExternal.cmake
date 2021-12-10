################################################################################

include(DownloadProject)

# With CMake 3.8 and above, we can hide warnings about git being in a
# detached head by passing an extra GIT_CONFIG option
if(NOT (${CMAKE_VERSION} VERSION_LESS "3.8.0"))
    set(WILDMESHING_TOOLKIT_EXTRA_OPTIONS "GIT_CONFIG advice.detachedHead=false")
else()
    set(WILDMESHING_TOOLKIT_EXTRA_OPTIONS "")
endif()

# Shortcut function
function(wildmeshing_toolkit_download_project name)
    download_project(
        PROJ         ${name}
        SOURCE_DIR   ${WILDMESHING_TOOLKIT_EXTERNAL}/${name}
        DOWNLOAD_DIR ${WILDMESHING_TOOLKIT_EXTERNAL}/.cache/${name}
        QUIET
        ${WILDMESHING_TOOLKIT_EXTRA_OPTIONS}
        ${ARGN}
    )
endfunction()

################################################################################

## libigl
function(wildmeshing_toolkit_download_libigl)
    wildmeshing_toolkit_download_project(libigl
        GIT_REPOSITORY https://github.com/libigl/libigl.git
        GIT_TAG        45cfc79fede992ea3923ded9de3c21d1c4faced1
    )
endfunction()

## Sanitizers
function(wildmeshing_toolkit_download_sanitizers)
    wildmeshing_toolkit_download_project(sanitizers-cmake
        GIT_REPOSITORY https://github.com/arsenm/sanitizers-cmake.git
        GIT_TAG        6947cff3a9c9305eb9c16135dd81da3feb4bf87f
    )
endfunction()

## spdlog
function(wildmeshing_toolkit_download_spdlog)
    wildmeshing_toolkit_download_project(spdlog
        URL         https://github.com/gabime/spdlog/archive/v1.3.1.tar.gz
        URL_MD5     3c17dd6983de2a66eca8b5a0b213d29f
    )
endfunction()

function(wildmeshing_toolkit_download_delaunay_psm)
    wildmeshing_toolkit_download_project(delaunay_psm
        GIT_REPOSITORY https://github.com/wildmeshing/Delaunay_psm.git
        GIT_TAG        3ecb4d6a1ccdcf5f3325a67470f18ae5b4074343
    )
endfunction()
