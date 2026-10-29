# Geogram (https://github.com/BrunoLevy/geogram)
# License: BSD 3-Clause
#
# wmtk does not use geogram itself -- fast-envelope dropped it when the three predicate backends
# collapsed onto Indirect_Predicates. It is here only because ipc-toolkit links it, for the exact
# expansion types behind ipc's point-edge and point-triangle distance-type classification
# (src/ipc/distance/distance_type.cpp, GEO::expansion_nt / GEO::vec3E / GEO::PCK).
#
# SO WHY A RECIPE AT ALL, RATHER THAN LETTING IPC BRING ITS OWN? ipc pins geogram 1.9.8, whose
# bundled third_party/PoissonRecon/Hash.h opens with
#
#     #ifdef _MSC_VER
#     #include <hash_map>
#
# and VS 2026 no longer ships that header, so the Windows build dies with
#
#     Hash.h(5): fatal error C1083: Cannot open include file: 'hash_map'
#
# on both Debug and Release. PoissonRecon is compiled unconditionally in geogram's third_party
# CMakeLists -- there is no option that excludes it -- so the version is the only lever. Upstream
# DELETED that header in v1.9.10, which is why this pins it there and nowhere else.
#
# The alternative was IPC_TOOLKIT_WITH_GEOGRAM OFF, which does build. It is rejected because it is
# not free: ipc falls back to its analytic distance-type classification, and the exact path is the
# default (DistanceTypeConfig::use_standard() is false). esp -- the subtree the
# offset potential actually evaluates -- calls distance_type from its collision builder and its
# pair-distance templates, so that fallback would land squarely on our own code, in exactly the
# near-degenerate configurations an offset surface produces. A version bump changes nothing;
# dropping the predicates changes what the potential is computed from.
#
# Options are ipc's own, verbatim, so this differs from what ipc would have configured in the
# version and in the platform below, and in nothing else.
#
# STATIC, NOT SHARED. geogram does not read BUILD_SHARED_LIBS: its build platform decides, and the
# platform it detects on macOS and Linux (cmake/geo_detect_platform.cmake) is a "-dynamic" one,
# whose config.cmake sets VORPALINE_BUILD_DYNAMIC, so cmake/geogram.cmake forces BUILD_SHARED_LIBS
# TRUE for the subtree. Everything else here is static (BUILD_SHARED_LIBS OFF, top-level
# CMakeLists.txt), so libgeogram was the only shared library in the build, and the Python wheel
# does not carry it: `pip install` succeeded and `import wildmeshing` then died with
#
#     ImportError: ... Library not loaded: .../wheel/platlib/lib/libgeogram.1.10.0.dylib
#
# Pick the static platform matching what geogram would have detected. Each pair differs only in
# the VORPALINE_BUILD_DYNAMIC line, except on Linux off x86, where geogram has no static
# "nonx86" platform: Linux64-gcc-aarch64 is Linux64-gcc without the -m64 that aarch64 g++
# rejects (its GEO_OS_LINUX_AARCH64 define is read nowhere in 1.9.10), and clang, which accepts
# -m64 on any 64-bit target, gets Linux64-clang. Windows needs nothing: geogram's MSVC and MinGW
# defaults (Win-vs-generic, Win64-gcc) are static already. A "-dynamic" value already in the
# cache -- from geogram's own detection in an earlier configure of the same build directory -- is
# replaced too; another static platform given with -DVORPALINE_PLATFORM is kept.

if(TARGET geogram::geogram)
    return()
endif()

message(STATUS "Third-party: creating target 'geogram::geogram'")

if(NOT VORPALINE_PLATFORM OR VORPALINE_PLATFORM MATCHES "-dynamic")
    set(_wmtk_geogram_platform "")
    if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        set(_wmtk_geogram_platform "Darwin-clang")
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
            if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
                set(_wmtk_geogram_platform "Linux64-gcc")
            elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
                set(_wmtk_geogram_platform "Linux64-clang")
            endif()
        else()
            if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
                set(_wmtk_geogram_platform "Linux64-gcc-aarch64")
            elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
                set(_wmtk_geogram_platform "Linux64-clang")
            endif()
        endif()
    elseif(VORPALINE_PLATFORM STREQUAL "Win-vs-dynamic-generic")
        set(_wmtk_geogram_platform "Win-vs-generic")
    endif()
    if(_wmtk_geogram_platform)
        set(VORPALINE_PLATFORM "${_wmtk_geogram_platform}" CACHE STRING
            "geogram build platform; wmtk builds geogram static (cmake/recipes/geogram.cmake)" FORCE)
    endif()
endif()

include(CPM)
CPMAddPackage(
    URI "gh:BrunoLevy/geogram@1.9.10"
    OPTIONS
        "GEOGRAM_WITH_GRAPHICS OFF"
        "GEOGRAM_WITH_LEGACY_NUMERICS OFF"
        "GEOGRAM_WITH_HLBFGS OFF"
        "GEOGRAM_WITH_TETGEN OFF"
        "GEOGRAM_WITH_TRIANGLE OFF"
        "GEOGRAM_WITH_LUA OFF"
        "GEOGRAM_LIB_ONLY ON"
)

if(NOT TARGET geogram::geogram AND TARGET geogram)
    add_library(geogram::geogram ALIAS geogram)
endif()

# If a platform slipped past the selection above, say so at configure time -- and for the Python
# package stop there, rather than at `import wildmeshing`.
get_target_property(_wmtk_geogram_type geogram TYPE)
if(NOT _wmtk_geogram_type STREQUAL "STATIC_LIBRARY")
    if(SKBUILD)
        set(_wmtk_geogram_level FATAL_ERROR)
    else()
        set(_wmtk_geogram_level WARNING)
    endif()
    message(${_wmtk_geogram_level}
        "geogram is a ${_wmtk_geogram_type} with VORPALINE_PLATFORM=${VORPALINE_PLATFORM}, and "
        "the Python wheel does not ship a shared libgeogram. Pass a static platform (one without "
        "-dynamic) from geogram's cmake/platforms with -DVORPALINE_PLATFORM.")
endif()

# geogram's GCC platforms (cmake/platforms/Linux-gcc*.cmake) compile it with -fopenmp, and a
# shared libgeogram links libgomp itself; the static archive declares no such dependency, so its
# OpenMP references (basic/process.cpp, OpenNL, PoissonRecon) reach the Python module unresolved.
# A shared module links with undefined symbols allowed, and `import wildmeshing` then fails with
# "undefined symbol: GOMP_critical_end". Link the runtime for everything that links geogram.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    target_link_options(geogram INTERFACE "-fopenmp")
endif()
