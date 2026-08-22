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
# default (DistanceTypeConfig::use_standard_ is false). high_order_contact -- the subtree the
# offset potential actually evaluates -- calls distance_type from its collision builder and its
# pair-distance templates, so that fallback would land squarely on our own code, in exactly the
# near-degenerate configurations an offset surface produces. A version bump changes nothing;
# dropping the predicates changes what the potential is computed from.
#
# Options are ipc's own, verbatim, so this differs from what ipc would have configured in the
# version and in nothing else.

if(TARGET geogram::geogram)
    return()
endif()

message(STATUS "Third-party: creating target 'geogram::geogram'")

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
