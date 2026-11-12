# IPC Toolkit (https://github.com/ipc-sim/ipc-toolkit)
# License: MIT
#
# Upstream, at polyfem's pin, so a build with both has one ipc. The esp work wmtk used to take
# from the wildmeshing mirror of fsichetti/ipc-toolkit@upstream-merge (626e27e1) is upstream as
# "Extremum Sum Potential (#259)", 7017fa9d. What wmtk uses from it is the `esp` subtree: the
# Extremum-Sum Potential evaluated at arbitrary points (ArbitraryPointESP) and its analytic
# gradient and Hessian, which the topological_offset component evaluates as its offset field.
#
# Only the topological_offset component links this. Core wmtk::toolkit does not, so nothing
# here reaches tetwild, triwild or simwild.

if(TARGET ipc::toolkit)
    return()
endif()

message(STATUS "Third-party: creating target 'ipc::toolkit'")

# wmtk asks for C++17 per target (target_compile_features), leaving CMAKE_CXX_STANDARD unset.
# Abseil, which ipc-toolkit pulls in for its hash functions, probes the compiler's DEFAULT
# standard at configure time and hard-errors when that is below 17 -- so a per-target request is
# invisible to it. Set the directory-scoped default for the subtree ipc-toolkit is configured in.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# EIGEN AND GEOGRAM FIRST, and deliberately. ipc-toolkit ships its own recipe for each, and both
# recipes -- theirs and ours -- open with `if(TARGET ...) return()`, so whichever runs first wins
# and the loser is a no-op. Creating both targets here means wmtk's settings are the ones the
# whole build gets: the Eigen ABI pin (recipes/eigen.cmake) and geogram at a version whose
# bundled PoissonRecon still compiles on current MSVC (recipes/geogram.cmake). Each of those
# files explains what breaks without it; both broke the build in ways that surface a long way
# from here.
include(eigen)
include(geogram)

include(CPM)
CPMAddPackage(
    NAME ipc_toolkit
    GITHUB_REPOSITORY ipc-sim/ipc-toolkit
    GIT_TAG dff05854d74617f1e21ccb2bb321c45d7b4c6802
    OPTIONS
    "IPC_TOOLKIT_BUILD_TESTS OFF"
    "IPC_TOOLKIT_BUILD_PYTHON OFF"
    "IPC_TOOLKIT_WITH_CUDA OFF"
    "IPC_TOOLKIT_WITH_SIMD OFF"
)

# SIMD OFF, so that every binary has one Eigen configuration. ipc-toolkit's SIMD branch compiles
# it for the host CPU (its FindSIMD settles on -march=native) and ends with
#
#   target_compile_definitions(ipc_toolkit PUBLIC EIGEN_DONT_VECTORIZE=1)
#
# which reaches only what links ipc: the topological_offset component and, through it, wmtk_app,
# the Python module, manifold_extraction and (WMTK_WITH_POLYFEM) polyfem. Core
# wildmeshing_toolkit and the other components stay vectorized. The two halves then instantiate
# the same Eigen templates with different arithmetic under the same names, the linker keeps one
# copy of each, and which arithmetic tetwild or simwild runs depends on the link order. The Python
# module with polyfem built in links polyfem first: on macOS, core wmtk, tetwild and simwild took
# 25 Eigen functions (among them ColPivHouseholderQR<Matrix3d>::computeInPlace and
# general_matrix_vector_product) and 103 igl ones from polyfem's scalar objects, and
# tetwild_thingi_100827, tetwild_thingi_366725 and simwild_double_sphere_notop_3d no longer
# matched wmtk_app (smoothing's Newton solves took other iteration counts).
#
# With SIMD off ipc defines no Eigen macro and adds no instruction-set flag, so the whole build is
# vectorized, as core wmtk already was. Measured on macOS against SIMD on: none of the 53
# deterministic registered integration configs changes its output (tetwild_octocat and
# tetwild_crown run on 10 threads), and all 55 match between the Python module and wmtk_app. The
# cost is ipc's xsimd kernels (distances, barrier, tangent bases): ipc uses its scalar code for
# them. The wildmeshing fork's esp (626e27e1) static_asserted that Eigen vectorization was off,
# which kept SIMD on; upstream's (7017fa9d, here at dff05854) does not.

# ipc-toolkit links its hash-map backends PRIVATE, but leaks them through the PUBLIC header
# ipc/utils/unordered_map_and_set.hpp, so any consumer including it fails to find
# <tsl/robin_map.h> / <absl/hash/hash.h>. Upstream's own test target works around this by
# linking them again; do it once here instead, so every wmtk consumer inherits the fix.
foreach(IPC_LEAKED_DEP IN ITEMS tsl::robin_map absl::hash)
    if(TARGET ${IPC_LEAKED_DEP})
        target_link_libraries(ipc_toolkit PUBLIC ${IPC_LEAKED_DEP})
    endif()
endforeach()
