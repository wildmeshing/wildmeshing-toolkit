# IPC Toolkit (https://github.com/wildmeshing/ipc-toolkit)
# License: MIT
#
# Public mirror of fsichetti/ipc-toolkit@upstream-merge, published in the wildmeshing
# organisation with the author's permission. What wmtk uses from it is the `high_order_contact`
# subtree: the smooth (offset geometric contact) potential and its analytic gradient and
# Hessian, which the topological_offset component evaluates as its offset field.
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

# Eigen FIRST, and deliberately. ipc-toolkit ships its own recipes/eigen.cmake, which creates the
# same Eigen3::Eigen target and defaults EIGEN_DONT_VECTORIZE to ON -- whichever recipe runs first
# wins, and the loser returns early. Creating the target from wmtk's recipe here means wmtk's
# settings are the ones the whole build gets, including the ABI pin that recipe pins.
include(eigen)

include(CPM)
CPMAddPackage(
    NAME ipc_toolkit
    GITHUB_REPOSITORY wildmeshing/ipc-toolkit
    GIT_TAG 5f301a258bb7cf9cd4e8fb7a348d1089aafcb0e7
    OPTIONS
    "IPC_TOOLKIT_BUILD_TESTS OFF"
    "IPC_TOOLKIT_BUILD_PYTHON OFF"
    "IPC_TOOLKIT_WITH_CUDA OFF"
)

# ipc-toolkit's SIMD branch ends with
#
#     target_compile_definitions(ipc_toolkit PUBLIC EIGEN_DONT_VECTORIZE=1)
#
# -- PUBLIC, with a comment saying the author does not know why making it private crashes. It
# is not optional for us either: high_order_contact, the subtree wmtk actually uses, carries a
# static_assert(packet_traits<double>::size == 1, "Eigen vectorization is NOT disabled!") in
# HighOrderCollisionTemplate's constructor, so building it with vectorization on does not
# compile. IPC_TOOLKIT_WITH_SIMD must therefore stay at its default ON.
#
# On its own that definition splits the build's Eigen ABI in half, because it reaches only the
# targets that link ipc while core wildmeshing_toolkit keeps the aligned layout -- which cost a
# day of green-looking local runs and red CI. recipes/eigen.cmake pins
# EIGEN_MAX_STATIC_ALIGN_BYTES for everyone precisely so that this definition can no longer do
# that: the pin wins over Eigen's inference, so ipc gets the scalar packets it demands while
# every fixed-size Eigen type keeps one alignment across the whole binary.
#
# That pin is load-bearing, and the symptom if it is lost is an alignment abort in tests that
# have nothing to do with either the offset or ipc. Check it here, next to the cause.
get_target_property(WMTK_EIGEN_DEFS Eigen3_Eigen INTERFACE_COMPILE_DEFINITIONS)
if(NOT WMTK_EIGEN_DEFS MATCHES "EIGEN_MAX_STATIC_ALIGN_BYTES")
    message(FATAL_ERROR
        "Eigen3::Eigen is missing the EIGEN_MAX_STATIC_ALIGN_BYTES pin from "
        "cmake/recipes/eigen.cmake. Without it ipc-toolkit's PUBLIC EIGEN_DONT_VECTORIZE gives "
        "the targets that link ipc a different alignment for Vector2d/Vector4d/Vector3r than "
        "the rest of the build, and Eigen's unaligned-array assert aborts "
        "wmtk_test_manifold_extraction and wmtk_test_topological_offset.")
endif()

# ipc-toolkit links its hash-map backends PRIVATE, but leaks them through the PUBLIC header
# ipc/utils/unordered_map_and_set.hpp, so any consumer including it fails to find
# <tsl/robin_map.h> / <absl/hash/hash.h>. Upstream's own test target works around this by
# linking them again; do it once here instead, so every wmtk consumer inherits the fix.
foreach(IPC_LEAKED_DEP IN ITEMS tsl::robin_map absl::hash)
    if(TARGET ${IPC_LEAKED_DEP})
        target_link_libraries(ipc_toolkit PUBLIC ${IPC_LEAKED_DEP})
    endif()
endforeach()
