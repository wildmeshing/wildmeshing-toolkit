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

# MSVC only defines M_PI when _USE_MATH_DEFINES is set before <cmath>, and ipc-toolkit does not
# set it -- so high_order_contact/collisions/high_order_quadrature.hpp fails to compile with
# `error C2065: 'M_PI': undeclared identifier`. PUBLIC because the offending use is in a header
# wmtk includes, so the definition has to reach our translation units too. Invisible on
# macOS/Linux, where the libc++ and libstdc++ <cmath> define M_PI unconditionally -- which is why
# this only ever showed up on the Windows jobs.
target_compile_definitions(ipc_toolkit PUBLIC _USE_MATH_DEFINES)

# ipc-toolkit's high_order_contact headers use std::array, std::uint*_t and <algorithm> without
# including them, and get away with it on libc++ and libstdc++, which pull those in transitively
# through other standard headers. MSVC's do not, so the Windows build fails with things like
#
#     high_order_contact_parameters.hpp(13): error C2079:
#       'ipc::FaceQuadPoint::lambda' uses undefined class 'std::array<double,3>'
#
# Force-include them rather than fix the headers here: this is a mirror we do not want to fork
# for whitespace-level changes, and the alternative is one CI round trip per missing include.
# PUBLIC for the same reason as _USE_MATH_DEFINES -- the offending declarations are in headers
# wmtk includes, so our translation units need the same treatment.
#
# TODO: push the includes upstream to wildmeshing/ipc-toolkit and drop this.
if(MSVC)
    target_compile_options(ipc_toolkit PUBLIC /FIarray /FIcstdint /FIalgorithm)
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
