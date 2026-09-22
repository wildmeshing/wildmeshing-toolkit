# PolyFEM (https://github.com/polyfem/polyfem)
# License: MIT
#
# Pinned to the tip of the `uday/imagesim` branch (2026-05-01), the version the pysimwild polyfem
# operations were written against and the reference binary for the port's byte-identical checks.
# Upstream main lacks three things those operations rely on: the collision body-id pair filter
# (`contact/collision_pairs`, `collision_mesh/collision_body_ids`), the `input/data/state` warm
# start between outer iterations, and acceptance of those keys under strict validation. Its own
# polysolve pin is an ancestor of ours with no solver-API change, and both pin Eigen 3.4.0; main's
# polysolve wants Eigen 5.
#
# polyfem's dependency recipes all open with `if(TARGET ...) return()`, so it compiles against OUR
# polysolve, ipc-toolkit, libigl, geogram, paraviewo, spdlog and jse, which the top-level
# CMakeLists creates first. Three of ours need a switch flipped for that, done by WMTK_WITH_POLYFEM.
#
# Options mirror the reference binary's build: tests, the embedded Python interpreter and the
# adjoint-optimization module off (one file of the latter trips an Eigen 3.4.0 sparse-sort path the
# macOS 27 SDK's libc++ rejects). In-timestep remeshing stays off because it is the one option that
# makes polyfem depend on wmtk, which would be a cycle.
#
# To build against a local checkout instead of downloading: -DCPM_polyfem_SOURCE=/path/to/polyfem

if(TARGET polyfem::polyfem)
    return()
endif()

message(STATUS "Third-party: creating target 'polyfem::polyfem'")

# polyfem sets CMAKE_CXX_STANDARD only as a top-level project; its `units` dependency reads the
# variable unconditionally (`if(${CMAKE_CXX_STANDARD} GREATER 16)` fails on an empty value).
# Directory-scoped here, so it reaches polyfem and its dependencies and nothing else of ours.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CPM)
CPMAddPackage(
    NAME polyfem
    GITHUB_REPOSITORY polyfem/polyfem
    # c7669ebb7 = b0d511c4b (branch tip 2026-05-01) + two commits of 2026-09-16: spdlog 1.17 / fmt 11
    # compatibility, and use_rest_shape_measure set only when the ipc-toolkit provides the field;
    # + three of 2026-09-18 the in-process backend needs: each contact form keeps its collision-set
    # cache in the form (it was a process-wide static), ALSolver records the termination status
    # of the solver that actually ran, which check_polyfem_success reads, and the constraints and
    # the collision proxy can be handed to a State in memory instead of through files;
    # + one of 2026-09-22: SlimSmooth.cpp declares the SSE intrinsics libigl's SVD code uses, without
    # which every x86-64 build failed (ipc-toolkit's EIGEN_DONT_VECTORIZE keeps Eigen from
    # including them).
    GIT_TAG c7669ebb781797406ec3f35da8f2e437ad2d4cae
    OPTIONS
    "POLYFEM_WITH_TESTS OFF"
    "POLYFEM_WITH_PYTHON OFF"
    "POLYFEM_WITH_OPTIMIZATION OFF"
    "POLYFEM_WITH_ITR OFF"
    "POLYFEM_WITH_APP OFF"
    # bezier (conservative element-inversion check) pulls a package whose CMake creates a target
    # named indirectPredicates, the same name FastEnvelope's recipe already created for wmtk.
    # The reference binary has it on, but nothing the ported operations run touches it: they use
    # the default discrete inversion check.
    "POLYFEM_WITH_BEZIER OFF"
)

# polyfem's input spec includes polysolve's linear- and nonlinear-solver spec files from the
# polysolve source root. polyfem's own polysolve pin exports that path as this macro; ours no
# longer does, so define it here from the CPM source directory of the polysolve WE added.
target_compile_definitions(polyfem PUBLIC POLYSOLVE_JSON_SPEC_DIR="${polysolve_SOURCE_DIR}")

set_target_properties(polyfem PROPERTIES FOLDER third_party)
