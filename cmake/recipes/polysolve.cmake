# PolySolve (https://github.com/polyfem/polysolve)
# License: MIT

if(TARGET polysolve::polysolve)
    return()
endif()

message(STATUS "Third-party: creating target 'polysolve::polysolve'")

# # TODO: this requires a conflicting version of Eigen. Reenable when Eigen 3.4+ is available.
# set(POLYSOLVE_WITH_ACCELERATE OFF CACHE BOOL "Enable Apple Accelerate" FORCE)

include(CPM)
# polysolve main f6008a2 (which has polysolve#126: with /advanced/track_info off, see
# wmtk/optimization/solver.hpp, a solve skips its info json and timings) plus polysolve#131: a
# solver created again with the same parameters skips their JSON validation (one solver per mesh is
# created for the vertex smoothing, 40 ms each before), and minimize() allocates and times less,
# with the same iterates. Move to the merge commit once #131 is in.
# Since 1b01e6c (the Eigen 5 fix) main also gained an MPI option that defaults to ON off Windows,
# hence POLYSOLVE_WITH_MPI OFF below.
CPMAddPackage(
    NAME polysolve
    GITHUB_REPOSITORY polyfem/polysolve
    GIT_TAG 37973660173e45c4cb0066e31371599fc1a6ea7f
    OPTIONS
    "POLYSOLVE_WITH_MPI OFF"
    "POLYSOLVE_WITH_ACCELERATE OFF"
    "POLYSOLVE_WITH_CHOLMOD OFF"
    "POLYSOLVE_WITH_UMFPACK OFF"
    "POLYSOLVE_WITH_SUPERLU OFF"
    "POLYSOLVE_WITH_SPQR OFF"
    "POLYSOLVE_WITH_MKL OFF"
    "POLYSOLVE_WITH_CUSOLVER OFF"
    "POLYSOLVE_WITH_PARDISO OFF"
    "POLYSOLVE_WITH_HYPRE OFF"
    "POLYSOLVE_WITH_AMGCL OFF"
    "POLYSOLVE_WITH_SPECTRA OFF"
)

set_target_properties(polysolve PROPERTIES FOLDER third_party)
set_target_properties(polysolve_linear PROPERTIES FOLDER third_party)