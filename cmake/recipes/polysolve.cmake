# PolySolve (https://github.com/polyfem/polysolve)
# License: MIT

if(TARGET polysolve::polysolve)
    return()
endif()

message(STATUS "Third-party: creating target 'polysolve::polysolve'")

# # TODO: this requires a conflicting version of Eigen. Reenable when Eigen 3.4+ is available.
# set(POLYSOLVE_WITH_ACCELERATE OFF CACHE BOOL "Enable Apple Accelerate" FORCE)

include(CPM)
# polysolve main 623a7ea plus polysolve#126, which adds /advanced/track_info: off (see
# wmtk/optimization/solver.hpp), a solve skips its info json and timings, about a third of a Newton
# solve of a few unknowns, with the same iterates. Move to the merge commit once #126 is in.
# Since the previous pin (1b01e6c, the Eigen 5 fix) main also gained an MPI option that defaults to
# ON off Windows, hence POLYSOLVE_WITH_MPI OFF below.
CPMAddPackage(
    NAME polysolve
    GITHUB_REPOSITORY polyfem/polysolve
    GIT_TAG 98d2db24d9d1b7313d5ae75599d641ce650a2627
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