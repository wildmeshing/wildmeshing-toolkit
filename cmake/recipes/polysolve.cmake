# PolySolve (https://github.com/polyfem/polysolve)
# License: MIT

if(TARGET polysolve::polysolve)
    return()
endif()

message(STATUS "Third-party: creating target 'polysolve::polysolve'")

# # TODO: this requires a conflicting version of Eigen. Reenable when Eigen 3.4+ is available.
# set(POLYSOLVE_WITH_ACCELERATE OFF CACHE BOOL "Enable Apple Accelerate" FORCE)

include(CPM)
CPMAddPackage(
    NAME polysolve
    GITHUB_REPOSITORY polyfem/polysolve
    GIT_TAG 57dbeadcb65932a38508e3a5a07e98732a1c50bc
    OPTIONS
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