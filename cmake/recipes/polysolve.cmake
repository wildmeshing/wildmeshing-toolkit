# PolySolve (https://github.com/polyfem/polysolve)
# License: MIT

if(TARGET polysolve::polysolve)
    return()
endif()

# Polysolve options for enabling/disabling optional libraries
option(POLYSOLVE_WITH_ACCELERATE OFF)
option(POLYSOLVE_WITH_CHOLMOD OFF)
option(POLYSOLVE_WITH_UMFPACK OFF)
option(POLYSOLVE_WITH_SUPERLU OFF)
option(POLYSOLVE_WITH_MKL OFF)
option(POLYSOLVE_WITH_CUSOLVER OFF)
option(POLYSOLVE_WITH_PARDISO OFF)
option(POLYSOLVE_WITH_HYPRE OFF)
option(POLYSOLVE_WITH_AMGCL OFF)
option(POLYSOLVE_WITH_SPECTRA OFF)

message(STATUS "Third-party: creating target 'polysolve'")

# TODO: this requires a conflicting version of Eigen. Reenable when Eigen 3.4+ is available.
set(POLYSOLVE_WITH_ACCELERATE OFF CACHE BOOL "Enable Apple Accelerate" FORCE)

include(CPM)
CPMAddPackage("gh:polyfem/polysolve#389e36b784ba30070dfcb5a64a43a4be11681315")
