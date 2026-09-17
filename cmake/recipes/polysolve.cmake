# PolySolve (https://github.com/polyfem/polysolve)
# License: MIT

if(TARGET polysolve::polysolve)
    return()
endif()

message(STATUS "Third-party: creating target 'polysolve::polysolve'")

# Apple Accelerate: the sparse solver the polyfem_ops reference runs used, so the in-process solve
# matches them. Needs the Eigen commit pinned in recipes/eigen.cmake. Apple only: polysolve resolves
# it through BLA_VENDOR=Apple, which does not exist elsewhere.
if(APPLE)
    set(WMTK_POLYSOLVE_WITH_ACCELERATE ON)
else()
    set(WMTK_POLYSOLVE_WITH_ACCELERATE OFF)
endif()

include(CPM)
CPMAddPackage(
    NAME polysolve
    GITHUB_REPOSITORY polyfem/polysolve
    GIT_TAG 231f7e8ed6fc36aad41f21d9b3bf0cdf7816f78c
    OPTIONS
    "POLYSOLVE_WITH_ACCELERATE ${WMTK_POLYSOLVE_WITH_ACCELERATE}"
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