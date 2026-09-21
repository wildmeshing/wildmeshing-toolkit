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
    # 1b01e6cec = 231f7e8 + one upstream commit, taken for its fix of the MSVC build against the Eigen
    # commit pinned in recipes/eigen.cmake: 231f7e8 pre-declares Eigen::SparseQR with Eigen 3.4.0's
    # template-parameter names, which that Eigen commit renamed, and MSVC rejects the mismatch
    # (C2653/C2976 in linear/Solver.cpp; both Windows CI jobs failed, run 35389234357). Its other
    # changes: an option that is off by default (allow_non_grad_convergence, which only picks the log
    # level of the final message), and polysolve's own Eigen/json recipes, which wmtk's recipes preempt.
    GIT_TAG 1b01e6cec13c6813ae48846d1f98654bbbf1402b
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