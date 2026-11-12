# PolySolve (https://github.com/polyfem/polysolve)
# License: MIT

if(TARGET polysolve::polysolve)
    return()
endif()

message(STATUS "Third-party: creating target 'polysolve::polysolve'")

# # TODO: this requires a conflicting version of Eigen. Reenable when Eigen 3.4+ is available.
# set(POLYSOLVE_WITH_ACCELERATE OFF CACHE BOOL "Enable Apple Accelerate" FORCE)

# TBB FIRST. With no TBB::tbb defined yet, polysolve's own recipe adds oneTBB 2021.6 while it
# configures, ipc-toolkit later adds its 2022.3.0 as well, and ipc's scalable_ccd fails
# -Werror=extra-semi on TBB's headers, which reach it as -I rather than -isystem. The same
# 2022.3.0 headers fail the same way when it is polysolve's recipe that adds them; why the
# recipe changes -I against -isystem is not known. recipes/onetbb.cmake is ipc-toolkit's own
# recipe, so the build keeps the one oneTBB, 2022.3.0, that it had before.
include(onetbb)

include(CPM)
# 621e8fb3 is polyfem's pin, so a build with both has one polysolve. It is 21 commits after the
# previous pin 1b01e6cec1 (polysolve#109, the Eigen 5 fix for MSVC).
#
# wmtk alone solves only small dense systems (DenseNewton with Eigen::LDLT, optimization/solver.hpp),
# so every sparse solver is off. polyfem's simulations need them (polyfem_sim asks for
# AccelerateLDLT, PardisoLDLT or CholmodDecomposition): with WMTK_WITH_POLYFEM, polysolve's own
# defaults, as a polyfem build has them, with their rule for CMake 4: AMGCL's CMakeLists sets a
# policy CMake 4 removed (CMP0058 OLD), so polysolve and polyfem, as top-level projects, turn it
# off there.
#
# POLYSOLVE_WITH_MPI OFF: from 621e8fb3 polysolve compiles its CPU hybrid solver whenever MPI is
# on, and that solver needs hypre, which is off below. MPI is on by default off Windows (it is
# nano-mpi, threads of this process, so nothing has to be installed for it to be found).
if(WMTK_WITH_POLYFEM)
    set(WMTK_POLYSOLVE_OPTIONS)
    if(CMAKE_VERSION VERSION_GREATER_EQUAL "4.0.0")
        list(APPEND WMTK_POLYSOLVE_OPTIONS "POLYSOLVE_WITH_AMGCL OFF")
    endif()
else()
    set(WMTK_POLYSOLVE_OPTIONS
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
endif()
CPMAddPackage(
    NAME polysolve
    GITHUB_REPOSITORY polyfem/polysolve
    GIT_TAG 621e8fb3889717a33c4fa33a608052e9d7459e63
    OPTIONS ${WMTK_POLYSOLVE_OPTIONS}
)

set_target_properties(polysolve PROPERTIES FOLDER third_party)
set_target_properties(polysolve_linear PROPERTIES FOLDER third_party)