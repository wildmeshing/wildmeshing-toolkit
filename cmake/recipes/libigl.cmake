if(TARGET igl::core)
    return()
endif()

message(STATUS "Third-party: creating target 'igl::core'")


include(FetchContent)
CPMAddPackage(
    libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    # 2.6.0 (2025-05-14). polyfem needs the 2023 calling convention of igl::predicates::ear_clipping
    # (four arguments), and the ipc-toolkit fork includes igl/predicates/predicates.h, which
    # polyfem's own 2026 pin no longer ships; 2.6.0 is also the ipc-toolkit fork's own pin. The
    # previous pin (3ea7f948, October 2022) needed WindingNumberAABB's old three-parameter template
    # in src/wmtk/utils/WindingNumber.hpp and lacked <cassert> in mat_max/mat_min, which the Eigen
    # pin below exposed. Measured 2026-09-16.
    GIT_TAG 40e7900ccbd767f1f360e0eb10f0f1a6432e0993
    OPTIONS
        # LIBIGL_PREDICATES: the toolkit itself moved its exact predicates to Indirect_Predicates
        # (see src/wmtk/utils/predicates.cpp), but ipc-toolkit (topological_offset) and polyfem
        # (polyfem_ops) both link igl::predicates. The earlier unquoted `LIBIGL_PREDICATES OFF` was
        # never in effect: every default build has produced igl::predicates (lib/libpredicates.a),
        # and a quoted OFF breaks ipc-toolkit's link at configure time. Measured 2026-09-16.
        "LIBIGL_PREDICATES ON"
        # LIBIGL_COPYLEFT_TETGEN ON
)

# include(eigen)
FetchContent_MakeAvailable(libigl)
