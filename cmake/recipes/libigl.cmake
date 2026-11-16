if(TARGET igl::core)
    return()
endif()

message(STATUS "Third-party: creating target 'igl::core'")


# aeeea9b4 is polyfem's pin, so a build with both has one libigl. It split
# igl/predicates/predicates.h into one header per predicate, and ipc-toolkit (src/ipc/ipc.cpp,
# src/ipc/geometry/intersection.cpp) still includes predicates.h: the patch puts back a
# predicates.h that includes the split headers. It is polyfem's
# cmake/patches/igl_predicates_compat.patch, verbatim.
include(FetchContent)
CPMAddPackage(
    libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG aeeea9b416a5b5474cb9d12c07ff1238a4c83ba1
    PATCHES "${CMAKE_CURRENT_LIST_DIR}/../patches/igl_predicates_compat.patch"
    OPTIONS
        # LIBIGL_PREDICATES: off since the toolkit moved its exact predicates to
        # Indirect_Predicates (see src/wmtk/utils/predicates.cpp). Nothing links
        # igl::predicates any more, and leaving it on still builds Shewchuk.
        LIBIGL_PREDICATES OFF
        # LIBIGL_COPYLEFT_TETGEN ON
)

# include(eigen)
FetchContent_MakeAvailable(libigl)