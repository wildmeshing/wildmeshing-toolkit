if(TARGET igl::core)
    return()
endif()

message(STATUS "Third-party: creating target 'igl::core'")


# aeeea9b4 is polyfem's pin, so a build with both has one libigl. It split
# igl/predicates/predicates.h into one header per predicate, and ipc-toolkit (src/ipc/ipc.cpp,
# src/ipc/geometry/intersection.cpp) still includes predicates.h: the patch puts back a
# predicates.h that includes the split headers. It is polyfem's
# cmake/patches/igl_predicates_compat.patch, verbatim.
#
# With WMTK_WITH_POLYFEM, polyfem's own libigl patches instead (they include the same
# predicates.h), read from polyfem's sources: with hypre on (recipes/polysolve.cmake), Shewchuk's
# predicates.h, vendored under another name, no longer clashes with hypre's, and the SVD preamble
# compiles on Linux.
if(WMTK_WITH_POLYFEM)
    include(polyfem_source)
    file(GLOB WMTK_LIBIGL_PATCHES "${polyfem_SOURCE_DIR}/cmake/patches/igl_*.patch")
else()
    set(WMTK_LIBIGL_PATCHES "${CMAKE_CURRENT_LIST_DIR}/../patches/igl_predicates_compat.patch")
endif()
include(FetchContent)
CPMAddPackage(
    libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG aeeea9b416a5b5474cb9d12c07ff1238a4c83ba1
    PATCHES ${WMTK_LIBIGL_PATCHES}
    OPTIONS
        # ipc-toolkit (and polyfem) link igl::predicates. This line read `LIBIGL_PREDICATES OFF`,
        # unquoted, which CPM takes as two options, LIBIGL_PREDICATES and OFF, each set ON: the
        # predicates were on all along. The toolkit's own exact predicates are
        # Indirect_Predicates (src/wmtk/utils/predicates.cpp).
        "LIBIGL_PREDICATES ON"
        # LIBIGL_COPYLEFT_TETGEN ON
)

# include(eigen)
FetchContent_MakeAvailable(libigl)