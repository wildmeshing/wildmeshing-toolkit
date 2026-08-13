if(TARGET igl::core)
    return()
endif()

message(STATUS "Third-party: creating target 'igl::core'")


include(FetchContent)
CPMAddPackage(
    libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG 3ea7f9480967fcf6bf02ce9b993c0ea6d2fc45f6
    OPTIONS
        # LIBIGL_PREDICATES: off since the toolkit moved its exact predicates to
        # Indirect_Predicates (see src/wmtk/utils/predicates.cpp). Nothing links
        # igl::predicates any more, and leaving it on still builds Shewchuk.
        LIBIGL_PREDICATES OFF
        # LIBIGL_COPYLEFT_TETGEN ON
)

# include(eigen)
FetchContent_MakeAvailable(libigl)