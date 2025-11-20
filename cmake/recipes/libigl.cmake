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
        LIBIGL_PREDICATES ON
)
# include(eigen)
FetchContent_MakeAvailable(libigl)