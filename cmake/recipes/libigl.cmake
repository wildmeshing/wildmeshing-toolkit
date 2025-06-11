if(TARGET igl::core)
    return()
endif()

message(STATUS "Third-party: creating target 'igl::core'")

option(LIBIGL_WITH_OPENGL            "Use OpenGL"                   ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"                     ON)

include(FetchContent)
CPMAddPackage(
    libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG 3ea7f9480967fcf6bf02ce9b993c0ea6d2fc45f6
    OPTIONS LIBIGL_INSTALL OFF
)
# include(eigen)
FetchContent_MakeAvailable(libigl)