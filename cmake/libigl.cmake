if(TARGET igl::core)
    return()
endif()

include(FetchContent)
FetchContent_Declare(
    libigl
    GIT_REPOSITORY https://github.com/zlyfunction/libigl.git
    GIT_TAG 6bc3b401e774a3efd88a0a0ea7fda2d87c3f0f55
)
FetchContent_MakeAvailable(libigl)