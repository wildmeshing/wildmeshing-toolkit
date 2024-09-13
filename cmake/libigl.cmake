if(TARGET igl::core)
    return()
endif()

include(FetchContent)
FetchContent_Declare(
    libigl
    GIT_REPOSITORY https://github.com/zlyfunction/libigl.git
    GIT_TAG 926c6552addbf605373df59d7dc443702cf8bb28
)
FetchContent_MakeAvailable(libigl)