if(TARGET mesh_generator_lib)
    return()
endif()

message(STATUS "Third-party: creating target 'VolumeMesher'")

include(FetchContent)
FetchContent_Declare(
    volumemesher
    GIT_REPOSITORY https://github.com/JcDai/VolumeRemesher.git
    GIT_TAG 69b9d748bf764a077359167b6ab035cfce314a6c
)
FetchContent_MakeAvailable(volumemesher)