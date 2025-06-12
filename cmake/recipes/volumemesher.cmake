if(TARGET mesh_generator_lib)
    return()
endif()

message(STATUS "Third-party: creating target 'VolumeMesher'")

include(FetchContent)
FetchContent_Declare(
    volumemesher
    GIT_REPOSITORY https://github.com/JcDai/VolumeRemesher.git
    GIT_TAG 87ac225118ad8c969beb530f837491e356266e52
)
FetchContent_MakeAvailable(volumemesher)