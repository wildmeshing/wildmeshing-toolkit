if(TARGET VolumeMesher)
    return()
endif()

message(STATUS "Third-party: creating target 'VolumeMesher'")

include(FetchContent)
FetchContent_Declare(
    volumemesher
    GIT_REPOSITORY https://github.com/JcDai/VolumeRemesher.git
    GIT_TAG cf32f36c99bd06ccc8852a5fa4de7153da88d5d5
)
FetchContent_MakeAvailable(volumemesher)