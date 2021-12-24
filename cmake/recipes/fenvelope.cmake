if(TARGET FastEnvelope)
    return()
endif()

message(STATUS "Third-party: creating target 'FastEnvelope'")

include(FetchContent)
FetchContent_Declare(
    fenvelope
    GIT_REPOSITORY https://github.com/wangbolun300/fast-envelope.git
    GIT_TAG 8a3441778655fc6b0140964548508d4e5a32289d
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(fenvelope)
