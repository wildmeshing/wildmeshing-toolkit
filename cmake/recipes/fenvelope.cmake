if(TARGET FastEnvelope)
    return()
endif()

message(STATUS "Third-party: creating target 'FastEnvelope'")

include(FetchContent)
FetchContent_Declare(
    fenvelope
    GIT_REPOSITORY https://github.com/wangbolun300/fast-envelope.git
    GIT_TAG 2a73d21433b0dd24c76bca7fb262db494d4784c3
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(fenvelope)
