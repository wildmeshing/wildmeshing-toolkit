if(TARGET FastEnvelope)
    return()
endif()

message(STATUS "Third-party: creating target 'FastEnvelope'")

include(FetchContent)
FetchContent_Declare(
    fenvelope
    GIT_REPOSITORY https://github.com/wangbolun300/fast-envelope.git
    GIT_TAG 5d5d5ac99b14400b2757e043fbc1bd9eacd0cced
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(fenvelope)
