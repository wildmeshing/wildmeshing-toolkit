
if(TARGET fintediff::finitediff)
    return()
endif()

message(STATUS "Third-party (external): creating target 'finitediff::finitediff'")

include(FetchContent)
FetchContent_Declare(
    finite-diff
    GIT_REPOSITORY https://github.com/zfergus/finite-diff.git
    GIT_TAG 632a73424cf3eafa8c6315ae0de3efe387936c17
)
FetchContent_MakeAvailable(finite-diff)
