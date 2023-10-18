
if(TARGET fintediff::finitediff)
    return()
endif()

message(STATUS "Third-party (external): creating target 'finitediff::finitediff'")

include(CPM)
CPMAddPackage(
    NAME finite-diff
    GITHUB_REPOSITORY zfergus/finite-diff
    GIT_TAG 632a73424cf3eafa8c6315ae0de3efe387936c17
)

set_target_properties(finitediff_finitediff PROPERTIES FOLDER third_party)