
if(TARGET jse::jse)
    return()
endif()

message(STATUS "Third-party (external): creating target 'jse::jse'")

include(CPM)
CPMAddPackage(
    NAME jse
    GITHUB_REPOSITORY geometryprocessing/json-spec-engine
    GIT_TAG e18e0e268c87dd2ffda07901d36684dfa76f0381
)
FetchContent_MakeAvailable(jse)

set_target_properties(jse PROPERTIES FOLDER third_party)