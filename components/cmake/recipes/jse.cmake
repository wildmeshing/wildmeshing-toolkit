
if(TARGET jse::jse)
    return()
endif()

message(STATUS "Third-party (external): creating target 'jse::jse'")

include(CPM)
CPMAddPackage(
    NAME jse
    GITHUB_REPOSITORY geometryprocessing/json-spec-engine
    GIT_TAG 11d028ebf54c3665e1a7c25d8ac622a8cb851223
)
FetchContent_MakeAvailable(jse)

set_target_properties(jse PROPERTIES FOLDER third_party)