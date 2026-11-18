
if(TARGET jse::jse)
    return()
endif()

message(STATUS "Third-party (external): creating target 'jse::jse'")

include(CPM)
CPMAddPackage(
    NAME jse
    GITHUB_REPOSITORY geometryprocessing/json-spec-engine
    GIT_TAG 1ca70f079d3536f297a6af615932ed04059c8fbc # main with json-spec-engine#9 (validation without copies)
)
FetchContent_MakeAvailable(jse)

set_target_properties(jse PROPERTIES FOLDER third_party)