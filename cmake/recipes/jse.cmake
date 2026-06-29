
if(TARGET jse::jse)
    return()
endif()

message(STATUS "Third-party (external): creating target 'jse::jse'")

include(CPM)
CPMAddPackage(
    NAME jse
    GITHUB_REPOSITORY geometryprocessing/json-spec-engine
    GIT_TAG 81c4dc4092115f37a841285dede3d39559f7fb86
)
FetchContent_MakeAvailable(jse)

set_target_properties(jse PROPERTIES FOLDER third_party)