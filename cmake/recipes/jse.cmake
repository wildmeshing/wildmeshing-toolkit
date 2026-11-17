
if(TARGET jse::jse)
    return()
endif()

message(STATUS "Third-party (external): creating target 'jse::jse'")

include(CPM)
CPMAddPackage(
    NAME jse
    GITHUB_REPOSITORY geometryprocessing/json-spec-engine
    GIT_TAG 618cc0ab7a48302fc5c7c8ec2c0ecc935d921f32 # geometryprocessing/json-spec-engine#9
)
FetchContent_MakeAvailable(jse)

set_target_properties(jse PROPERTIES FOLDER third_party)