
if(TARGET nanospline::nanospline)
    return()
endif()

message(STATUS "Third-party (external): creating target 'nanospline::nanospline'")
option(NANOSPLINE_BUILD_TESTS "Build Tests" OFF)
set(NANOSPLINE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
include(FetchContent)
FetchContent_Declare(
    nanospline
    GIT_REPOSITORY https://github.com/qnzhou/nanospline.git
    GIT_TAG 660e3db75d8faa43f201f7638e9bd198bd5237d5
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(nanospline)

set_target_properties(nanospline_ PROPERTIES FOLDER third_party)