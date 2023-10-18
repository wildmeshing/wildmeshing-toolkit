
if(TARGET nanospline::nanospline)
    return()
endif()

message(STATUS "Third-party (external): creating target 'nanospline::nanospline'")
option(NANOSPLINE_BUILD_TESTS "Build Tests" OFF)
set(NANOSPLINE_BUILD_TESTS OFF CACHE BOOL "" FORCE)

include(CPM)
CPMAddPackage(
    NAME nanospline
    GITHUB_REPOSITORY qnzhou/nanospline
    GIT_TAG 660e3db75d8faa43f201f7638e9bd198bd5237d5
)

set_target_properties(nanospline_ PROPERTIES FOLDER third_party)