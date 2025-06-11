
if(TARGET paraviewo::paraviewo)
    return()
endif()

message(STATUS "Third-party (external): creating target 'paraviewo::paraviewo'")
option(PARAVIEWO_BUILD_TESTS "Build Tests" OFF)
set(PARAVIEWO_BUILD_TESTS OFF CACHE BOOL "" FORCE)

include(CPM)
CPMAddPackage(
    NAME paraviewo
    GITHUB_REPOSITORY polyfem/paraviewo
    GIT_TAG 2e0cbba
)

set_target_properties(paraviewo PROPERTIES FOLDER third_party)
set_target_properties(tinyxml2 PROPERTIES FOLDER third_party)

set_target_properties(hdf5-static PROPERTIES FOLDER third_party)
set_target_properties(hdf5_hl-static PROPERTIES FOLDER third_party)