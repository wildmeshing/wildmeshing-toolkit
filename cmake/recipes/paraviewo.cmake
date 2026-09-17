
if(TARGET paraviewo::paraviewo)
    return()
endif()

message(STATUS "Third-party (external): creating target 'paraviewo::paraviewo'")
option(PARAVIEWO_BUILD_TESTS "Build Tests" OFF)
set(PARAVIEWO_BUILD_TESTS OFF CACHE BOOL "" FORCE)

if(NOT DEFINED WMTK_PARAVIEWO_WITH_HDF5)
    set(WMTK_PARAVIEWO_WITH_HDF5 OFF)
endif()

include(CPM)
CPMAddPackage(
    NAME paraviewo
    GITHUB_REPOSITORY polyfem/paraviewo
    GIT_TAG 0a56a08e54ce733ee1d43a4ce87365b2c17ac87c
    OPTIONS
    # polyfem includes h5pp unconditionally, so WMTK_WITH_POLYFEM turns this on (top-level CMakeLists).
    "PARAVIEWO_WITH_HDF5 ${WMTK_PARAVIEWO_WITH_HDF5}"
)

set_target_properties(paraviewo PROPERTIES FOLDER third_party)
set_target_properties(tinyxml2 PROPERTIES FOLDER third_party)