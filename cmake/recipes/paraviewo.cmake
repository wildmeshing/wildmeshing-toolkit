
if(TARGET paraviewo::paraviewo)
    return()
endif()

message(STATUS "Third-party (external): creating target 'paraviewo::paraviewo'")
option(PARAVIEWO_BUILD_TESTS "Build Tests" OFF)
set(PARAVIEWO_BUILD_TESTS OFF CACHE BOOL "" FORCE)

# HDF5 off: wmtk writes plain VTU (297fe96ea8). polyfem reads and writes HDF5 through h5pp, which
# paraviewo brings, so with WMTK_WITH_POLYFEM it is on, as polyfem has it.
if(WMTK_WITH_POLYFEM)
    set(WMTK_PARAVIEWO_HDF5 ON)
else()
    set(WMTK_PARAVIEWO_HDF5 OFF)
endif()
include(CPM)
CPMAddPackage(
    NAME paraviewo
    GITHUB_REPOSITORY polyfem/paraviewo
    GIT_TAG 1c01aae7ad37020f751eb598c62ae4ae74a3ada7 # polyfem's pin
    OPTIONS
    "PARAVIEWO_WITH_HDF5 ${WMTK_PARAVIEWO_HDF5}"
)

set_target_properties(paraviewo PROPERTIES FOLDER third_party)
set_target_properties(tinyxml2 PROPERTIES FOLDER third_party)