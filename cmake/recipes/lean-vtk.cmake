if(TARGET LeanVTK)
    return()
endif()

message(STATUS "Third-party (external): creating target 'LeanVTK'")
option(LEANVTK_BUILD_TESTS "Build Tests" OFF)
set(LEANVTK_BUILD_TESTS OFF CACHE BOOL "" FORCE)
include(FetchContent)
FetchContent_Declare(
    lean-vtk
    GIT_REPOSITORY https://github.com/mmorse1217/lean-vtk
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(lean-vtk)