if(TARGET LeanVTK)
    return()
endif()

message(STATUS "Third-party (external): creating target 'LeanVTK'")

include(FetchContent)
FetchContent_Declare(
    lean-vtk
    GIT_REPOSITORY https://github.com/mmorse1217/lean-vtk
    GIT_TAG 7e456eddf9e542e5fc06ce0c3be13ad0db795cb5
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(lean-vtk)
