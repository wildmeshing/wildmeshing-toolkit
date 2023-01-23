if(TARGET LeanVTK)
    return()
endif()

message(STATUS "Third-party (external): creating target 'LeanVTK'")

include(FetchContent)
FetchContent_Declare(
    lean-vtk
    GIT_REPOSITORY https://github.com/mmorse1217/lean-vtk
    GIT_TAG 3353127af2b6d8b320b539c64bfdd14492593b81
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(lean-vtk)