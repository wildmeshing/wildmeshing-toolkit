set(SRC_FILES
    AttributesUpdateBase.hpp
    AttributesUpdateBase.cpp

    Operation.hpp
    Operation.cpp

    OptimizationSmoothing.cpp
    OptimizationSmoothing.hpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})

add_subdirectory(edge_mesh)
add_subdirectory(tri_mesh)
# add_subdirectory(tet_mesh)
add_subdirectory(utils)

