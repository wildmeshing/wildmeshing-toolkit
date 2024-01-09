set(SRC_FILES
    AttributesUpdate.hpp
    AttributesUpdate.cpp

    Operation.hpp
    Operation.cpp

    MeshOperation.hpp
    MeshOperation.cpp

    EdgeCollapse.hpp
    EdgeCollapse.cpp

    EdgeSplit.hpp
    EdgeSplit.cpp

    EdgeOperationData.hpp
    EdgeOperationData.cpp

    OptimizationSmoothing.cpp
    OptimizationSmoothing.hpp

    SeamlessSmoothing.hpp
    SeamlessSmoothing.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})

add_subdirectory(attribute_new)
add_subdirectory(attribute_update)

#
add_subdirectory(edge_mesh)
add_subdirectory(tri_mesh)

# #TODODIscuss
add_subdirectory(tet_mesh)

#
add_subdirectory(composite)

#
add_subdirectory(utils)

#
