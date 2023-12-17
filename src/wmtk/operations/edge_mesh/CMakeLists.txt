set(SRC_FILES
    EdgeMeshOperation.hpp
    EdgeMeshOperation.cpp

    EdgeOperationData.hpp
    EdgeOperationData.cpp

    CollapseNewAttributeStrategy.hpp
    CollapseNewAttributeStrategy.cpp

)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
