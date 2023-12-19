set(SRC_FILES
    AttributesUpdateBase.hpp
    AttributesUpdateBase.cpp
    Operation.hpp
    Operation.cpp
    TupleOperation.hpp
    TupleOperation.cpp
    OperationFactory.hpp
    OperationFactory.cpp
    OperationQueue.hpp
    OperationSettings.hpp
    OperationSettingsBase.cpp
    OperationSettingsBase.hpp

    OptSmoothing.cpp
    OptSmoothing.hpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})

add_subdirectory(edge_mesh)
add_subdirectory(tri_mesh)
add_subdirectory(tet_mesh)
add_subdirectory(utils)

