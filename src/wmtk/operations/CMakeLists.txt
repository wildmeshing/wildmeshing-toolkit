set(SRC_FILES
    Operation.hpp
    Operation.cpp
    OperationFactory.hpp
    OperationFactory.cpp
    OperationQueue.hpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})

add_subdirectory(tri_mesh)
add_subdirectory(tet_mesh)