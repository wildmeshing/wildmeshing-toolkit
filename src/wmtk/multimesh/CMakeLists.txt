
set(SRC_FILES
    same_simplex_dimension_surjection.hpp
    same_simplex_dimension_surjection.cpp
    same_simplex_dimension_bijection.hpp
    same_simplex_dimension_bijection.cpp
    MultiMeshVisitor.hpp
    )
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})

add_subdirectory(utils)
