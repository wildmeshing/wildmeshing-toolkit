set(SRC_FILES
    tuple_vector_to_homogeneous_simplex_vector.hpp
    tuple_vector_to_homogeneous_simplex_vector.cpp
    make_unique.cpp
    make_unique.hpp
    SimplexComparisons.hpp
    SimplexComparisons.cpp
    )
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
