set(SRC_FILES
    tuple_vector_to_homogeneous_simplex_vector.hpp
    tuple_vector_to_homogeneous_simplex_vector.cpp
    make_unique.cpp
    make_unique.hpp
    SimplexComparisons.hpp
    SimplexComparisons.hxx
    unique_homogeneous_simplices.hpp
    unique_homogeneous_simplices.cpp
    )
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
