set(SRC_FILES
    relative_face_index.cpp
    simplex_index_from_valid_index.cpp
    relative_face_index.hpp
    simplex_index_from_valid_index.hpp
    valid_index_to_simplex_index.hpp
    )

target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
