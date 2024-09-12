set(SRC_FILES
    relative_face_index.cpp
    relative_face_index.hpp

    simplex_index_from_valid_index.cpp
    simplex_index_from_valid_index.hpp

    valid_index_to_simplex_index.cpp
    valid_index_to_simplex_index.hpp

    subdart_maximal_action_to_face.hpp
    subdart_maximal_action_to_face.cpp

    largest_shared_subdart_size.hpp
    largest_shared_subdart_size.cpp

    local_id_table_offset.hpp
    local_id_table_offset.cpp
    )

target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
