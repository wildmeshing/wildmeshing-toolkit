set(SRC_FILES

    simplex_index_from_valid_index.cpp
    simplex_index_from_valid_index.hpp

    valid_index_to_simplex_index.cpp
    valid_index_to_simplex_index.hpp

    local_id_table_offset.hpp
    local_id_table_offset.cpp
    )

target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
