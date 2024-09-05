
set(SRC_FILES
    find_local_switch_sequence.cpp
    find_local_switch_sequence.hpp
    find_local_dart_action.cpp
    find_local_dart_action.hpp
    local_switch_tuple.cpp
    local_switch_tuple.hpp
    transport_tuple.cpp
    transport_tuple.hpp

    tuple_map_attribute_io.hpp
    tuple_map_attribute_io.cpp
   
    internal/TupleTag.hpp
    internal/TupleTag.cpp
    create_tag.hpp
    create_tag.cpp
    extract_child_mesh_from_tag.hpp
    extract_child_mesh_from_tag.cpp


    check_map_valid.cpp
    check_map_valid.hpp
    )
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
