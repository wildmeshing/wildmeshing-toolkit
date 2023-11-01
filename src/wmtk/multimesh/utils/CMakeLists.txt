
set(SRC_FILES
    find_local_switch_sequence.cpp
    local_switch_tuple.cpp
    transport_tuple.cpp
    find_local_switch_sequence.hpp
    local_switch_tuple.hpp
    transport_tuple.hpp
    tuple_map_attribute_io.hpp
    tuple_map_attribute_io.cpp
    )
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})

