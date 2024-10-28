set(COMPONENT_NAME longest_edge_split)
add_component(${COMPONENT_NAME})
if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

set(SRC_FILES
    longest_edge_split.hpp
    longest_edge_split.cpp
    LongestEdgeSplitOptions.hpp
    )


target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})

# target_link_libraries(wmtk_${COMPONENT_NAME} PRIVATE
#     #
# )