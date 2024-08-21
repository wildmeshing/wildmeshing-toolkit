
set(SRC_FILES
    internal/EdgeInsOptions.hpp
    edge_insertion.hpp
    edge_insertion.cpp
    internal/edge_insertion.hpp
    internal/edge_insertion.cpp)

include(libigl)

#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PRIVATE igl::core)

target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PRIVATE wmtkc_multimesh_from_tag)