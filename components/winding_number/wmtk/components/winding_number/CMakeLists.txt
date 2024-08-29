set(SRC_FILES
    internal/winding_number.hpp
    internal/winding_number.cpp
    winding_number.hpp
    winding_number.cpp
)

include(libigl)

target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PRIVATE igl::core)