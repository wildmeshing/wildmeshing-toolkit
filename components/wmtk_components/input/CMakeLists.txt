
set(SRC_FILES
    internal/InputOptions.hpp
    internal/mesh_with_tag_from_image.cpp
    internal/mesh_with_tag_from_image.hpp
    input.cpp
	input.hpp)

#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PRIVATE stb::image)

