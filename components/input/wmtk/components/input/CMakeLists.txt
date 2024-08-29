
set(SRC_FILES
    internal/InputOptions.hpp
    internal/mesh_with_tag_from_image.cpp
    internal/mesh_with_tag_from_image.hpp
    input.cpp
	input.hpp)


include(stb)
#COMPONENT_TARGET_NAME is set from the main cmake
target_sources(${COMPONENT_TARGET_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(${COMPONENT_TARGET_NAME} PRIVATE stb::image wmtk::component_utils)

add_component_test(tests/input.cpp)