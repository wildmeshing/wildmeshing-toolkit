
set(SRC_FILES
    internal/InputOptions.hpp
    internal/mesh_with_tag_from_image.cpp
    internal/mesh_with_tag_from_image.hpp
    input.cpp
	input.hpp)

include(stb)


add_component("input")

target_sources(wmtk_input PRIVATE ${SRC_FILES})
target_link_libraries(wmtk_input PRIVATE stb::image)

add_component_test(wmtk::input tests/input.cpp)