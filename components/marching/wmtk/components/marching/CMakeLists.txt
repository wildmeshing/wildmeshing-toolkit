
set(SRC_FILES
    internal/Marching.hpp
    internal/Marching.cpp
    MarchingOptions.hpp
	marching.hpp
    marching.cpp
    )


add_component("marching")

target_sources(wmtk_marching PRIVATE ${SRC_FILES})
target_link_libraries(wmtk_marching PRIVATE stb::image)

add_component_test(wmtk::marching tests/marching.cpp)
