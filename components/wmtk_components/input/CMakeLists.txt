
set(SRC_FILES
    internal/InputOptions.hpp
    internal/mesh_with_tag_from_image.cpp
    internal/mesh_with_tag_from_image.hpp
    input.cpp
	input.hpp)
target_sources(wildmeshing_components PRIVATE ${SRC_FILES})
