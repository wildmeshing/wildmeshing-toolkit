
set(SRC_FILES
    internal/OutputOptions.hpp
    output.cpp
	output.hpp)


add_component("output")

target_sources(wmtk_output PRIVATE ${SRC_FILES})
