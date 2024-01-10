
set(SRC_FILES
    internal/ProceduralOptions.hpp
    internal/Grid2Options.hpp
    internal/Grid2Options.cpp
    internal/Grid3Options.hpp
    internal/Grid3Options.cpp
    internal/GridOptions.hpp
    internal/GridOptions.cpp
    internal/TriangleFanOptions.hpp
    internal/TriangleFanOptions.cpp
    internal/DiskOptions.hpp
    internal/DiskOptions.cpp
    internal/grid_utils.hpp
    internal/grid_utils.cpp
    procedural.cpp
	procedural.hpp)


include(stb)
#CURRENT_COMPONENT_LIB_NAME is set from the main cmake
target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PRIVATE stb::image)

