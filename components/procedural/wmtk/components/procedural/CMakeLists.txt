
set(SRC_FILES
    ProceduralOptions.hpp
    Grid2Options.hpp
    Grid2Options.cpp
    Grid3Options.hpp
    Grid3Options.cpp
    GridOptions.hpp
    GridOptions.cpp
    TriangleFanOptions.hpp
    TriangleFanOptions.cpp
    DiskOptions.hpp
    DiskOptions.cpp
    grid_utils.hpp
    grid_utils.cpp
    )

add_component("procedural")

include(stb)
target_sources(wmtk_procedural PRIVATE ${SRC_FILES})
target_link_libraries(wmtk_procedural PRIVATE stb::image)

