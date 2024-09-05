set(COMPONENT_NAME procedural)
add_component(${COMPONENT_NAME})
if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()

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


include(stb)
target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(wmtk_${COMPONENT_NAME} PRIVATE stb::image)

