
set(SRC_FILES
    src/wmtk/components/utils/ComponentOptionsBase.hpp
    src/wmtk/components/utils/json_utils.hpp
    src/wmtk/components/utils/json_utils.cpp
    src/wmtk/components/utils/get_attributes.hpp
    src/wmtk/components/utils/get_attributes.cpp

    src/wmtk/components/utils/resolve_path.cpp
    src/wmtk/components/utils/resolve_path.hpp

    src/wmtk/components/utils/Paths.hpp
    )

target_sources(wmtk_component_utils PRIVATE ${SRC_FILES})
target_link_libraries(wmtk_component_utils PRIVATE wmtk::warnings wmtk::toolkit)
target_include_directories(wmtk_component_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# create alias target
add_library(wmtk::component_utils ALIAS wmtk_component_utils)
