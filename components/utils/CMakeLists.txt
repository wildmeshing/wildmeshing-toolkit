
set(SRC_FILES
    wmtk/components/utils/ComponentOptionsBase.hpp
    wmtk/components/utils/json_utils.hpp
    wmtk/components/utils/json_utils.cpp
    wmtk/components/utils/get_attributes.hpp
    wmtk/components/utils/get_attributes.cpp

    wmtk/components/utils/resolve_path.cpp
    wmtk/components/utils/resolve_path.hpp
    wmtk/components/utils/json_macros.hpp

    wmtk/components/utils/PathResolver.cpp
    wmtk/components/utils/PathResolver.hpp
    )

add_library(wmtk_component_utils ${SRC_FILES})
target_link_libraries(wmtk_component_utils PRIVATE wmtk::warnings wmtk::toolkit)
target_include_directories(wmtk_component_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# create alias target
add_library(wmtk::component_utils ALIAS wmtk_component_utils)

# Group source files for IDEs
file(GLOB_RECURSE COMPONENTS_FILES_FOR_SOURCE_GROUP "${CMAKE_CURRENT_SOURCE_DIR}/wmtk/components/utils/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/wmtk/components/utils/*.hpp")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/wmtk/components/utils" PREFIX "src" FILES ${COMPONENTS_FILES_FOR_SOURCE_GROUP})

set_target_properties(wmtk_component_utils PROPERTIES FOLDER wmtk_components)
