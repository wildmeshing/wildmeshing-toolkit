
add_library(wildmeshing_components)
add_library(wmtk::components ALIAS wildmeshing_components)
target_link_libraries(wildmeshing_components PRIVATE wmtk::warnings)
wmtk_target_link_system_libraries(wildmeshing_components PUBLIC
    wildmeshing_toolkit
    nlohmann_json::nlohmann_json
)
# Include headers
target_include_directories(wildmeshing_components PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(wmtk_components)

# Group source files for IDEs
file(GLOB_RECURSE COMPONENTS_FILES_FOR_SOURCE_GROUP "*.cpp" "*.h" "*.hpp")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/wmtk_components" PREFIX "components" FILES ${COMPONENTS_FILES_FOR_SOURCE_GROUP})
