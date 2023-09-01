# Include headers
target_include_directories(wildmeshing_components PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(wmtk_components)

# Group source files for IDEs
file(GLOB_RECURSE COMPONENTS_FILES_FOR_SOURCE_GROUP "cpp" "*/h" "*/hpp")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/wmtk_components" PREFIX "components" FILES ${COMPONENTS_FILES_FOR_SOURCE_GROUP})
