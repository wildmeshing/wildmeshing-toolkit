# Include headers
target_include_directories(wildmeshing_toolkit PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(wmtk)

# Group source files for IDEs
file(GLOB_RECURSE WMTK_FILES_FOR_SOURCE_GROUP "*.cpp" "*.h" "*.hpp")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/wmtk" PREFIX "wmtk" FILES ${WMTK_FILES_FOR_SOURCE_GROUP})
