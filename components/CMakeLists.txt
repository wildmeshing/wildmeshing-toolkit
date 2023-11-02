
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/recipes/components")

add_library(wildmeshing_components)
add_library(wmtk::components ALIAS wildmeshing_components)


include(delaunay_psm)
include(jse)

target_link_libraries(wildmeshing_components PRIVATE wmtk::warnings)
target_link_libraries(wildmeshing_components PUBLIC
    wmtk::toolkit
    geogram::delaunay_psm
    jse::jse
)
# Include headers
target_include_directories(wildmeshing_components PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(wmtk_components)

# Group source files for IDEs
file(GLOB_RECURSE COMPONENTS_FILES_FOR_SOURCE_GROUP "*.cpp" "*.h" "*.hpp")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/wmtk_components" PREFIX "components" FILES ${COMPONENTS_FILES_FOR_SOURCE_GROUP})
