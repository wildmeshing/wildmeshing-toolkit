
include(wmtk_data)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/components/cmake/recipes")
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/components/cmake")

include(add_component)
include(add_component_test)

# list of all components
add_subdirectory("tetwild/wmtk/components/tetwild")





