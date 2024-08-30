list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/applications")


option(WMTK_ENABLE_APPLICATION_delaunay "Runs delaunay application" ON)
if(WMTK_ENABLE_APPLICATION_delaunay)
add_subdirectory(delaunay)
endif()
