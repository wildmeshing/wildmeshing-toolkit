# Core library
file(GLOB_RECURSE LIB_SOURCES CONFIGURE_DEPENDS
    "src/*.cpp"
    "src/*.hpp"
)

add_library(wmtk_app_interior_tetopt ${LIB_SOURCES})
add_library(wmtk::interior_tetopt ALIAS wmtk_app_interior_tetopt)
target_link_libraries(wmtk_app_interior_tetopt PUBLIC
	wmtk::toolkit
	wmtk::data
	igl::predicates
)
target_include_directories(wmtk_app_interior_tetopt PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)


add_executable(wmtk_interior_tetopt_bin main.cpp)
target_link_libraries(wmtk_interior_tetopt_bin PUBLIC
	wmtk_app_interior_tetopt
	CLI11::CLI11
)
wmtk_copy_dll(wmtk_interior_tetopt_bin)