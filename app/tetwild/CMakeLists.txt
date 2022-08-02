
################################################################################
# FloatTetwild
################################################################################

include(gmp)
include(fenvelope)

add_library(wmtk_tetwild
	TetWild.cpp
	TetWild.h

	common.h
	common.cpp
	Parameters.h

	EdgeSplitting.cpp
	EdgeCollapsing.cpp
	EdgeSwapping.cpp
	Smooth.cpp

	TriangleInsertion.cpp
)
add_library(wmtk::tetwild ALIAS wmtk_tetwild)

target_include_directories(wmtk_tetwild PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/)

target_link_libraries(wmtk_tetwild PUBLIC
	Tracy::TracyClient
	wmtk::toolkit
	wmtk::data
	wmtk::sec_lib
	wmtk::remeshing_lib
	igl::predicates
	gmp::gmp
)

add_executable(tetwild main.cpp)
target_compile_options(tetwild PRIVATE "-rdynamic")
target_link_libraries(tetwild PUBLIC wmtk::tetwild)
wmtk_copy_dll(tetwild)
