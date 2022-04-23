
################################################################################
# FloatTetwild
################################################################################

include(gmp)
include(fenvelope)

add_library(wmtk_tetwild
	tetwild/TetWild.cpp
	tetwild/TetWild.h

	tetwild/common.h
	tetwild/common.cpp
	tetwild/Parameters.h

	tetwild/EdgeSplitting.cpp
	tetwild/EdgeCollapsing.cpp
	tetwild/EdgeSwapping.cpp
	tetwild/TetWildSmooth.cpp

	tetwild/TriangleInsertion.cpp
	tetwild/Rational.hpp
)
add_library(wmtk::tetwild ALIAS wmtk_tetwild)

target_include_directories(wmtk_tetwild PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tetwild)

target_link_libraries(wmtk_tetwild PUBLIC
	Tracy::TracyClient
	wmtk::toolkit
	wmtk::data
	wmtk::sec_lib
	igl::predicates
	FastEnvelope
	gmp::gmp
)

add_executable(tetwild tetwild/main.cpp)
target_compile_options(tetwild PRIVATE "-rdynamic")
target_link_libraries(tetwild PUBLIC wmtk::tetwild)
wmtk_copy_dll(tetwild)
