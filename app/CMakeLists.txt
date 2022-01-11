
################################################################################
include(wmtk_data)

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

	tetwild/TriangleInsertion.cpp
		tetwild/Rational.hpp
)
add_library(wmtk::tetwild ALIAS wmtk_tetwild)

target_include_directories(wmtk_tetwild PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tetwild)

target_link_libraries(wmtk_tetwild PUBLIC
	wmtk::toolkit
	wmtk::data
	igl::predicates
	FastEnvelope
	gmp::gmp
)

add_executable(tetwild tetwild/main.cpp)
target_link_libraries(tetwild PUBLIC wmtk::tetwild)

################################################################################
# Quadric Edge Collapse 2D
################################################################################

add_library(wmtk_edge_collapse
	#EdgeCollapse/QEC.cpp
	EdgeCollapse/EdgeCollapse.h
	EdgeCollapse/main.cpp

	EdgeCollapse/ShortestEdgeCollapse.cpp
	EdgeCollapse/QEC.cpp
)
add_library(wmtk::edge_collapse ALIAS wmtk_edge_collapse)

target_include_directories(wmtk_edge_collapse PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(wmtk_edge_collapse PUBLIC
	wmtk::toolkit
	wmtk::data
	igl::predicates
	FastEnvelope
)

add_executable(edge_collapse EdgeCollapse/main.cpp)
target_link_libraries(edge_collapse PUBLIC wmtk::edge_collapse)

################################################################################
# Tests
################################################################################

add_subdirectory(tests)
