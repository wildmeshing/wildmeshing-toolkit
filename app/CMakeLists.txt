
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
# Edge Operations 2D
################################################################################

add_library(wmtk_edge_operations
	#EdgeOperations2d/QEC.cpp
	EdgeOperations2d/EdgeOperations2d.h 
	EdgeOperations2d/main.cpp
	
	EdgeOperations2d/ShortestEdgeCollapse.cpp
	EdgeOperations2d/QEC.cpp
)
add_library(wmtk::edge_operations ALIAS wmtk_edge_operations)

target_link_libraries(wmtk_edge_operations PUBLIC
	wmtk::toolkit
	wmtk::data
	igl::predicates
	FastEnvelope
)
target_include_directories(wmtk_edge_operations PUBLIC EdgeOperations2d)


add_executable(edge_operations EdgeOperations2d/main.cpp)
target_link_libraries(edge_operations PUBLIC wmtk::edge_operations)

################################################################################
# Parallel Edge Collapse 2D
################################################################################

add_library(ParallelEdgeCollapse
	ParallelEdgeCollapse/ParallelEdgeCollapse.h
	ParallelEdgeCollapse/ParallelQEC.cpp
	ParallelEdgeCollapse/ParallelShortestEdgeCollapse.cpp
)
add_library(wmtk::ParallelEdgeCollapse ALIAS ParallelEdgeCollapse)

target_link_libraries(ParallelEdgeCollapse PUBLIC 
	igl::predicates 
	FastEnvelope 
	wmtk::toolkit
	wmtk::data)
# add_library(wmtk::parallel_edge_collapse ALIAS ParallelEdgeCollapse)
target_include_directories(ParallelEdgeCollapse PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})


target_compile_features(ParallelEdgeCollapse PUBLIC cxx_std_17)

add_executable(ParallelEdgeCollapse_bin ParallelEdgeCollapse/main.cpp)
target_link_libraries(ParallelEdgeCollapse_bin PUBLIC ParallelEdgeCollapse)

# Harmonic Triangulations for TetMesh [Alexa2019]
################################################################################

add_library(wmtk_app_harmonic_tet
	harmonic_tet/HarmonicTet.hpp
	harmonic_tet/HarmonicTet.cpp
)
add_library(wmtk::harmonic_tet ALIAS wmtk_app_harmonic_tet)
target_link_libraries(wmtk_app_harmonic_tet PUBLIC
	wmtk::toolkit
	wmtk::data
	igl::predicates
)
target_include_directories(wmtk_app_harmonic_tet PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/harmonic_tet)

################################################################################
# Tests
################################################################################

add_subdirectory(tests)
