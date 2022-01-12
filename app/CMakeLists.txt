
################################################################################

include(wmtk_data)
include(fenvelope)

################################################################################
# FloatTetwild
################################################################################

add_library(wmtk_tetwild
	tetwild/TetWild.cpp
	tetwild/TetWild.h

	tetwild/common.h
	tetwild/Parameters.h

	tetwild/EdgeSplitting.cpp
	tetwild/EdgeCollapsing.cpp
	tetwild/EdgeSwapping.cpp
)
add_library(wmtk::tetwild ALIAS wmtk_tetwild)

target_include_directories(wmtk_tetwild PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tetwild)

target_link_libraries(wmtk_tetwild PUBLIC
	wmtk::toolkit
	wmtk::data
	igl::predicates
	FastEnvelope
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


################################################################################
# Tests
################################################################################

add_subdirectory(tests)
