
################################################################################

include(wmtk_data)
include(gmp)

################################################################################
# Shared Lib
################################################################################

#add_library(app_shared
##	shared/Logger.cpp
##	shared/Logger.hpp
#	shared/Rational.hpp
#
#
##	shared/external/Exception.h
##	shared/external/MshSaver.cpp
##	shared/external/MshSaver.h
#)
#target_include_directories(app_shared PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/shared ${CMAKE_CURRENT_SOURCE_DIR}/shared/external)
#target_link_libraries(app_shared PUBLIC wmtk::toolkit wmtk::data gmp::gmp)


include(fenvelope)

################################################################################
# FloatTetwild
################################################################################

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

#<<<<<<< HEAD
#target_link_libraries(TetWild PUBLIC FastEnvelope app_shared)
#target_include_directories(TetWild PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tetwild)
## 	target_link_libraries(TetWild PUBLIC FastEnvelope)
#target_compile_features(TetWild PUBLIC cxx_std_17)
#
#
#add_executable(TetWild_bin tetwild/main.cpp)
#target_link_libraries(TetWild_bin PUBLIC TetWild)
#=======
target_include_directories(wmtk_tetwild PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tetwild)
#>>>>>>> origin/main

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
