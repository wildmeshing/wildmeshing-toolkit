
################################################################################

include(wmtk_data)

################################################################################
# Shared Lib
################################################################################

add_library(app_shared
	shared/Logger.cpp
	shared/Logger.hpp


	shared/external/Exception.h
	shared/external/MshSaver.cpp
	shared/external/MshSaver.h
)
target_include_directories(app_shared PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/shared ${CMAKE_CURRENT_SOURCE_DIR}/shared/external)
target_link_libraries(app_shared PUBLIC wmtk::toolkit wmtk::data)

include(fenvelope)

################################################################################
# FloatTetwild
################################################################################

add_library(TetWild
	tetwild/TetWild.cpp
	tetwild/TetWild.h

	tetwild/common.h
	tetwild/Parameters.h

	tetwild/EdgeSplitting.cpp
	tetwild/EdgeCollapsing.cpp
	tetwild/EdgeSwapping.cpp
)

target_link_libraries(TetWild PUBLIC igl::predicates FastEnvelope app_shared)
target_include_directories(TetWild PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tetwild)
# 	target_link_libraries(TetWild PUBLIC FastEnvelope)
target_compile_features(TetWild PUBLIC cxx_std_17)


add_executable(TetWild_bin tetwild/main.cpp)
target_link_libraries(TetWild_bin PUBLIC TetWild)



################################################################################
# Quadric Edge Collapse 2D
################################################################################

add_library(QuadricEdgeCollapse
	QuadricEdgeCollapse/QEC.cpp
)

target_link_libraries(QuadricEdgeCollapse PUBLIC igl::predicates FastEnvelope app_shared)
target_include_directories(QuadricEdgeCollapse PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})


target_compile_features(QuadricEdgeCollapse PUBLIC cxx_std_17)

add_executable(QuadricEdgeCollapse_bin QuadricEdgeCollapse/main.cpp)
target_link_libraries(QuadricEdgeCollapse_bin PUBLIC QuadricEdgeCollapse)


################################################################################
# Tests
################################################################################

add_subdirectory(tests)
