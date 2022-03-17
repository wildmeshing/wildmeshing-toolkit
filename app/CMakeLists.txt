
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

add_executable(wmtk_harmonic_tet_bin harmonic_tet/main.cpp)

# Direct dependencies
target_link_libraries(wmtk_harmonic_tet_bin PUBLIC
	wmtk::harmonic_tet
	CLI11::CLI11
)
wmtk_copy_dll(wmtk_harmonic_tet_bin)

################################################################################
# Tests
################################################################################

add_subdirectory(tests)


add_subdirectory(Remeshing)

add_subdirectory(ShortestEdgeCollapse)

add_subdirectory(QSLIM)
