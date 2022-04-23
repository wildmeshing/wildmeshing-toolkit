################################################################################
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