################################################################################
# Harmonic Triangulations for TetMesh [Alexa2019]
################################################################################

# Include more application dependencies here
include(cli11)

add_library(wmtk_app_harmonic_tet
	HarmonicTet.hpp
	HarmonicTet.cpp
)
add_library(wmtk::harmonic_tet ALIAS wmtk_app_harmonic_tet)
target_link_libraries(wmtk_app_harmonic_tet PUBLIC
	wmtk::toolkit
	wmtk::data
)
target_include_directories(wmtk_app_harmonic_tet PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(wmtk_harmonic_tet_bin main.cpp)

# Direct dependencies
target_link_libraries(wmtk_harmonic_tet_bin PUBLIC
	wmtk::harmonic_tet
	CLI11::CLI11
)
wmtk_copy_dll(wmtk_harmonic_tet_bin)

# Its tests read models from WMTK_DATA_DIR, which cmake/wmtk_data.cmake downloads only when tests
# are built, so they are built only then: with BUILD_TESTING, as the toolkit's are. CTest, which
# defines it, is included after app/, so here it is undefined on a first configure that will take
# CTest's default, ON.
if(WMTK_APP_UNIT_TESTS AND NOT (DEFINED BUILD_TESTING AND NOT BUILD_TESTING))
    add_subdirectory(tests)
endif()
