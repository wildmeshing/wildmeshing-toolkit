# incremental tetwild

include(gmp)
include(fenvelope)
include(volumemesher)

add_library(wmtk_incremental_tetwild
	IncrementalTetWild.cpp
	IncrementalTetWild.h

	common.h
	common.cpp
	Parameters.h

	EdgeSplitting.cpp
	EdgeCollapsing.cpp
	EdgeSwapping.cpp
	Smooth.cpp

	TriangleInsertion.cpp
	VolumemesherInsertion.cpp

)
add_library(wmtk::incremental_tetwild ALIAS wmtk_incremental_tetwild)

target_include_directories(wmtk_incremental_tetwild PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/)

target_link_libraries(wmtk_incremental_tetwild PUBLIC
	Tracy::TracyClient
	wmtk::toolkit
	wmtk::data
	wmtk::sec_lib
	wmtk::remeshing_lib
	igl::predicates
	gmp::gmp
	VolumeRemesher::VolumeRemesher
)

add_executable(incremental_tetwild_bin main.cpp)
target_compile_options(incremental_tetwild_bin PRIVATE "-rdynamic")
target_link_libraries(incremental_tetwild_bin PUBLIC wmtk::incremental_tetwild
	CLI11::CLI11
)
wmtk_copy_dll(incremental_tetwild_bin)

if(WMTK_APP_UNIT_TESTS)
    add_subdirectory(tests)
endif()


