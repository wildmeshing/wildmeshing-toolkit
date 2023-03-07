# ###############################################################################
# Triwild
# ###############################################################################

include(gmp)
include(fenvelope)

add_library(wmtk_adaptive_tessellation
	Collapse.cpp
	Swap.cpp
	Split.cpp
	Smooth.cpp
	AdaptiveTessellation.cpp
	AdaptiveTessellation.h
	Parameters.h
)
add_library(wmtk::adaptive_tessellation ALIAS wmtk_adaptive_tessellation)

target_include_directories(wmtk_adaptive_tessellation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/)

target_link_libraries(wmtk_adaptive_tessellation PUBLIC
	Tracy::TracyClient
	wmtk::toolkit
	wmtk::data
	wmtk::sec_lib
	wmtk::remeshing_lib
	igl::predicates
	lagrange::bvh
	gmp::gmp
)

add_executable(adaptive_tessellation main.cpp)
target_compile_options(adaptive_tessellation PRIVATE "-rdynamic")
target_link_libraries(adaptive_tessellation PUBLIC wmtk::adaptive_tessellation)
wmtk_copy_dll(adaptive_tessellation)

if(WMTK_APP_UNIT_TESTS)
    add_subdirectory(tests)
endif()
