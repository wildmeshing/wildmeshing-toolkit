# ###############################################################################
# Adaptive Tessellation
# ###############################################################################

if(NOT WMTK_APP_UNIFORM_REMESH)
    message(FATAL_ERROR "Adaptive tessellation requires remeshing app; try WMTK_APP_UNIFORM_REMESH=ON")
endif()

if(NOT WMTK_APP_SEC)
    message(FATAL_ERROR "Adaptive tessellation requires shorted edge collapse app; try WMTK_APP_SEC=ON")
endif()

include(gmp)
include(fenvelope)
include(lagrange)
lagrange_include_modules(io)
lagrange_include_modules(core)
add_library(wmtk_adaptive_tessellation
	Collapse.cpp
	Swap.cpp
	Split.cpp
	Smooth.cpp
	Smooth.h
	Collapse.h
	AdaptiveTessellation.cpp
	AdaptiveTessellation_attributes.cpp
    AdaptiveTessellation_debug.cpp
    AdaptiveTessellation_serialization.cpp
	AdaptiveTessellation.h
	Parameters.h
	GlobalIntersection.h
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
	lagrange::io
	lagrange::core
	gmp::gmp
	stb::image
)
add_executable(adaptive_tessellation main.cpp)
target_compile_options(adaptive_tessellation PRIVATE "-rdynamic")
target_link_libraries(adaptive_tessellation PUBLIC wmtk::adaptive_tessellation)
wmtk_copy_dll(adaptive_tessellation)

if(WMTK_APP_UNIT_TESTS)
    add_subdirectory(tests)
endif()
