################################################################################
cmake_minimum_required(VERSION 3.15.0)
project(Apps)
################################################################################

if(INPUT_THIRD_PARTY_DIR)
	set(TWTK_APPS_EXTERNAL ${CMAKE_CURRENT_SOURCE_DIR}/${INPUT_THIRD_PARTY_DIR}/)
else()
	set(TWTK_APPS_EXTERNAL ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/)
endif()


list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/recipes/")

# Color output
include(UseColors)

# Prepend function
include(PrependCurrentPath)

# Extra warnings
include(Warnings)

# Sort projects inside the solution
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Generate position independent code by default
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

################################################################################

# FloatTetwild options
option(TWTK_APPS_WITH_SANITIZERS    "Use sanitizers"                OFF)

# Options for libigl modules
option(LIBIGL_USE_STATIC_LIBRARY        "Use libigl as static library" OFF)
option(LIBIGL_WITH_COMISO            	"Use CoMiso"                   OFF)
option(LIBIGL_WITH_EMBREE            	"Use Embree"                   OFF)
option(LIBIGL_WITH_OPENGL            	"Use OpenGL"                   OFF)
option(LIBIGL_WITH_OPENGL_GLFW       	"Use GLFW"                     OFF)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI 	"Use ImGui"                    OFF)
option(LIBIGL_WITH_PNG               	"Use PNG"                      OFF)
option(LIBIGL_WITH_TETGEN            	"Use Tetgen"                   OFF)
option(LIBIGL_WITH_TRIANGLE          	"Use Triangle"                 OFF)
option(LIBIGL_WITH_PREDICATES        	"Use exact predicates"         ON)
option(LIBIGL_WITH_XML               	"Use XML"                      OFF)


# Options for envelope module
option(FAST_ENVELOPE_WITH_UNIT_TESTS 				"unit test project" 					OFF)
option(FAST_ENVELOPE_ENABLE_TBB        				"Enable TBB"                			OFF)
option(FAST_ENVELOPE_WITH_GEOGRAM_PREDICATES        "Use whole Geogram for predicates"      OFF)
option(FAST_ENVELOPE_WITH_GEOGRAM_PSM_PREDICATES    "Use Geogram predicates only"           OFF)
option(FAST_ENVELOPE_WITH_GMP						"Use gmp to have rational predicates"   OFF)
option(FAST_ENVELOPE_WITH_TIMER  				 	"use timer in the library for debug" 	OFF)

# Sanitizers
if(TWTK_APPS_WITH_SANITIZERS)
	include(sanitizers)
endif()
include(data)


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
target_link_libraries(app_shared PUBLIC WildMeshingToolkit warnings::all wmtk_data)
target_compile_features(app_shared PUBLIC cxx_std_17)

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

add_subdirectory(EdgeCollapse)

################################################################################
# Tests
################################################################################

enable_testing()
add_subdirectory(tests)
