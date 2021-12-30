################################################################################
cmake_minimum_required(VERSION 3.15.0)
project(Apps)
################################################################################

if(INPUT_THIRD_PARTY_DIR)
	set(TWTK_APPS_EXTERNAL ${CMAKE_CURRENT_SOURCE_DIR}/${INPUT_THIRD_PARTY_DIR}/)
else()
	set(TWTK_APPS_EXTERNAL ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/)
endif()


list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

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

# Sanitizer options
option(SANITIZE_ADDRESS   "Sanitize Address"       OFF)
option(SANITIZE_MEMORY    "Sanitize Memory"        OFF)
option(SANITIZE_THREAD    "Sanitize Thread"        OFF)
option(SANITIZE_UNDEFINED "Sanitize Undefined"     OFF)

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

# Sanitizers
if(TWTK_APPS_WITH_SANITIZERS)
	list(APPEND CMAKE_MODULE_PATH ${TWTK_APPS_EXTERNAL}/sanitizers-cmake/cmake)
endif()


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
target_link_libraries(app_shared PUBLIC WildMeshingToolkit warnings::all)
target_compile_features(app_shared PUBLIC cxx_std_17)
if(TWTK_APPS_WITH_SANITIZERS)
	add_sanitizers(app_shared)
endif()

################################################################################
# FloatTetwild
################################################################################

add_library(TetWild
	tetwild/TetWild.cpp
	tetwild/TetWild.h

	tetwild/common.h
	tetwild/Parameters.h
	tetwild/Envelope.h

	tetwild/EdgeSplitting.cpp
	tetwild/EdgeCollapsing.cpp
)

target_link_libraries(TetWild PUBLIC igl::predicates app_shared)
target_include_directories(TetWild PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tetwild)
# 	target_link_libraries(TetWild PUBLIC FastEnvelope)
target_compile_features(TetWild PUBLIC cxx_std_17)

if(TWTK_APPS_WITH_SANITIZERS)
	add_sanitizers(TetWild)
endif()

add_executable(TetWild_bin tetwild/main.cpp)
target_link_libraries(TetWild_bin PUBLIC TetWild)



################################################################################
# Quadric Edge Collapse 2D
################################################################################

add_library(EdgeCollapse
	#EdgeCollapse/QEC.cpp
	EdgeCollapse/EdgeCollapse.h 
	EdgeCollapse/main.cpp
	
	EdgeCollapse/ShortestEdgeCollapse.cpp
	EdgeCollapse/QEC.cpp
)

target_link_libraries(EdgeCollapse PUBLIC igl::predicates app_shared)
target_include_directories(EdgeCollapse PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})


target_compile_features(EdgeCollapse PUBLIC cxx_std_17)

if(TWTK_APPS_WITH_SANITIZERS)
	add_sanitizers(EdgeCollapse)
endif()

add_executable(QuadricEdgeCollapse_bin EdgeCollapse/main.cpp)
target_link_libraries(QuadricEdgeCollapse_bin PUBLIC EdgeCollapse)


################################################################################
# Tests
################################################################################

enable_testing()
add_subdirectory(tests)