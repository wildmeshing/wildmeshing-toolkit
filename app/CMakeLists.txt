################################################################################
cmake_minimum_required(VERSION 3.8)
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
# FloatTetwild
################################################################################

add_executable(TetWild_bin
		tetwild/main.cpp

		external/Exception.h
		external/MshSaver.cpp
		external/MshSaver.h

		tetwild/TetWild.cpp
		tetwild/TetWild.h

		Logger.cpp
		Logger.hpp

		tetwild/common.h
		tetwild/Parameters.h
		tetwild/Envelope.h

		tetwild/EdgeSplitting.cpp
		tetwild/EdgeCollapsing.cpp
		)

target_link_libraries(TetWild_bin PUBLIC igl::predicates WildMeshingToolkit warnings::all)
target_include_directories(TetWild_bin PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# 	target_link_libraries(TetWild PUBLIC FastEnvelope)
####

target_compile_features(TetWild_bin PUBLIC ${CXX14_FEATURES})

if(TWTK_APPS_WITH_SANITIZERS)
	add_sanitizers(TetWild_bin)
endif()


if(NOT (${CMAKE_VERSION} VERSION_LESS "3.6.0"))
	set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT TetWild_bin)
endif()






################################################################################
# Quadric Edge Collapse 2D
################################################################################

add_executable(QuadricEdgeCollapse_bin
		QuadricEdgeCollapse/main.cpp

		external/Exception.h
		external/MshSaver.cpp
		external/MshSaver.h

		Logger.cpp
		Logger.hpp
		)

target_link_libraries(QuadricEdgeCollapse_bin PUBLIC igl::predicates WildMeshingToolkit warnings::all)
target_include_directories(QuadricEdgeCollapse_bin PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})


target_compile_features(QuadricEdgeCollapse_bin PUBLIC ${CXX14_FEATURES})

if(TWTK_APPS_WITH_SANITIZERS)
	add_sanitizers(QuadricEdgeCollapse_bin)
endif()


if(NOT (${CMAKE_VERSION} VERSION_LESS "3.6.0"))
	set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT QuadricEdgeCollapse_bin)
endif()