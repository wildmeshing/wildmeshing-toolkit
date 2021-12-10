################################################################################
cmake_minimum_required(VERSION 3.8)
project(Apps)
################################################################################

if(INPUT_THIRD_PARTY_DIR)
	set(FLOAT_TETWILD_EXTERNAL ${CMAKE_CURRENT_SOURCE_DIR}/${INPUT_THIRD_PARTY_DIR}/)
else()
	set(FLOAT_TETWILD_EXTERNAL ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/)
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
option(FLOAT_TETWILD_ENABLE_TBB         "Enable TBB"                    ON)
option(FLOAT_TETWILD_USE_FLOAT          "Use floats instead of double"  OFF)
option(FLOAT_TETWILD_WITH_SANITIZERS    "Use sanitizers"                OFF)
option(FLOAT_TETWILD_WITH_EXACT_ENVELOPE    "Use exact envelope"        OFF)

# Sanitizer options
option(SANITIZE_ADDRESS   "Sanitize Address"       OFF)
option(SANITIZE_MEMORY    "Sanitize Memory"        OFF)
option(SANITIZE_THREAD    "Sanitize Thread"        OFF)
option(SANITIZE_UNDEFINED "Sanitize Undefined"     OFF)

# Options for libigl modules
option(LIBIGL_USE_STATIC_LIBRARY          "Use libigl as static library" OFF)
option(LIBIGL_WITH_COMISO            "Use CoMiso"                   OFF)
option(LIBIGL_WITH_EMBREE            "Use Embree"                   OFF)
option(LIBIGL_WITH_OPENGL            "Use OpenGL"                   OFF)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"                     OFF)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"                    OFF)
option(LIBIGL_WITH_PNG               "Use PNG"                      OFF)
option(LIBIGL_WITH_TETGEN            "Use Tetgen"                   OFF)
option(LIBIGL_WITH_TRIANGLE          "Use Triangle"                 OFF)
option(LIBIGL_WITH_PREDICATES        "Use exact predicates"         ON)
option(LIBIGL_WITH_XML               "Use XML"                      OFF)

# Sanitizers
if(FLOAT_TETWILD_WITH_SANITIZERS)
	list(APPEND CMAKE_MODULE_PATH ${FLOAT_TETWILD_EXTERNAL}/sanitizers-cmake/cmake)
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
		tetwild/main.cpp

		tetwild/EdgeSplitting.cpp
		tetwild/EdgeCollapsing.cpp
		)

################################################################################
# Required libraries
################################################################################

target_link_libraries(TetWild_bin PUBLIC igl::predicates WildMeshingToolkit warnings::all)
target_include_directories(TetWild_bin PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# if(FLOAT_TETWILD_WITH_EXACT_ENVELOPE)
# 	target_link_libraries(TetWild PUBLIC FastEnvelope)
# 	target_compile_definitions(TetWild PUBLIC NEW_ENVELOPE)
# endif()

####


target_compile_features(TetWild_bin PUBLIC ${CXX14_FEATURES})

if(FLOAT_TETWILD_WITH_SANITIZERS)
	add_sanitizers(TetWild_bin)
endif()



if(NOT (${CMAKE_VERSION} VERSION_LESS "3.6.0"))
	set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT TetWild_bin)
endif()