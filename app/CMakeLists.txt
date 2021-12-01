################################################################################
cmake_minimum_required(VERSION 3.8)
project(TetWild)
################################################################################

# Detects whether this is a top-level project
#get_directory_property(HAS_PARENT PARENT_DIRECTORY)
#if(HAS_PARENT)
#	set(FLOAT_TETWILD_TOPLEVEL_PROJECT OFF)
#else()
	set(FLOAT_TETWILD_TOPLEVEL_PROJECT ON)
#endif()

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

# FloatTetwild utils
# include(FloatTetwildUtils)

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

# Setup dependencies
# include(FloatTetwildDependencies)

################################################################################
# FloatTetwild library
################################################################################

#find_package(GMPfTetWild)
#IF(NOT ${GMP_FOUND})
#	MESSAGE(FATAL_ERROR "Cannot find GMP")
#ENDIF()

# find_package(MPFR)
# IF(NOT ${MPFR_FOUND})
# 	MESSAGE(FATAL_ERROR "Cannot find MPFR")
# ENDIF()

# add_library() can only be called without any source since CMake 3.11 ...
#set(SOURCES
#		TetWild.cpp TetWild.h
#				common.h
#				Parameters.h
#				Envelope.h
#		)
#
#prepend_current_path(SOURCES)
#wildmeshing_toolkit_copy_headers(${SOURCES})
#wildmeshing_toolkit_set_source_group(${SOURCES})
#target_sources(${PROJECT_NAME} PRIVATE ${SOURCES})

add_library(${PROJECT_NAME} #SOURCES)
		external/Exception.h
		external/MshSaver.cpp
		external/MshSaver.h

		TetWild.cpp
		TetWild.h

		Logger.cpp
		Logger.hpp

		common.h
		Parameters.h
		Envelope.h
		main.cpp
		)

################################################################################
# Required libraries
################################################################################

# if(FLOAT_TETWILD_USE_FLOAT)
# 	target_compile_definitions(${PROJECT_NAME} PUBLIC -DFLOAT_TETWILD_USE_FLOAT)
# endif()

# target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${GMP_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PUBLIC igl::predicates WildMeshingToolkit
		# ${GMP_LIBRARIES}
		# ${MPFR_LIBRARIES}
)
# if(FLOAT_TETWILD_ENABLE_TBB)
# 	target_link_libraries(${PROJECT_NAME} PUBLIC tbb::tbb)
# 	target_compile_definitions(${PROJECT_NAME} PUBLIC FLOAT_TETWILD_USE_TBB)
# endif()

# if(FLOAT_TETWILD_WITH_EXACT_ENVELOPE)
# 	target_link_libraries(${PROJECT_NAME} PUBLIC FastEnvelope)
# 	target_compile_definitions(${PROJECT_NAME} PUBLIC NEW_ENVELOPE)
# endif()

####

if(FLOAT_TETWILD_TOPLEVEL_PROJECT)
	add_executable(${PROJECT_NAME}_bin main.cpp)
	target_compile_features(${PROJECT_NAME}_bin PUBLIC ${CXX14_FEATURES})

	target_link_libraries(${PROJECT_NAME}_bin
			PUBLIC
			${PROJECT_NAME}
			warnings::all
			)

	if(TARGET igl::tetgen)
		target_link_libraries(${PROJECT_NAME} PUBLIC igl::tetgen)
		target_compile_definitions(${PROJECT_NAME} PUBLIC -DLIBIGL_WITH_TETGEN)
	endif()

	if(FLOAT_TETWILD_WITH_SANITIZERS)
		add_sanitizers(${PROJECT_NAME}_bin)
	endif()

	if(NOT (${CMAKE_VERSION} VERSION_LESS "3.6.0"))
		set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME}_bin)
	endif()
endif()