# Detects whether this is a top-level project
get_directory_property(HAS_PARENT PARENT_DIRECTORY)
if(HAS_PARENT)
	set(WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT OFF)
else()
	set(WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT ON)
endif()

if(INPUT_THIRD_PARTY_DIR)
	set(WILDMESHING_TOOLKIT_EXTERNAL ${CMAKE_CURRENT_SOURCE_DIR}/${INPUT_THIRD_PARTY_DIR}/)
else()
	set(WILDMESHING_TOOLKIT_EXTERNAL ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/)
endif()

# Check required CMake version
set(REQUIRED_CMAKE_VERSION "3.15.0")
if(WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT)
    cmake_minimum_required(VERSION ${REQUIRED_CMAKE_VERSION})
else()
    # Don't use cmake_minimum_required here to avoid implicitly overriding parent policies
    if(${CMAKE_VERSION} VERSION_LESS ${REQUIRED_CMAKE_VERSION})
        message(FATAL_ERROR "CMake required version to build the toolkit is ${REQUIRED_CMAKE_VERSION}")
    endif()
endif()

################################################################################
project(WildMeshingToolkit DESCRIPTION "A mesh optimization toolkit")
################################################################################


list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/recipes/")

# Color output
include(UseColors)

# Prepend function
include(PrependCurrentPath)

# FloatTetwild utils
include(WildmeshingToolkitUtils)

# Extra warnings
include(Warnings)

# Sort projects inside the solution
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Generate position independent code by default
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

################################################################################


# Sanitizer options
option(WILDMESHING_TOOLKIT_WITH_SANITIZERS	  "Enable sanitizers in compilation targets"   OFF)
option(WILDMESHING_TOOLKIT_SANITIZE_ADDRESS   "Sanitize Address"       OFF)
option(WILDMESHING_TOOLKIT_SANITIZE_MEMORY    "Sanitize Memory"        OFF)
option(WILDMESHING_TOOLKIT_SANITIZE_THREAD    "Sanitize Thread"        OFF)
option(WILDMESHING_TOOLKIT_SANITIZE_UNDEFINED "Sanitize Undefined"     OFF)

# Options for libigl modules
option(LIBIGL_USE_STATIC_LIBRARY     "Use libigl as static library" OFF)
option(LIBIGL_WITH_COMISO            "Use CoMiso"                   OFF)
option(LIBIGL_WITH_EMBREE            "Use Embree"                   OFF)
option(LIBIGL_WITH_OPENGL            "Use OpenGL"                   OFF)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"                     OFF)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"                    OFF)
option(LIBIGL_WITH_PNG               "Use PNG"                      OFF)
option(LIBIGL_WITH_TETGEN            "Use Tetgen"                   OFF)
option(LIBIGL_WITH_TRIANGLE          "Use Triangle"                 OFF)
option(LIBIGL_WITH_PREDICATES        "Use exact predicates"         ON) #remove me once tetwild is out of this repo
option(LIBIGL_WITH_XML               "Use XML"                      OFF)

# Sanitizers
if(WILDMESHING_TOOLKIT_WITH_SANITIZERS)
	list(APPEND CMAKE_MODULE_PATH ${WILDMESHING_TOOLKIT_EXTERNAL}/sanitizers-cmake/cmake)
endif()

# Setup dependencies
include(WildmeshingToolkitDependencies)


# add_library() can only be called without any source since CMake 3.11 ...
add_library(${PROJECT_NAME})

# Public include directory for FloatTetwild
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_BINARY_DIR}/include)

# Extra warnings
target_link_libraries(${PROJECT_NAME} PRIVATE warnings::all)

# Use C++17
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

if(WILDMESHING_TOOLKIT_WITH_SANITIZERS)
	add_sanitizers(${PROJECT_NAME})
endif()


target_link_libraries(${PROJECT_NAME} PUBLIC spdlog::spdlog igl::core
    geogram::delaunay_psm)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_BINARY_DIR}/include)


################################################################################
# Subdirectories
################################################################################

# Other sources
add_subdirectory(src)

# Compile extras only if this is a top-level project
if(WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT)
	# Unit tests
	enable_testing()
	add_subdirectory(tests)
endif()

add_subdirectory(app)
