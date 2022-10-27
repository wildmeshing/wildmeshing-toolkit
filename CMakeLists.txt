# Detects whether this is a top-level project
get_directory_property(HAS_PARENT PARENT_DIRECTORY)

if(HAS_PARENT)
    set(WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT OFF)
else()
    set(WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT ON)
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

# ###############################################################################
project(WildMeshingToolkit DESCRIPTION "A mesh optimization toolkit")

# ###############################################################################
option(WMTK_BUILD_DOCS "Build doxygen" OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/recipes/")

include(sanitizers)
include(wmtk_colors)
include(wmtk_warnings)
include(wmtk_copy_dll)
include(tracy)

# Sort projects inside the solution
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Generate position independent code by default
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ###############################################################################
# WildMeshingToolkit library
# ###############################################################################

# Core library
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS
    "src/*.cpp"
    "src/*.h"
    "src/*.hpp"
)
message(DEBUG "SRC_FILES: ${SRC_FILES}")

add_library(wildmeshing_toolkit "${SRC_FILES}")
add_library(wmtk::toolkit ALIAS wildmeshing_toolkit)

# Include headers
target_include_directories(wildmeshing_toolkit PUBLIC src)

# Compile definitions
target_compile_definitions(wildmeshing_toolkit PUBLIC _USE_MATH_DEFINES)
target_compile_definitions(wildmeshing_toolkit PUBLIC NOMINMAX)

# C++ standard
target_compile_features(wildmeshing_toolkit PUBLIC cxx_std_17)

# Set IDE folders
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" PREFIX "Source Files" FILES ${SRC_FILES})

# Dependencies
if(NOT TARGET igl::core)
    include(libigl)
endif()

igl_include(predicates)

if(NOT TARGET spdlog)
    include(spdlog)
endif()

include(geogram)
include(onetbb)
include(mshio)
include(metis)
include(gmp)

include(lagrange)
lagrange_include_modules(bvh)

if(TARGET nlohmann_json::nlohmann_json)
    return()
endif()

message(STATUS "Third-party (external): creating target 'nlohmann_json::nlohmann_json'")

# nlohmann_json is a big repo for a single header, so we just download the release archive
set(NLOHMANNJSON_VERSION "v3.7.3")

include(FetchContent)
FetchContent_Declare(
    nlohmann_json
    URL "https://github.com/nlohmann/json/releases/download/${NLOHMANNJSON_VERSION}/include.zip"
    URL_HASH SHA256=87b5884741427220d3a33df1363ae0e8b898099fbc59f1c451113f6732891014
)
FetchContent_MakeAvailable(nlohmann_json)

add_library(nlohmann_json INTERFACE)
add_library(nlohmann_json::nlohmann_json ALIAS nlohmann_json)

include(GNUInstallDirs)
target_include_directories(nlohmann_json INTERFACE
    "$<BUILD_INTERFACE:${nlohmann_json_SOURCE_DIR}>/include"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

# Install rules
set(CMAKE_INSTALL_DEFAULT_COMPONENT_NAME nlohmann_json)
install(DIRECTORY ${nlohmann_json_SOURCE_DIR}/include/nlohmann DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS nlohmann_json EXPORT NlohmannJson_Targets)
install(EXPORT NlohmannJson_Targets DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/nlohmann_json NAMESPACE nlohmann_json::)
export(EXPORT NlohmannJson_Targets FILE "${CMAKE_CURRENT_BINARY_DIR}/NlohmannJsonTargets.cmake")

target_link_libraries(wildmeshing_toolkit PRIVATE wmtk::warnings)
target_link_libraries(wildmeshing_toolkit PUBLIC
    spdlog::spdlog
    igl::core
    igl::predicates
    geogram
    TBB::tbb
    mshio::mshio
    metis::metis
    lagrange::core
    Tracy::TracyClient
    lagrange::bvh
    nlohmann_json
)

# ###############################################################################
# Subdirectories
# ###############################################################################

# Compile extras only if this is a top-level project
if(WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT)
    # Unit tests
    enable_testing()
    add_subdirectory(tests)

    # Demo apps
    add_subdirectory(app)
endif()

if(WMTK_BUILD_DOCS)
    # check if Doxygen is installed
    find_package(Doxygen)

    if(DOXYGEN_FOUND)
        # set input and output files
        set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
        set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

        # request to configure the file
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
        message("Doxygen build started")

        # note the option ALL which allows to build the docs together with the application
        add_custom_target(wmtk_doc_doxygen ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM)
    else(DOXYGEN_FOUND)
        message("Doxygen need to be installed to generate the doxygen documentation")
    endif(DOXYGEN_FOUND)
endif()
