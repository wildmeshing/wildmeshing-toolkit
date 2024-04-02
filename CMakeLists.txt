# Detects whether this is a top-level project
get_directory_property(HAS_PARENT PARENT_DIRECTORY)

if(HAS_PARENT)
    set(WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT OFF)
else()
    set(WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT ON)
endif()

# Check required CMake version
set(REQUIRED_CMAKE_VERSION "3.24.0")

if(WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT)
    cmake_minimum_required(VERSION ${REQUIRED_CMAKE_VERSION})
else()
    # Don't use cmake_minimum_required here to avoid implicitly overriding parent policies
    if(${CMAKE_VERSION} VERSION_LESS ${REQUIRED_CMAKE_VERSION})
        message(FATAL_ERROR "CMake required version to build the toolkit is ${REQUIRED_CMAKE_VERSION}")
    endif()
endif()

cmake_policy(SET CMP0135 NEW)

# Enable ccache if available
find_program(CCACHE_PROGRAM ccache)

if(CCACHE_PROGRAM)
    option(WMTK_WITH_CCACHE "Enable ccache when building WMTK" ${WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT})
else()
    option(WMTK_WITH_CCACHE "Enable ccache when building WMTK" OFF)
endif()

if(WMTK_WITH_CCACHE AND CCACHE_PROGRAM)
    set(ccacheEnv
        CCACHE_BASEDIR=${CMAKE_BINARY_DIR}
        CCACHE_SLOPPINESS=clang_index_store,include_file_ctime,include_file_mtime,locale,pch_defines,time_macros
    )

    foreach(lang IN ITEMS C CXX)
        set(CMAKE_${lang}_COMPILER_LAUNCHER
            ${CMAKE_COMMAND} -E env ${ccacheEnv} ${CCACHE_PROGRAM}
        )
    endforeach()
endif()

# ###############################################################################
project(WildMeshingToolkit DESCRIPTION "A mesh optimization toolkit" LANGUAGES C CXX)

# ###############################################################################
option(WMTK_BUILD_DOCS "Build doxygen" OFF)
option(BUILD_SHARED_LIBS "Build Shared Libraries" OFF) # we globally want to disable this option due to use of TBB

option(WMTK_CODE_COVERAGE "Enable coverage reporting" OFF)

add_library(wmtk_coverage_config INTERFACE)

if(WMTK_CODE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # Add required flags (GCC & LLVM/Clang)
    target_compile_options(wmtk_coverage_config INTERFACE
        -g # generate debug info
        --coverage # sets all required flags
    )
    target_link_options(wmtk_coverage_config INTERFACE --coverage)
endif()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/recipes/")

include(sanitizers)
include(wmtk_cpm_cache)
include(wmtk_colors)
include(wmtk_warnings)
include(wmtk_copy_dll)

# Sort projects inside the solution
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Generate position independent code by default
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ###############################################################################
# WildMeshingToolkit library
# ###############################################################################

# Dependencies
include(paraviewo)
include(spdlog)
include(gmp)
include(mshio)
include(predicates)
include(polysolve)
include(fast_envelope)
include(bvh)
include(volume_mesher)

include(libigl)

# Core library
add_library(wildmeshing_toolkit)
add_library(wmtk::toolkit ALIAS wildmeshing_toolkit)
target_link_libraries(wildmeshing_toolkit PUBLIC wmtk_coverage_config)
set_property(TARGET wildmeshing_toolkit PROPERTY COMPILE_WARNING_AS_ERROR ON)

add_subdirectory(src)

# Group source files for IDEs
file(GLOB_RECURSE WMTK_FILES_FOR_SOURCE_GROUP "src/wmtk/*.cpp" "src/wmtk/*.h" "src/wmtk/*.hpp")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src/wmtk" PREFIX "wmtk" FILES ${WMTK_FILES_FOR_SOURCE_GROUP})

# Compile definitions
target_compile_definitions(wildmeshing_toolkit PUBLIC _USE_MATH_DEFINES)
target_compile_definitions(wildmeshing_toolkit PUBLIC NOMINMAX)

# To Record Operations
target_compile_definitions(wildmeshing_toolkit PUBLIC WMTK_RECORD_OPERATIONS)

# C++ standard
target_compile_features(wildmeshing_toolkit PUBLIC cxx_std_17)

# target_compile_options(wildmeshing_toolkit PUBLIC  -fconcepts)

# the max dimension an attribute can be - default is to be any size (a dynamic size), but this can be set to a number like 6
set(WMTK_MAX_ATTRIBUTE_DIMENSION Eigen::Dynamic CACHE STRING "Set the largest size a vector can be in an attribute")
target_compile_definitions(wildmeshing_toolkit PUBLIC WMTK_MAX_ATTRIBUTE_DIMENSION=${WMTK_MAX_ATTRIBUTE_DIMENSION})

option(WMTK_USE_MONOTONIC_ATTRIBUTE_CACHE "Use monotonic allocator to store the cache buffer" OFF)

if(WMTK_USE_MONOTONIC_ATTRIBUTE_CACHE)
    target_compile_definitions(wildmeshing_toolkit PRIVATE WMTK_USE_MONOTONIC_ATTRIBUTE_CACHE)
endif()

option(WMTK_ENABLE_GENERIC_CHECKPOINTS "Enable checkpoint selection" OFF)

if(WMTK_ENABLE_GENERIC_CHECKPOINTS)
    target_compile_definitions(wildmeshing_toolkit PRIVATE WMTK_ENABLE_GENERIC_CHECKPOINTS)
endif()

target_link_libraries(wildmeshing_toolkit PRIVATE wmtk::warnings)
target_link_libraries(wildmeshing_toolkit PUBLIC
    spdlog::spdlog
    paraviewo::paraviewo
    gmp::gmp
    mshio::mshio
    predicates::predicates
    nlohmann_json::nlohmann_json
    polysolve::polysolve
    FastEnvelope::FastEnvelope
    simple_bvh::simple_bvh
    VolumeRemesher::VolumeRemesher
    igl::core
)

if(MSVC)
    target_compile_options(wildmeshing_toolkit PUBLIC /MP)
    target_compile_options(wildmeshing_toolkit PUBLIC $<$<CONFIG:RELWITHDEBINFO>:/GL>)
    target_compile_options(wildmeshing_toolkit PUBLIC $<$<CONFIG:RELWITHDEBINFO>:/Ob2>) # inline whenever suitable
    target_compile_options(wildmeshing_toolkit PUBLIC $<$<CONFIG:RELWITHDEBINFO>:/Ot>) # favor faster code over binary size
    target_compile_options(wildmeshing_toolkit PUBLIC $<$<CONFIG:RELEASE>:/GL>)
    target_compile_options(wildmeshing_toolkit PUBLIC $<$<CONFIG:RELEASE>:/Ot>)
    target_compile_definitions(wildmeshing_toolkit PUBLIC _SILENCE_ALL_MS_EXT_DEPRECATION_WARNINGS)
endif()

if(WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT)
    # Unit tests
    enable_testing()
endif()

add_subdirectory(components)

# ###############################################################################
# Subdirectories
# ###############################################################################
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

# Compile extras only if this is a top-level project
if(WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT)
    add_subdirectory(tests)

    add_executable(wmtk_app main.cpp)
    target_compile_features(wmtk_app PUBLIC cxx_std_17)

    # ###############################################################################
    # WMTK JSON APP
    # ###############################################################################
    target_compile_options(wmtk_app PRIVATE "-rdynamic")
    target_link_libraries(wmtk_app PUBLIC
        wmtk::components
        CLI11::CLI11
    )

    if(MSVC)
        target_compile_options(wmtk_app PUBLIC /MP)
        target_compile_options(wmtk_app PUBLIC $<$<CONFIG:RELWITHDEBINFO>:/GL>)
        target_compile_options(wmtk_app PUBLIC $<$<CONFIG:RELWITHDEBINFO>:/Ob2>) # inline whenever suitable
        target_compile_options(wmtk_app PUBLIC $<$<CONFIG:RELWITHDEBINFO>:/Ot>) # favor faster code over binary size
        target_link_options(wmtk_app PRIVATE $<$<CONFIG:RELWITHDEBINFO>:/LTCG>)
        target_link_options(wmtk_app PRIVATE $<$<CONFIG:RELWITHDEBINFO>:/INCREMENTAL:NO>)
        target_compile_options(wmtk_app PUBLIC $<$<CONFIG:RELEASE>:/GL>)
        target_compile_options(wmtk_app PUBLIC $<$<CONFIG:RELEASE>:/Ot>)
        target_link_options(wmtk_app PRIVATE $<$<CONFIG:RELEASE>:/LTCG>)
        target_link_options(wmtk_app PRIVATE $<$<CONFIG:RELEASE>:/INCREMENTAL:NO>)
    endif()

    add_subdirectory(bijective_map)

    # add_executable(bijective_map_app bijective_map_app.cpp track_operations.hpp)

    # # target_compile_options(bijective_map_app PRIVATE "-rdynamic")
    # target_link_libraries(bijective_map_app PUBLIC
    # wmtk::toolkit
    # wmtk::data
    # wmtk::components
    # CLI11::CLI11
    # jse::jse
    # igl::core igl::opengl igl::glfw
    # )
endif()
