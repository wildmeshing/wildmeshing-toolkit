# Detects whether this is a top-level project
get_directory_property(HAS_PARENT PARENT_DIRECTORY)

if(HAS_PARENT)
    set(WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT OFF)
else()
    set(WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT ON)
endif()

# Check required CMake version
set(REQUIRED_CMAKE_VERSION "3.25.0")
if(WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT)
    cmake_minimum_required(VERSION ${REQUIRED_CMAKE_VERSION})
    if(CMAKE_VERSION VERSION_GREATER_EQUAL "4.0.0")
        message(WARNING "CMake version is ${CMAKE_VERSION}, setting stuff for dependencies.")
        SET(CMAKE_POLICY_VERSION_MINIMUM ${REQUIRED_CMAKE_VERSION})
    endif()
else()
    # Don't use cmake_minimum_required here to avoid implicitly overriding parent policies
    if(${CMAKE_VERSION} VERSION_LESS ${REQUIRED_CMAKE_VERSION})
        message(FATAL_ERROR "CMake required version to build WildMeshingToolkit is ${REQUIRED_CMAKE_VERSION}")
    endif()
endif()

# Python builds (scikit-build-core): object files outside CMakeFiles/. scikit-build-core deletes the
# build directory's top-level CMakeFiles/ whenever it builds in a new isolated environment, which
# cibuildwheel does for every Python version, and with it the objects of the targets this file
# defines (the toolkit library, metis, GKlib): each wheel recompiled them. CMake's SHORT strategy
# (4.2 and later; older versions ignore it) keeps every target's objects in .o/ instead. Not with
# Visual Studio, whose object directories are outside CMakeFiles/ already.
if(DEFINED SKBUILD AND NOT CMAKE_GENERATOR MATCHES "^Visual Studio")
    set(CMAKE_INTERMEDIATE_DIR_STRATEGY SHORT CACHE STRING "Naming of target intermediate directories")
endif()

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
    # A launcher given on the command line wins. The Windows CI runners have Strawberry Perl's
    # C:\Strawberry\c\bin\ccache.exe on PATH, and before this check it replaced the
    # -DCMAKE_CXX_COMPILER_LAUNCHER=sccache the Windows jobs pass: every Windows build went through
    # a ccache whose cache nothing saved, and sccache's stayed empty.
    foreach(lang IN ITEMS C CXX)
        if(NOT DEFINED CMAKE_${lang}_COMPILER_LAUNCHER)
            set(CMAKE_${lang}_COMPILER_LAUNCHER
                ${CMAKE_COMMAND} -E env ${ccacheEnv} ${CCACHE_PROGRAM}
            )
        endif()
    endforeach()
endif()

################################################################################
# CMake Policies
################################################################################

cmake_policy(SET CMP0054 NEW) # Only interpret if() arguments as variables or keywords when unquoted.
cmake_policy(SET CMP0076 NEW) # target_sources() command converts relative paths to absolute.
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24")
  cmake_policy(SET CMP0135 NEW) # Set the timestamps of all extracted contents to the time of the extraction.
endif()

# ###############################################################################
project(WildMeshingToolkit DESCRIPTION "A mesh optimization toolkit")

# ###############################################################################
option(WMTK_BUILD_DOCS "Build doxygen" OFF)
option(WMTK_BUILD_INTEGRATION_TESTS "Build integration tests (might take very long to run)" OFF) # off by default
option(WMTK_BUILD_DOC_EXAMPLES "Build the documentation's C++ examples; with BUILD_TESTING, test them and its Python and JSON examples" OFF)

# Cross-checks every TriMesh split and collapse against a brute-force reference built from
# the whole mesh, so it is O(#F) per operation. Tests only -- off by default, and gated at
# compile time rather than by a runtime flag so a normal build carries none of it.
option(WMTK_DEBUG_BRUTE_FORCE_OPS "Validate each TriMesh operation against a brute-force reference (very slow)" OFF)

# One orientation count per input on every tracked face (2D: edge), rather than one for all its
# inputs (SurfaceTagAttributes::m_input_orientation): filter "tracked" with filter_inputs then works
# where inputs coincide, and tagged meshes carry each input's count (`orientation`). 32 bytes more
# per face record, 43 instead of 11.
option(WMTK_PER_INPUT_ORIENTATION "One orientation count per input on each tracked face (32 bytes more per face record)" OFF)

# PolyFEM built in, for the Python module's in-process simulation (python/src/polyfem_bindings.cpp).
# It also turns on polysolve's sparse solvers and libigl's predicates (recipes/polysolve.cmake,
# recipes/libigl.cmake).
option(WMTK_WITH_POLYFEM "Build PolyFEM in, for in-process simulation from Python" OFF)
option (BUILD_SHARED_LIBS "Build Shared Libraries" OFF) # we globally want to disable this option

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/recipes/")

# Generate position independent code by default
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(wmtk_cpm_cache)
include(CPM)
include(sanitizers)
include(wmtk_colors)
include(wmtk_warnings)
include(wmtk_copy_dll)
include(nlohmann_json)
include(eigen)
include(volumeremesher) # target VolumeRemesher::VolumeRemesher
include(mshio)
include(metis)
include(gmp)
include(libigl)
include(spdlog)
include(paraviewo)
include(fenvelope)
include(nanoflann)
include(polysolve)

include(simple_bvh)
if(WMTK_WITH_POLYFEM)
    include(polyfem)
endif()
# Sort projects inside the solution
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

option(WMTK_PYBIND "Generate python bindings" ON)
if(WMTK_PYBIND)
    include(pybind11)
    include(pybind11_json)
endif()

# ###############################################################################
# WildMeshingToolkit library
# ###############################################################################

add_library(wildmeshing_toolkit STATIC)
add_library(wmtk::toolkit ALIAS wildmeshing_toolkit)

# set_property(TARGET wildmeshing_toolkit PROPERTY COMPILE_WARNING_AS_ERROR  ON)

add_subdirectory(src/wmtk)

# Group source files for IDEs
file(GLOB_RECURSE WMTK_FILES_FOR_SOURCE_GROUP "src/wmtk/*.cpp" "src/wmtk/*.h" "src/wmtk/*.hpp")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src/wmtk" PREFIX "wmtk" FILES ${WMTK_FILES_FOR_SOURCE_GROUP})

# Include headers
target_include_directories(wildmeshing_toolkit PUBLIC src)

# Compile definitions
target_compile_definitions(wildmeshing_toolkit PUBLIC _USE_MATH_DEFINES)
target_compile_definitions(wildmeshing_toolkit PUBLIC NOMINMAX)

if(WMTK_DEBUG_BRUTE_FORCE_OPS)
    message(STATUS "TriMesh operations will be cross-checked against a brute-force reference. This is very slow.")
    # PUBLIC: tests #ifdef on the same macro to decide whether to exercise the validator.
    target_compile_definitions(wildmeshing_toolkit PUBLIC WMTK_DEBUG_BRUTE_FORCE_OPS)
endif()

if(WMTK_PER_INPUT_ORIENTATION)
    # PUBLIC: the face record's layout depends on it, so everything that includes it must agree.
    target_compile_definitions(wildmeshing_toolkit PUBLIC WMTK_PER_INPUT_ORIENTATION)
endif()

# C++ standard
target_compile_features(wildmeshing_toolkit PUBLIC cxx_std_17)

#target_compile_options(wildmeshing_toolkit PUBLIC  -fconcepts)


# std::thread (used by the concurrency shim that replaced TBB) needs the platform
# threading library; TBB used to pull this in transitively.
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

target_link_libraries(wildmeshing_toolkit PRIVATE wmtk::warnings)
target_link_libraries(wildmeshing_toolkit PUBLIC
    spdlog::spdlog
    igl::core
    gmp::gmp
    VolumeRemesher::VolumeRemesher
    mshio::mshio
    metis::metis
    nlohmann_json::nlohmann_json
    nanoflann::nanoflann
    paraviewo::paraviewo
    FastEnvelope::FastEnvelope
    simple_bvh::simple_bvh
    polysolve::polysolve
    Threads::Threads
)

if(MSVC)
    target_compile_options(wildmeshing_toolkit PUBLIC /MP)
    target_compile_definitions(wildmeshing_toolkit PUBLIC _SILENCE_ALL_MS_EXT_DEPRECATION_WARNINGS)
endif()

#################################################################################
# JSON specifications
#################################################################################
set(WMTK_GENERATED_INCLUDE_DIR "${PROJECT_BINARY_DIR}/generated")

# ###############################################################################
# Subdirectories
# ###############################################################################

# Compile extras only if this is a top-level project
if(WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT)
    # wmtk_app
    add_subdirectory(app)

    # Python bindings: the compiled module wildmeshing._core
    if(WMTK_PYBIND)
        add_subdirectory(python/src)
    endif()

    include(CTest)

    # Unit tests
    if(BUILD_TESTING)
        enable_testing()
        add_subdirectory(tests)
    endif()

    # components (automatically added to the app and integration tests)
    add_subdirectory(components)

    # The documentation's examples (docs/examples), built and run as tests
    if(WMTK_BUILD_DOC_EXAMPLES)
        add_subdirectory(docs/examples)
    endif()
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
