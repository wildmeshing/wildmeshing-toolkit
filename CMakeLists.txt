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
option(WMTK_FP_STRICT "Compile with strict, reproducible floating point (no FMA contraction / fast-math) so integration-test outputs are bit-identical across compilers and OSes" OFF)
option (BUILD_SHARED_LIBS "Build Shared Libraries" OFF) # we globally want to disable this option due to use of TBB

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/recipes/")

# Generate position independent code by default
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Strict, reproducible floating point for the golden-hash integration tests. Set
# GLOBALLY here -- before any dependency or target is configured -- so that every
# translation unit (the toolkit, the components, the app, and third-party code
# such as Eigen, libigl, geogram and polysolve) is compiled identically. Two
# effects, both required for bit-identical output across compilers/OSes/CPUs:
#   * FMA contraction (a*b+c fused into a single rounding) is the main source of
#     cross-compiler divergence, so disable it; and never allow fast-math.
#   * EIGEN_DONT_VECTORIZE forces Eigen's scalar code path so SIMD reductions sum
#     in the same order on every architecture (ARM NEON vs x86 SSE/AVX otherwise
#     differ at ~1 ULP, which can cascade into different discrete mesh decisions).
#     It also pins Eigen's alignment/ABI, which is why it must be global and
#     consistent across every target.
if(WMTK_FP_STRICT)
    if(MSVC)
        add_compile_options(/fp:strict)
    else()
        add_compile_options(-ffp-contract=off -fno-fast-math)
    endif()
    add_compile_definitions(EIGEN_DONT_VECTORIZE)
    message(STATUS "WMTK_FP_STRICT: reproducible floating point enabled (no FMA contraction, no fast-math, Eigen scalar path)")
endif()

include(wmtk_cpm_cache)
include(CPM)
include(sanitizers)
include(wmtk_colors)
include(wmtk_warnings)
include(wmtk_copy_dll)
include(nlohmann_json)
include(eigen)
include(delaunay_psm)
# TBB is no longer used by wmtk itself (replaced by the std::thread shim), but a
# transitive dependency (geogram, pulled in via FastEnvelope) links against it on
# Linux, so the library must still be built and available. wmtk does not link
# TBB::tbb.
include(onetbb)
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
    igl::predicates
    gmp::gmp
    geogram::delaunay_psm
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
    add_subdirectory(app) # also contains python bindings

    include(CTest)

    # Unit tests
    if(BUILD_TESTING)
        enable_testing()
        add_subdirectory(tests)
    endif()

    # components (automatically added to the app and integration tests)
    add_subdirectory(components)
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
