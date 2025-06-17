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
project(WildMeshingToolkit DESCRIPTION "A mesh optimization toolkit")

# ###############################################################################
option(WMTK_BUILD_DOCS "Build doxygen" OFF)
option (BUILD_SHARED_LIBS "Build Shared Libraries" OFF) # we globally want to disable this option due to use of TBB

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/recipes/")

include(wmtk_cpm_cache)
include(CPM)
include(sanitizers)
include(wmtk_colors)
include(wmtk_warnings)
include(wmtk_copy_dll)
include(tracy)
include(nlohmann_json)
include(hdf5)
include(high_five)
include(geogram)
include(onetbb)
include(mshio)
include(metis)
include(gmp)
include(probabilistic_quadrics)
include(lagrange)
include(nlohmann_json)
include(libigl)
igl_include(predicates)
include(spdlog)
include(paraviewo)

# Sort projects inside the solution
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Generate position independent code by default
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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


target_link_libraries(wildmeshing_toolkit PRIVATE wmtk::warnings)
target_link_libraries(wildmeshing_toolkit PUBLIC
    spdlog::spdlog
    igl::core
    igl::predicates
    gmp::gmp
    geogram
    TBB::tbb
    mshio::mshio
    metis::metis
    lagrange::core
    Tracy::TracyClient
    nlohmann_json::nlohmann_json
    HighFive::HighFive
    hdf5::hdf5
    probabilistic_quadrics::probabilistic_quadrics
    paraviewo::paraviewo
)

if(MSVC)
    target_compile_options(wildmeshing_toolkit PUBLIC /MP)
    target_compile_options(wildmeshing_toolkit PUBLIC $<$<CONFIG:RELWITHDEBINFO>:/GL>)
    target_compile_options(wildmeshing_toolkit PUBLIC $<$<CONFIG:RELWITHDEBINFO>:/LTCG>)
    target_compile_options(wildmeshing_toolkit PUBLIC $<$<CONFIG:RELWITHDEBINFO>:/Ob2>) # inline whenever suitable
    target_compile_options(wildmeshing_toolkit PUBLIC $<$<CONFIG:RELWITHDEBINFO>:/Ot>) # favor faster code over binary size
    target_compile_options(wildmeshing_toolkit PUBLIC $<$<CONFIG:RELEASE>:/GL>)
    target_compile_options(wildmeshing_toolkit PUBLIC $<$<CONFIG:RELEASE>:/LTCG>)
    target_compile_options(wildmeshing_toolkit PUBLIC $<$<CONFIG:RELEASE>:/Ot>)
    target_compile_definitions(wildmeshing_toolkit PUBLIC _SILENCE_ALL_MS_EXT_DEPRECATION_WARNINGS)
endif()

# ###############################################################################
# Subdirectories
# ###############################################################################

# Compile extras only if this is a top-level project
if(WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT)
    # Unit tests
    enable_testing()
    add_subdirectory(tests)

    # wmtk_app
    add_subdirectory(app)
    # components (automatically added to the app)
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
