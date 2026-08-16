# data
# License: MIT

if(TARGET wmtk::data)
    return()
endif()

include(ExternalProject)
include(FetchContent)

set(WMTK_DATA_ROOT "${PROJECT_SOURCE_DIR}/data/" CACHE PATH "Where should the toolkit download and look for test data?")

ExternalProject_Add(
    wmtk_data_download
    PREFIX "${FETCHCONTENT_BASE_DIR}/wmtk-test-data"
    SOURCE_DIR ${WMTK_DATA_ROOT}

    GIT_REPOSITORY https://github.com/wildmeshing/data2.git
    # Adds the 2D dragon-rectangle topological offset case (mesh, config, manifest entry): the
    # one target distance of a 1e-5..1 sweep that converges on both criteria. It sets
    # throw_on_nonconvergence, so a convergence regression fails the run rather than warning.
    #
    # Keep this in sync with the `ref:` of the data2 checkout in .github/workflows/pip.yml --
    # the Python integration suite reads the same manifest and must see the same files.
    GIT_TAG 9b947609c7cf4f40d578c27d0e4d2a98fc840d85

    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    LOG_DOWNLOAD ON
)

# Create a dummy target for convenience
add_library(wmtk_data INTERFACE)
add_library(wmtk::data ALIAS wmtk_data)

add_dependencies(wmtk_data wmtk_data_download)

target_compile_definitions(wmtk_data INTERFACE WMTK_DATA_DIR=\"${WMTK_DATA_ROOT}\")
