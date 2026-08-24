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
    # The topological_offset cases live in their own group, topological_offset_models.json, which
    # the hidden ([.]) "topological-offset-models" test case reads -- NOT in integration_tests.json,
    # so they never run in CI. They fail every Release job: two threw at construction on a check
    # since fixed here, and all of them became far more expensive when the offset moved to the
    # alternating A/B optimization -- up to ab_max_rounds phases of a full mesh_improvement each,
    # where it used to run one, enough for topological_offset_3d alone to exceed the suite's
    # 7200 s budget.
    #
    # At this revision ALL EIGHT are commented out: the names sit in a _commented_out key and the
    # group's list is empty, so it runs nothing. 2D and 3D now share one optimization flow, and it
    # does not converge on the ESP-based offset potential, so registering any of them would only
    # assert a known failure. The configs are all still in the repo, and an earlier revision had
    # already brought the 3D three up to the current spec -- offset_gradient_rel stated
    # explicitly, offset_field pinned to "smooth" now that the default means ESP, and the inert
    # max_iterations replaced by ab_max_rounds.
    #
    # The last two names added are FIXTURES FOR AN OPEN PROBLEM rather than cases awaiting a fix
    # elsewhere: topological_offset_2d_two_circles and topological_offset_2d_annots_tag4_in both
    # parse against the current strict spec and are parked because the case FAILS. They are the
    # outward and inward forms of the same thing -- two offset fronts approaching one curve, where
    # the summed potential never falls to the level c, so those vertices chase a level set that
    # does not exist and the stuck-refine calls they generate run the ambient mesh away to 1e50.
    # See "OPEN PROBLEMS" in .claude/CLAUDE.md.
    #
    GIT_TAG 5ec0988783795586ad58de404d152393bdabc96f

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
