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
    # At 99c36d4 (data2 PRs #6 and #7, 2026-08-28) the SIX 2D cases are registered and run --
    # topological_offset_2d, _two_circles, _annots_tag4_in, _vertex_input, _dragon and the new
    # _two_overlap (models/two_overlap_crossing.msh, two overlapping disks, the minimal form of the
    # dragon's defect). The group asserts only that a case runs without throwing; dragon and
    # two_overlap are fixtures for the open wall problem (input boundaries inside the offset
    # distance) and are kept as regression fixtures, not as converging cases. The three 3D cases
    # stay parked. Keys the spec no longer has (pre_optimize_sizing_from_edges as a default,
    # sizing_propagate_min, the old phase_b_conv_rel default) were dropped from the fixtures.
    # At 0514682 (2026-09-01) _annots_tag4_in drops its front_conv_rel 0.001 override -- it predates
    # the rule that offset_envelope_rel may not exceed front_conv_rel, and the run refused to start;
    # with the default accuracy 0.025 the case converges.
    #
    GIT_TAG 0514682dee2e0229f38638e00cefb535c15f8fc9

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
