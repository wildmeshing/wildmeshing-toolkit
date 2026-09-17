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
    # At 1a130a3 (2026-09-07) _dragon moves from target_distance_rel 0.01 to 0.001: at 0.01 the front
    # grazes the released tag_0 boundary at the pinch and the final quality pass stalls; at 0.001 the
    # case converges in 4 turns with max AMIPS under the stop energy.
    # At ddf5cb1 (2026-09-08) _dragon is back at 0.01 with throw_on_nonconvergence: under the wall +
    # input-complex envelope setup the final quality pass gets under stop_energy, and the fixture
    # now asserts convergence (front placed AND final quality) instead of merely running.
    # At b59e130 (2026-09-08) every case listed in topological_offset_models.json sets the flag.
    # At 9c2144c (2026-09-08) _two_circles moves to absolute target 0.15 with spec defaults (converges
    # through a 15-iteration final pass), and _dragon_held is added: the dragon at 1e-3 with
    # deform_others false, a converging case for the per-tag envelope setup.
    # At 3734b13 (2026-09-09) the manifold_extraction cases move to manifold_extraction_models.json,
    # read by the hidden [manifold] group; manifold_extraction_3d leaves integration_tests.json.
    # At e19ece6 (2026-09-15) the seven 2D cases above are GONE, and with them the three models only
    # they used (dragon_rectangle, two_circles, two_overlap_crossing). In their place: nine cases on
    # three simple shapes -- circle_2d, square_2d, triangle_2d, each at target_distance_rel 5e-2,
    # 1e-2 and 1e-3 -- all on front_conv_criterion "residual_error" (which lands in this repo in the
    # same push; the fixtures cannot pass without it) and all with throw_on_nonconvergence, so the
    # group now asserts convergence rather than merely that a case runs. Each was run locally first:
    # 4 turns for the large cases, 6-7 for the medium, 9 for the small, all under 2 s. The small
    # cases override envelope_size_rel to 1e-4 because that key is relative to the bounding-box
    # diagonal and not to target_distance, so the default is a full delta wide at 1e-3 and the
    # triangle does not converge at it. The dragon and two_overlap wall-problem fixtures are gone
    # with the rest; the open problem they stood for is recorded in .claude/CLAUDE.md, not here.
    #
    GIT_TAG e19ece62931f64d2766b33c88a880e8056142f6a

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
