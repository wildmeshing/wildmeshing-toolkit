#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <map>
#include <nlohmann/json.hpp>
#include <wmtk/utils/Logger.hpp>

// components
#include <components_include.hpp>

using namespace wmtk;
using path = std::filesystem::path;

const std::string tags_integration = "[integration_test]";

const path data_dir = WMTK_DATA_DIR;
const path integration_tests_dir = data_dir / "integration_tests";
const path integration_tests_json_file = integration_tests_dir / "integration_tests.json";

nlohmann::json load_json(const path& json_input_file)
{
    // read JSON input file
    nlohmann::json j;
    try {
        std::ifstream ifs(json_input_file);
        j = nlohmann::json::parse(ifs);
    } catch (const std::exception& e) {
        log_and_throw_error("Could not load or parse JSON input file \n{}", e.what());
    }

    // add path to input file to the json so that it can be used for relative output paths
    j["input_dir"] = json_input_file.parent_path().string();

    return j;
}

void wmtk_wrapper(const nlohmann::json& j)
{
    std::map<std::string, std::function<void(nlohmann::json)>> components_map;
    // include auto-generated map
#include <components_map.hpp>

    // make sure input file contains the application name
    if (!j.contains("application")) {
        log_and_throw_error("JSON input file must contain entry `application`.");
    }

    std::string app_str = j["application"];
    if (components_map.count(app_str) == 0) {
        log_and_throw_error("Application {} unknown", app_str);
    }

    // execute
    components_map[app_str](j);
}

void wmtk_wrapper(const path& json_input_file)
{
    const auto j = load_json(json_input_file);
    wmtk_wrapper(j);
}

TEST_CASE("Integration_Tests", tags_integration)
{
    namespace fs = std::filesystem;

    nlohmann::json integration_tests_json;
    REQUIRE_NOTHROW(integration_tests_json = load_json(integration_tests_json_file));

    std::vector<std::string> input_files;
    REQUIRE_NOTHROW(input_files = integration_tests_json["integration_tests"]);

    for (const auto& input_file : input_files) {
        const path& f = integration_tests_dir / input_file;
        logger().info(">>>>>>>>>> Integration test: {} <<<<<<<<<<", f.filename().string());
        CHECK(fs::exists(f));
        CHECK_NOTHROW(wmtk_wrapper(f));
    }
    logger().info("Tested {} files:", input_files.size());
    for (const auto& input_file : input_files) {
        logger().info("    {}", input_file);
    }
}

/**
 * Models that exhausted max_iterations = 80 at stop_energy 10 before #997 -- 14 Thingi10K
 * meshes for tetwild and 16 triwild20k curve networks. They exercise the optimizer right at
 * its convergence limit, which is what makes them worth keeping and also what makes them
 * expensive: minutes to hours each, serial, and hundreds of thousands of elements.
 *
 * Hidden ([.]) so it is never registered with ctest and cannot run in CI. Run it explicitly:
 *
 *     ./wmtk_integration_tests "[challenging]"
 *
 * Each config sets throw_on_fail, so reaching stop_energy is the assertion.
 */
TEST_CASE("challenging-low-stop-energy-models", tags_integration + "[challenging][.]")
{
    namespace fs = std::filesystem;

    nlohmann::json j;
    REQUIRE_NOTHROW(
        j = load_json(integration_tests_dir / "challenging_low_stop_energy_models.json"));

    std::vector<std::string> input_files;
    REQUIRE_NOTHROW(input_files = j["integration_tests"]);
    REQUIRE(!input_files.empty());

    for (const auto& input_file : input_files) {
        const path& f = integration_tests_dir / input_file;
        logger().info(">>>>>>>>>> Challenging model: {} <<<<<<<<<<", f.filename().string());
        CHECK(fs::exists(f));
        CHECK_NOTHROW(wmtk_wrapper(f));
    }
    logger().info("Tested {} challenging models.", input_files.size());
}

/**
 * The topological_offset cases: nine 2D, three apiece on three simple shapes, and nine 3D on the
 * cube.
 *
 * Hidden ([.]) so it is never registered with ctest and cannot run in CI. Run it explicitly:
 *
 *     ./wmtk_integration_tests "[offset]"
 *
 * What the manifest (data2 integration_tests/topological_offset_models.json) lists: for each of
 * circle_2d / square_2d / triangle_2d, a "large" case at target_distance_rel 5e-2, a "medium" one
 * at 1e-2 and a "small" one at 1e-3, every one otherwise at the defaults -- euclidean field,
 * tag_0 selected -- with front_conv_criterion "residual_error". That criterion measures the
 * field's own residual as a LENGTH (|d - target_distance| for the euclidean field) over
 * front_conv_rel x target_distance, so what these cases assert is the distance error itself
 * rather than the size of a Newton step, which is what makes them worth running on shapes this
 * small. All nine set throw_on_nonconvergence, so a case that fails to place the front fails the
 * test. Locally the large cases converge in 4 turns, the medium ones in 6-7 and the small ones in
 * 9, every one of them under 2 s.
 *
 * The three targets span two decades on purpose: the front's accuracy relative to delta is not
 * constant across them -- the worst vertex reads 0.018-0.041x the bar at 1e-2 and 0.032-0.049x at
 * 5e-2 -- so a regression that only bites at one scale is not hidden by the other two.
 *
 * The small cases also carry envelope_size_rel 1e-4, and that is load-bearing, not tidiness:
 * envelope_size_rel is relative to the bounding-box diagonal and NOT to target_distance, so the
 * default 1e-3 is a tenth of delta at target_distance_rel 1e-2 but a FULL delta at 1e-3 -- an
 * input-complex envelope as wide as the whole offset. At the default the triangle does not
 * converge (40 turns, 14.6x the bar); at 1e-4 it converges in 9 with 228x less distance error.
 *
 * The 3D cases (data2 596be38) are the same idea one dimension up, on models/cube_3d.msh: the
 * 3x3 grid the offset work has been swept on, target_distance_rel 5e-2 / 1e-2 / 1e-3 against
 * front_conv_rel 0.5 / 0.1 / 0.025, named cube_{large,medium,small}_offset_conv{0.5,0.1,0.025}.
 * Each is defaults apart from those two keys plus envelope_size_rel 1e-4 and min_sizing_scalar
 * 1e-3, so they exercise the shipping defaults over that grid -- NOT the sweep's parameters,
 * which also set front_alignment_energy false and both EXPERIMENTAL_ keys true.
 *
 * THREE THINGS TO KNOW BEFORE TRUSTING THEM. They do not set throw_on_nonconvergence, unlike the
 * 2D nine, so today they assert only that the run completes without throwing. None of them has
 * been run -- every 2D case was run before it was added, these were not. And
 * target_distance_rel 1e-3 with front_conv_rel 0.025 is a KNOWN STALL on this model: the
 * placement criterion freezes with the sag criterion solved and refinement exhausted, so that
 * case is expected to burn max_rounds.
 *
 * Time the group before registering any of it near CI. That is what retired the previous 3D
 * cases: topological_offset_3d.json and its two siblings (on double_sphere and 127891) sat in
 * the manifest's _commented_out key, unrun, after becoming expensive enough for one of them to
 * exceed the suite's 7200 s budget on the A/B loop. They and models/127891.msh were deleted in
 * data2 596be38; double_sphere.msh stays, three simwild 3D cases read it.
 *
 * An empty "integration_tests" list is a pass, not a failure.
 */
TEST_CASE("topological-offset-models", tags_integration + "[offset][.]")
{
    namespace fs = std::filesystem;

    nlohmann::json j;
    REQUIRE_NOTHROW(j = load_json(integration_tests_dir / "topological_offset_models.json"));

    std::vector<std::string> input_files;
    REQUIRE_NOTHROW(input_files = j["integration_tests"]);

    for (const auto& input_file : input_files) {
        const path& f = integration_tests_dir / input_file;
        logger().info(">>>>>>>>>> Topological offset: {} <<<<<<<<<<", f.filename().string());
        CHECK(fs::exists(f));
        CHECK_NOTHROW(wmtk_wrapper(f));
    }
    logger().info("Tested {} topological_offset models.", input_files.size());
}

/**
 * The manifold_extraction cases, one 2D and one 3D, listed in manifold_extraction_models.json.
 *
 * Hidden ([.]) so it is never registered with ctest and cannot run in CI. Run it explicitly:
 *
 *     ./wmtk_integration_tests "[manifold]"
 *
 * manifold_extraction_3d was in Integration_Tests until data2 dropped it from
 * integration_tests.json; the group asserts that the cases run.
 */
TEST_CASE("manifold-extraction-models", tags_integration + "[manifold][.]")
{
    namespace fs = std::filesystem;

    nlohmann::json j;
    REQUIRE_NOTHROW(j = load_json(integration_tests_dir / "manifold_extraction_models.json"));

    std::vector<std::string> input_files;
    REQUIRE_NOTHROW(input_files = j["integration_tests"]);
    REQUIRE(!input_files.empty());

    for (const auto& input_file : input_files) {
        const path& f = integration_tests_dir / input_file;
        logger().info(">>>>>>>>>> Manifold extraction: {} <<<<<<<<<<", f.filename().string());
        CHECK(fs::exists(f));
        CHECK_NOTHROW(wmtk_wrapper(f));
    }
    logger().info("Tested {} manifold_extraction models.", input_files.size());
}

TEST_CASE("TetWild", tags_integration + "[.]")
{
    const path f = integration_tests_dir / "tetwild_octocat.json";
    nlohmann::json j;
    REQUIRE_NOTHROW(j = load_json(f));
    REQUIRE_NOTHROW(wmtk_wrapper(j));
}