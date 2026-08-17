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
 * The six topological_offset cases: three 2D, three 3D.
 *
 * Hidden ([.]) so it is never registered with ctest and cannot run in CI. Run it explicitly:
 *
 *     ./wmtk_integration_tests "[offset]"
 *
 * They were in Integration_Tests until data2 c414d7f. Two of them threw at construction on a
 * both-surfaces check that tested a flag pair rather than the geometry (fixed in 366c038e85);
 * all six then became far more expensive when the offset moved to the alternating A/B
 * optimization, which runs up to ab_max_rounds phases of a full mesh_improvement where the old
 * loop ran one -- enough for topological_offset_3d alone to exceed the suite's 7200 s budget.
 *
 * THE 3D CASES ARE NOT EXPECTED TO CONVERGE. 3D plateaus around 9.6x the Phi tolerance because
 * the offset band is in a one-for-one split/collapse stalemate and cannot be refined; only the
 * dragon sets throw_on_nonconvergence, so the rest report and continue. What this group asserts
 * is that they RUN -- which is exactly what the construction throw broke, invisibly, because
 * Debug CI does not build the integration tests at all.
 */
TEST_CASE("topological-offset-models", tags_integration + "[offset][.]")
{
    namespace fs = std::filesystem;

    nlohmann::json j;
    REQUIRE_NOTHROW(j = load_json(integration_tests_dir / "topological_offset_models.json"));

    std::vector<std::string> input_files;
    REQUIRE_NOTHROW(input_files = j["integration_tests"]);
    REQUIRE(!input_files.empty());

    for (const auto& input_file : input_files) {
        const path& f = integration_tests_dir / input_file;
        logger().info(">>>>>>>>>> Topological offset: {} <<<<<<<<<<", f.filename().string());
        CHECK(fs::exists(f));
        CHECK_NOTHROW(wmtk_wrapper(f));
    }
    logger().info("Tested {} topological_offset models.", input_files.size());
}

TEST_CASE("TetWild", tags_integration + "[.]")
{
    const path f = integration_tests_dir / "tetwild_octocat.json";
    nlohmann::json j;
    REQUIRE_NOTHROW(j = load_json(f));
    REQUIRE_NOTHROW(wmtk_wrapper(j));
}