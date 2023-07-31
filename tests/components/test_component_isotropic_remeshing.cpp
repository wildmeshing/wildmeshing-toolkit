#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk_components/input/input.hpp>
#include <wmtk_components/isotropic_remeshing/isotropic_remeshing.hpp>
#include <wmtk_components/output/output.hpp>

using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("component_isotropic_remeshing", "[components],[isotropic_remeshing]")
{
    std::map<std::string, std::filesystem::path> files;

    json input_component_json = {
        {"type", "input"},
        {"name", "input_mesh"},
        {"file", data_dir / "bunny.off"}};


    wmtk::components::input(input_component_json, files);


    SECTION("should pass")
    {
        json mesh_isotropic_remeshing_json = {
            {"type", "isotropic_remeshing"},
            {"input", "input_mesh"},
            {"output", "output_mesh"},
            {"length_abs", 0.003},
            {"length_rel", -1},
            {"iterations", 3},
            {"lock_boundary", true}};
        CHECK_NOTHROW(wmtk::components::isotropic_remeshing(mesh_isotropic_remeshing_json, files));

        {
            json component_json = {
                {"type", "output"},
                {"input", "output_mesh"},
                {"file", "bunny_isotropic_remeshing"}};

            CHECK_NOTHROW(wmtk::components::output(component_json, files));
        }
    }
}