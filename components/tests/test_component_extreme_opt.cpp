#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/components/base/Paths.hpp>
#include <wmtk/components/extreme_opt/extreme_opt.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/ParaviewWriter.hpp>

using namespace wmtk::components::base;

using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("extreme_opt_cube", "[components][extreme_opt][.]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");
    json seamed_json = {
        {"type", "input"},
        {"name", "cube_seamed"},
        {"file", (data_dir / "extreme_opt_data_msh" / "cube_pos.msh").string()},
        {"ignore_z", false}};
    wmtk::components::input(Paths(), seamed_json, cache);
    json cut_json = {
        {"type", "input"},
        {"name", "cube_cut"},
        {"file", (data_dir / "extreme_opt_data_msh" / "cube_tex.msh").string()},
        {"ignore_z", true}};
    wmtk::components::input(Paths(), cut_json, cache);

    json extreme_opt = {
        {"mesh_name", "cube"},
        {"length_rel", 0.1},
        {"iterations", 1},
        {"lock_boundary", false},
        {"smooth_boundary", true},
        {"check_inversion", true},
        {"do_split", true},
        {"do_collapse", true},
        {"collapse_optimize_E_max", false},
        {"do_swap", true},
        {"swap_optimize_E_max", false},
        {"do_smooth", true},
        {"uniform_reference", true},
        {"debug_output", true}};

    wmtk::components::extreme_opt(Paths(), extreme_opt, cache);
}
