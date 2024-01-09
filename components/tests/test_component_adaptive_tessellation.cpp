// #include <catch2/catch_test_macros.hpp>
// #include <nlohmann/json.hpp>
// #include <wmtk/components/adaptive_tessellation/adaptive_tessellation.hpp>
// #include <wmtk/io/Cache.hpp>
// #include <wmtk/io/ParaviewWriter.hpp>
// using namespace wmtk;
// using namespace wmtk::tests;
// const std::filesystem::path data_dir = WMTK_DATA_DIR;
// TEST_CASE("at_test")
// {
//     nlohmann::json input = {
//         {"planar", true},
//         {"passes", 30},
//         // {"input", data_dir / "adaptive_tessellation_test" / "after_smooth_uv.msh"},
//         {"input", data_dir / "subdivided_quad.msh"},
//         // {"input", data_dir / "2d" / "rect1.msh"},
//         // {"input", data_dir / "splited_quad.hdf"},
//         {"target_edge_length", 0.01},
//         {"intermediate_output", true},
//         {"filename", "plot_error"}};
//     CHECK_NOTHROW(wmtk::components::adaptive_tessellation(input));
// }
// // TODO add tests for adaptive tessellation