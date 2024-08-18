#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/io/Cache.hpp>
#include "wmtk/utils/Logger.hpp"
#include "wmtk_components/extract_soup/wmtk/components/extract_soup/extract_soup.hpp"
#include "wmtk_components/extract_soup/wmtk/components/extract_soup/internal/Extract_Soup.hpp"

using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;
using namespace wmtk::components::base;

TEST_CASE("component_extract_soup", "[.]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    // input
    {
        json component_json = {
            {"input", "/home/zhouyuan/workplace/slicer_extract/wildmeshing-toolkit/data/temp.raw"},
            {"output", "/home/zhouyuan/workplace/slicer_extract/wildmeshing-toolkit/data/temp.out"},
            {"mode", true},
            {"delta_x", 0.1},
            {"volumetric_encoded_file", ""},
            {"volumetric_encoded_bc_file", ""},
            {"level", 1}};


        CHECK_NOTHROW(wmtk::components::extract_soup(Paths(), component_json, cache));
    }
}

TEST_CASE("surface", "[.]")
{
    wmtk::components::internal::extract_triangle_soup_from_image(
        "/home/zhouyuan/workplace/slicer_extract/wildmeshing-toolkit/data/test.off",
        "/home/zhouyuan/workplace/slicer_extract/wildmeshing-toolkit/data/test",
        7);
}

// void gmsh2hdf_tag(std::string volumetric_file, std::string gmsh_file, std::string output_file);
TEST_CASE("gmsh2hdf", "[.]")
{
    wmtk::components::internal::gmsh2hdf_tag(
        "/home/zhouyuan/workplace/wildmeshing-toolkit/buildr/mydata/head.raw",
        "/home/zhouyuan/workplace/wildmeshing-toolkit/buildr/mydata/head7+.off.mesh",
        "/home/zhouyuan/workplace/wildmeshing-toolkit/buildr/mydata/head7+.out",
        0.1);
}