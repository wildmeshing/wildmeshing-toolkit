#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include "wmtk/components/multimesh_from_tag/internal/MultiMeshFromTag.hpp"
#include "wmtk/utils/Logger.hpp"
#include "wmtk_components/extract/wmtk/components/extract/internal/Extract.hpp"

using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("surface", "[.]")
{
    wmtk::components::internal::extract_triangle_soup_from_image(
        "/home/zhouyuan/workplace/wildmeshing-toolkit/buildr/mydata/head7+.off",
        "/home/zhouyuan/workplace/wildmeshing-toolkit/buildr/mydata/head.raw",
        1,
        7);
}

// void gmsh2hdf_tag(std::string volumetric_file, std::string gmsh_file, std::string output_file);
TEST_CASE("gmsh2hdf", "[.]")
{
    wmtk::components::internal::gmsh2hdf_tag(
        "/home/zhouyuan/workplace/wildmeshing-toolkit/buildr/mydata/head.raw",
        "/home/zhouyuan/workplace/wildmeshing-toolkit/buildr/mydata/head7+.off.mesh",
        "/home/zhouyuan/workplace/wildmeshing-toolkit/buildr/mydata/head7+.out");
}