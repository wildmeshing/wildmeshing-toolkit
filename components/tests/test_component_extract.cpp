#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include "wmtk_components/extract/wmtk/components/extract/internal/Extract.hpp"

using json = nlohmann::json;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("surface", "[.]")
{
    wmtk::components::internal::extract_triangle_soup_from_image(
        data_dir / "../data/mydata/kinder7.off",
        data_dir / "../data/mydata/kinder.raw",
        1,
        7);
}

// void gmsh2hdf_tag(std::string volumetric_file, std::string gmsh_file, std::string output_file);
TEST_CASE("gmsh2hdf", "[.]")
{
    wmtk::components::internal::gmsh2hdf_tag(
        data_dir / "../data/mydata/torso.raw",
        data_dir / "../data/mydata/torso7.off.mesh",
        data_dir / "../data/mydata/torso7.out");
}