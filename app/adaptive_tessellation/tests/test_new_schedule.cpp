#include <AdaptiveTessellation.h>
#include <igl/doublearea.h>
#include <igl/readOBJ.h>
#include <catch2/catch.hpp>
#include <wmtk/utils/ManifoldUtils.hpp>
#include <wmtk/utils/TriQualityUtils.hpp>
#include "Collapse.h"
#include "Smooth.h"
#include "Split.h"
#include "Swap.h"
using namespace wmtk;
using namespace lagrange;
using namespace adaptive_tessellation;

TEST_CASE("double area")
{
    std::filesystem::path input_folder = WMTK_DATA_DIR;
    std::filesystem::path input_mesh_path = input_folder / "hemisphere_splited.obj";
    std::filesystem::path position_path = input_folder / "images/hemisphere_512_position.exr";
    std::filesystem::path normal_path =
        input_folder / "images/hemisphere_512_normal-world-space.exr";
    std::filesystem::path height_path =
        input_folder / "images/riveted_castle_iron_door_512_height.exr";

    AdaptiveTessellation m;
    m.mesh_preprocessing(input_mesh_path, position_path, normal_path, height_path);
    Image image;
    image.load(height_path, WrappingMode::MIRROR_REPEAT, WrappingMode::MIRROR_REPEAT);

    Eigen::MatrixXd dbla3d, dbla2d;
    Eigen::MatrixXd CN, FN;
    Eigen::MatrixXd input_V_, input_VT_;
    Eigen::MatrixXi input_F_, input_FT_;
    igl::readOBJ(input_mesh_path.string(), input_V_, input_VT_, CN, input_F_, input_FT_, FN);

    igl::doublearea(input_V_, input_F_, dbla3d);
    igl::doublearea(input_VT_, input_FT_, dbla2d);

    for (const auto t : m.get_faces()) {
        double area2d = m.get_2d_tri_area(t);
        double area3d = m.get_3d_tri_area(t);
        REQUIRE_THAT(area2d, Catch::Matchers::WithinRel(abs(dbla2d(t.fid(m))) * 0.5, 1e-2));
        REQUIRE_THAT(area3d, Catch::Matchers::WithinRel(abs(dbla3d(t.fid(m))) * 0.5, 1e-2));
    }
}