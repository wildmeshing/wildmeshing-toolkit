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
    image.load(position_path, WrappingMode::MIRROR_REPEAT, WrappingMode::MIRROR_REPEAT);
    m.set_parameters(
        1e-9,
        0.4,
        image,
        WrappingMode::MIRROR_REPEAT,
        SAMPLING_MODE::BICUBIC,
        DISPLACEMENT_MODE::MESH_3D,
        adaptive_tessellation::ENERGY_TYPE::AREA_QUADRATURE,
        adaptive_tessellation::EDGE_LEN_TYPE::AREA_ACCURACY,
        1);
    Eigen::MatrixXd dbla3d, dbla2d;
    Eigen::MatrixXd CN, FN;
    Eigen::MatrixXd input_V_, input_VT_;
    Eigen::MatrixXi input_F_, input_FT_;
    igl::readOBJ(input_mesh_path.string(), input_V_, input_VT_, CN, input_F_, input_FT_, FN);

    igl::doublearea(input_V_, input_F_, dbla3d);
    igl::doublearea(input_VT_, input_FT_, dbla2d);

    for (const auto t : m.get_faces()) {
        auto vids = m.oriented_tri_vids(t);
        const auto& p1_2d = m.vertex_attrs[vids[0]].pos;
        const auto& p2_2d = m.vertex_attrs[vids[1]].pos;
        const auto& p3_2d = m.vertex_attrs[vids[2]].pos;
        REQUIRE_THAT(
            wmtk::triangle_2d_area(p1_2d, p2_2d, p3_2d),
            Catch::Matchers::WithinRel(abs(dbla2d(t.fid(m))) * 0.5, 1e-1));
        auto tri_conn_3d = input_F_.row(t.fid(m));
        const auto& p1 = input_V_.row(tri_conn_3d(0));
        const auto& p2 = input_V_.row(tri_conn_3d(1));
        const auto& p3 = input_V_.row(tri_conn_3d(2));


        REQUIRE_THAT(
            wmtk::triangle_3d_area<double>(p1, p2, p3),
            Catch::Matchers::WithinRel(abs(dbla3d(t.fid(m))) * 0.5, 1e-1));
    }
}