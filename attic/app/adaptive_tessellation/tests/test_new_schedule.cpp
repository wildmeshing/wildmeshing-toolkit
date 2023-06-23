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
// Returns f if it exists or dir/f if that exists. If both do not exist exit.
inline std::filesystem::path get_path(
    const std::filesystem::path& dir,
    const std::filesystem::path& f)
{
    if (std::filesystem::exists(f)) {
        return f;
    }
    if (std::filesystem::exists(dir / f)) {
        return dir / f;
    } else {
        wmtk::logger().warn("File {} does not exist. creating the folder {}", f, dir / f);
        std::filesystem::create_directories(dir / f);
        return dir / f;
    }
}


TEST_CASE("inversed boundary")
{
    std::filesystem::path input_folder = WMTK_DATA_DIR;

    std::filesystem::path input_mesh_path = input_folder / "seamPyramid.obj";
    std::filesystem::path position_path = input_folder / "images/seamPyramid_position.exr";
    std::filesystem::path normal_path = input_folder / "images/seamPyramid_normal_smooth.exr";
    std::filesystem::path height_path = input_folder / "images/seamPyramid_height_10.exr";
    AdaptiveTessellation m;

    Image image;
    image.load(height_path, WrappingMode::MIRROR_REPEAT, WrappingMode::MIRROR_REPEAT);

    m.mesh_preprocessing(input_mesh_path, position_path, normal_path, height_path);
    m.set_parameters(
        0.02,
        0.4,
        image,
        WrappingMode::MIRROR_REPEAT,
        SAMPLING_MODE::BICUBIC,
        DISPLACEMENT_MODE::MESH_3D,
        adaptive_tessellation::ENERGY_TYPE::AREA_QUADRATURE,
        adaptive_tessellation::EDGE_LEN_TYPE::AREA_ACCURACY,
        1);

    for (auto& v : m.get_vertices()) {
        // assert the position obtained using t is the same as the position cached
        if (m.vertex_attrs[v.vid(m)].fixed || !m.is_boundary_vertex(v)) {
            continue;
        }
        REQUIRE(
            (m.vertex_attrs[v.vid(m)].pos - m.mesh_parameters.m_boundary.t_to_uv(
                                                m.vertex_attrs[v.vid(m)].curve_id,
                                                m.vertex_attrs[v.vid(m)].t))
                .squaredNorm() < 1e-8);
        for (auto& e : m.get_one_ring_edges_for_vertex(v)) {
            if (m.is_seam_edge(e)) {
                wmtk::TriMesh::Tuple mirror_edge = m.get_oriented_mirror_edge(e);
                REQUIRE(m.vertex_attrs[v.vid(m)].fixed == m.vertex_attrs[mirror_edge.vid(m)].fixed);
                if (m.vertex_attrs[v.vid(m)].fixed) {
                    continue;
                }
                m.vertex_attrs[mirror_edge.vid(m)].curve_id =
                    m.edge_attrs[mirror_edge.eid(m)].curve_id.value();
                auto a = m.vertex_attrs[mirror_edge.vid(m)].pos;
                auto b = m.mesh_parameters.m_boundary.t_to_uv(
                    m.vertex_attrs[mirror_edge.vid(m)].curve_id,
                    m.vertex_attrs[mirror_edge.vid(m)].t);
                auto res = m.mesh_parameters.m_boundary.uv_to_t(a);
                auto c = m.mesh_parameters.m_boundary.t_to_uv(res.first, res.second);
                CAPTURE(
                    v.vid(m),
                    mirror_edge.vid(m),
                    m.vertex_attrs[mirror_edge.vid(m)].fixed,
                    a.transpose(),
                    b.transpose(),
                    c.transpose(),
                    m.vertex_attrs[mirror_edge.vid(m)].curve_id,
                    m.vertex_attrs[mirror_edge.vid(m)].t,
                    res.first,
                    res.second);
                std::ofstream out("pts.xyz");
                out << a[0] << " " << a[1] << " 0\n";
                // out <<  b[0] << " " << b[1] << " 0\n";
                REQUIRE(
                    (m.vertex_attrs[mirror_edge.vid(m)].pos -
                     m.mesh_parameters.m_boundary.t_to_uv(
                         m.vertex_attrs[mirror_edge.vid(m)].curve_id,
                         m.vertex_attrs[mirror_edge.vid(m)].t))
                        .squaredNorm() < 1e-8);
            }
        }
    }

    m.split_all_edges();

    for (auto& v : m.get_vertices()) {
        // assert the position obtained using t is the same as the position cached
        if (m.vertex_attrs[v.vid(m)].fixed || !m.vertex_attrs[v.vid(m)].boundary_vertex) {
            continue;
        }
        for (auto& e : m.get_one_ring_edges_for_vertex(v)) {
            if (m.edge_attrs[e.eid(m)].curve_id.has_value()) {
                m.vertex_attrs[v.vid(m)].curve_id = m.edge_attrs[e.eid(m)].curve_id.value();
            }
        }
        auto a = m.vertex_attrs[v.vid(m)].pos;
        auto b = m.mesh_parameters.m_boundary.t_to_uv(
            m.vertex_attrs[v.vid(m)].curve_id,
            m.vertex_attrs[v.vid(m)].t);
        auto res = m.mesh_parameters.m_boundary.uv_to_t(a);
        auto c = m.mesh_parameters.m_boundary.t_to_uv(res.first, res.second);
        CAPTURE(v.vid(m), m.vertex_attrs[v.vid(m)].pos.transpose());
        CAPTURE(
            v.vid(m),
            m.vertex_attrs[v.vid(m)].fixed,
            a.transpose(),
            b.transpose(),
            c.transpose(),
            m.vertex_attrs[v.vid(m)].curve_id,
            m.vertex_attrs[v.vid(m)].t,
            res.first,
            res.second);
        std::ofstream out("pts.xyz");
        out << a[0] << " " << a[1] << " 0\n";
        // out << b[0] << " " << b[1] << " 0\n";
        // out << c[0] << " " << c[1] << " 0\n";
        REQUIRE(
            (m.vertex_attrs[v.vid(m)].pos - m.mesh_parameters.m_boundary.t_to_uv(
                                                m.vertex_attrs[v.vid(m)].curve_id,
                                                m.vertex_attrs[v.vid(m)].t))
                .squaredNorm() < 1e-8);
        for (auto& e : m.get_one_ring_edges_for_vertex(v)) {
            if (m.is_seam_edge(e)) {
                wmtk::TriMesh::Tuple mirror_edge = m.get_oriented_mirror_edge(e);
                REQUIRE(m.vertex_attrs[v.vid(m)].fixed == m.vertex_attrs[mirror_edge.vid(m)].fixed);
                m.vertex_attrs[mirror_edge.vid(m)].curve_id =
                    m.edge_attrs[mirror_edge.eid(m)].curve_id.value();
                auto a = m.vertex_attrs[mirror_edge.vid(m)].pos;
                auto b = m.mesh_parameters.m_boundary.t_to_uv(
                    m.vertex_attrs[mirror_edge.vid(m)].curve_id,
                    m.vertex_attrs[mirror_edge.vid(m)].t);
                auto res = m.mesh_parameters.m_boundary.uv_to_t(a);
                auto c = m.mesh_parameters.m_boundary.t_to_uv(res.first, res.second);
                CAPTURE(
                    v.vid(m),
                    mirror_edge.vid(m),
                    m.vertex_attrs[mirror_edge.vid(m)].fixed,
                    m.vertex_attrs[mirror_edge.vid(m)].boundary_vertex,
                    a.transpose(),
                    b.transpose(),
                    c.transpose(),
                    m.vertex_attrs[mirror_edge.vid(m)].curve_id,
                    m.vertex_attrs[mirror_edge.vid(m)].t,
                    res.first,
                    res.second);
                std::ofstream out("pts.xyz");
                out << a[0] << " " << a[1] << " 0\n";
                // out <<  b[0] << " " << b[1] << " 0\n";
                REQUIRE(
                    (m.vertex_attrs[mirror_edge.vid(m)].pos -
                     m.mesh_parameters.m_boundary.t_to_uv(
                         m.vertex_attrs[mirror_edge.vid(m)].curve_id,
                         m.vertex_attrs[mirror_edge.vid(m)].t))
                        .squaredNorm() < 1e-8);
            }
        }
    }
}


TEST_CASE("check cumulate energy")
{
    std::filesystem::path input_folder = WMTK_DATA_DIR;

    std::filesystem::path input_mesh_path = input_folder / "seamPyramid.obj";
    std::filesystem::path position_path = input_folder / "images/seamPyramid_position.exr";
    std::filesystem::path normal_path = input_folder / "images/seamPyramid_normal_smooth.exr";
    std::filesystem::path height_path = input_folder / "images/seamPyramid_height_10.exr";
    AdaptiveTessellation m;

    Image image;
    image.load(height_path, WrappingMode::MIRROR_REPEAT, WrappingMode::MIRROR_REPEAT);

    m.mesh_preprocessing(input_mesh_path, position_path, normal_path, height_path);
    m.set_parameters(
        0.02,
        0.4,
        image,
        WrappingMode::MIRROR_REPEAT,
        SAMPLING_MODE::BICUBIC,
        DISPLACEMENT_MODE::MESH_3D,
        adaptive_tessellation::ENERGY_TYPE::AREA_QUADRATURE,
        adaptive_tessellation::EDGE_LEN_TYPE::AREA_ACCURACY,
        1);
    wmtk::logger().info("cumulated error before: {} ", m.cumulated_per_face_error());
    m.split_all_edges();
    wmtk::logger().info("cumulated error after: {} ", m.cumulated_per_face_error());
    auto output_folder = get_path("test_split_smooth", "");
    m.mesh_parameters.m_output_folder = output_folder;
    int cnt = 0;
    auto output_dir = get_path(output_folder, "itr" + std::to_string(cnt));
    auto output_file = output_dir / "split_result.obj";
    m.write_obj_displaced(output_file);

    while (cnt < 5) {
        REQUIRE(m.check_mesh_connectivity_validity());
        m.smooth_all_vertices();
        output_dir = get_path(output_folder, "itr" + std::to_string(cnt));
        output_file = output_dir / "smooth_result.obj";
        wmtk::logger().info(
            "cumulated error after at itr {} : {} ",
            cnt,
            m.cumulated_per_face_error());
        m.write_obj_displaced(output_file);
        cnt++;
    }
}

TEST_CASE("quality remesh")
{
    constexpr bool use_intermediate = false;

    std::filesystem::path input_folder = WMTK_DATA_DIR;
    std::filesystem::path input_mesh_path = input_folder / "seamPyramid.obj";
    std::filesystem::path position_path = input_folder / "images/seamPyramid_position.exr";
    std::filesystem::path normal_path = input_folder / "images/seamPyramid_normal_smooth.exr";
    std::filesystem::path height_path = input_folder / "images/seamPyramid_height_10.exr";
    auto output_folder = get_path("test_quality", "");
    AdaptiveTessellation m;
    if (use_intermediate) {
        m.mesh_preprocessing_from_intermediate(
            input_mesh_path,
            output_folder / "split_result_uv.ply",
            output_folder / "split_result_world.ply",
            position_path,
            normal_path,
            height_path);
    } else {
        m.mesh_preprocessing(input_mesh_path, position_path, normal_path, height_path);
    }
    Image image;
    image.load(height_path, WrappingMode::MIRROR_REPEAT, WrappingMode::MIRROR_REPEAT);
    REQUIRE(m.check_mesh_connectivity_validity());

    m.set_parameters(
        0.001,
        0.1,
        image,
        WrappingMode::MIRROR_REPEAT,
        SAMPLING_MODE::BICUBIC,
        DISPLACEMENT_MODE::MESH_3D,
        adaptive_tessellation::ENERGY_TYPE::EDGE_LENGTH,
        adaptive_tessellation::EDGE_LEN_TYPE::LINEAR3D,
        1);
    std::string cp = std::filesystem::current_path().string();
    m.mesh_parameters.m_output_folder = output_folder;
    if (!use_intermediate) {
        m.split_all_edges();
        m.write_obj_displaced(m.mesh_parameters.m_output_folder / "split_result.obj");
        m.write_ply_intermediate(
            output_folder / "split_result_uv.ply",
            output_folder / "split_result_world.ply");
    }
    REQUIRE(!m.has_self_intersections());
    m.swap_all_edges_quality_pass();
    m.write_obj_displaced(m.mesh_parameters.m_output_folder / "swap_result.obj");
    REQUIRE(!m.has_self_intersections());
    m.collapse_all_edges();
    m.write_obj_displaced(m.mesh_parameters.m_output_folder / "collapse_result.obj");
    REQUIRE(!m.has_self_intersections());
    m.smooth_all_vertices();
    m.write_obj_displaced(m.mesh_parameters.m_output_folder / "smooth_result.obj");
    REQUIRE(!m.has_self_intersections());
}
