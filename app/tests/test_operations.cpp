#include <TetWild.h>
#include <common.h>
#include <wmtk/TetMesh.h>

#include <igl/read_triangle_mesh.h>
#include <igl/write_triangle_mesh.h>
#include <catch2/catch.hpp>
#include <wmtk/utils/io.hpp>
#include "spdlog/common.h"

using namespace wmtk;
using namespace tetwild;

TEST_CASE("mesh_improvement", "[tetwild_operation][.slow]")
{
    std::string input_path = WMT_DATA_DIR "/37322.stl";

    Eigen::MatrixXd V;
    Eigen::MatrixXd F;
    igl::read_triangle_mesh(input_path, V, F);

    std::vector<Vector3d> vertices(V.rows());
    std::vector<std::array<size_t, 3>> faces(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        vertices[i] = V.row(i);
    }
    std::vector<fastEnvelope::Vector3i> env_faces(F.rows()); // todo: add new api for envelope
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            faces[i][j] = F(i, j);
            env_faces[i][j] = F(i, j);
        }
    }

    int NUM_THREADS = 4;
    tetwild::TetWild::InputSurface input_surface;
    input_surface.params.lr = 1 / 15.0;
    input_surface.init(vertices, faces);
    input_surface.remove_duplicates();
    Eigen::MatrixXd new_F(input_surface.faces.size(), 3);
    for (int i = 0; i < input_surface.faces.size(); i++) {
        new_F(i, 0) = input_surface.faces[i][0];
        new_F(i, 1) = input_surface.faces[i][1];
        new_F(i, 2) = input_surface.faces[i][2];
    }
    auto partitioned_v = partition_mesh_vertices(new_F, NUM_THREADS);
    std::vector<int> partition_id(partitioned_v.rows());
    for (int i = 0; i < partitioned_v.rows(); i++) {
        partition_id[i] = partitioned_v(i, 0);
    }
    input_surface.partition_id = partition_id;
    //
    fastEnvelope::FastEnvelope envelope;
    envelope.init(vertices, env_faces, input_surface.params.eps);
    //
    tetwild::TetWild mesh(input_surface.params, envelope, NUM_THREADS);

    mesh.triangle_insertion(input_surface);
    //    mesh.check_attributes();

    mesh.mesh_improvement(5);
}

TEST_CASE("edge_splitting", "[tetwild_operation]")
{
    Parameters params;
    params.lr = 1 / 10.;
    params.init(Vector3d(0, 0, 0), Vector3d(1, 1, 1));

    fastEnvelope::FastEnvelope envelope;
    TetWild tetwild(params, envelope);

    std::vector<VertexAttributes> vertices(4);
    vertices[0].m_posf = Vector3d(0, 0, 0);
    vertices[1].m_posf = Vector3d(1, 0, 0);
    vertices[2].m_posf = Vector3d(0, 1, 0);
    vertices[3].m_posf = Vector3d(0, 0, 1);
    std::vector<std::array<size_t, 4>> tets = {{{0, 1, 2, 3}}};
    std::vector<TetAttributes> tet_attrs(1);
    for (auto& v : vertices) v.m_is_rounded = true;

    tetwild.init(vertices.size(), tets);
    tetwild.create_mesh_attributes(vertices, tet_attrs);

    REQUIRE_FALSE(tetwild.is_inverted(tetwild.tuple_from_tet(0)));
    tetwild.split_all_edges();
    REQUIRE(tetwild.check_mesh_connectivity_validity());

    REQUIRE(tetwild.m_vertex_attribute.size() == 218);
    tetwild.swap_all_edges();
    REQUIRE(tetwild.check_mesh_connectivity_validity());
    tetwild.swap_all_faces();

    REQUIRE(tetwild.check_mesh_connectivity_validity());
}


TEST_CASE("edge_collapsing", "[tetwild_operation]")
{
    Parameters params;
    params.lr = 1 / 20.;
    params.init(Vector3d(0, 0, 0), Vector3d(1, 1, 1));

    fastEnvelope::FastEnvelope envelope;
    TetWild tetwild(params, envelope);


    std::vector<VertexAttributes> vertices(4);
    vertices[0].m_posf = Vector3d(0, 0, 0);
    vertices[1].m_posf = Vector3d(1, 0, 0);
    vertices[2].m_posf = Vector3d(0, 1, 0);
    vertices[3].m_posf = Vector3d(0, 0, 1);
    std::vector<std::array<size_t, 4>> tets = {{{0, 1, 2, 3}}};
    std::vector<TetAttributes> tet_attrs(1);
    for (auto& v : vertices) {
        v.m_is_rounded = true;
        v.m_pos = to_rational(v.m_posf);
        // v.m_is_on_surface = true;
    }
    tetwild.m_collapse_check_link_condition = true;

    tetwild.init(vertices.size(), tets);
    tetwild.create_mesh_attributes(vertices, tet_attrs);


    tetwild.split_all_edges();
    REQUIRE(tetwild.check_mesh_connectivity_validity());
    CHECK(tetwild.m_vertex_attribute.size() == 1377);

    tetwild.collapse_all_edges();
    REQUIRE(tetwild.check_mesh_connectivity_validity());
    CHECK(tetwild.get_vertices().size() <= 574);
    REQUIRE(tetwild.tet_capacity() == 5608);

    tetwild.consolidate_mesh();
    auto n_tet_after = tetwild.get_tets().size();
    auto n_verts_after = tetwild.get_vertices().size();
    REQUIRE(n_tet_after <= 2170);
    REQUIRE(tetwild.tet_capacity() == n_tet_after);
    REQUIRE(tetwild.check_mesh_connectivity_validity());
    REQUIRE([&tetwild]() -> bool {
        for (auto& t : tetwild.get_tets()) {
            if (tetwild.is_inverted(t)) return false;
        }
        return true;
    }());
}

TEST_CASE("inversion-check-rational-tetwild", "[tetwild_operation]")
{
    Parameters params;
    params.lr = 1 / 10.;
    params.init(Vector3d(0, 0, 0), Vector3d(1, 1, 1));

    fastEnvelope::FastEnvelope envelope;
    TetWild tetwild(params, envelope);

    std::vector<VertexAttributes> vertices(4);
    vertices[0].m_pos = Vector3(0, 0, 0);
    vertices[1].m_pos = Vector3(1, 0, 0);
    vertices[2].m_pos = Vector3(0, 1, 0);
    vertices[3].m_pos = Vector3(0, 0, 1);
    for (auto& v : vertices) v.m_is_rounded = false;
    std::vector<std::array<size_t, 4>> tets = {{{0, 1, 2, 3}}};
    std::vector<TetAttributes> tet_attrs(1);

    tetwild.init(vertices.size(), tets);
    tetwild.create_mesh_attributes(vertices, tet_attrs);
    REQUIRE_FALSE(tetwild.is_inverted(tetwild.tuple_from_tet(0)));
}

TEST_CASE("optimize-bunny-tw", "[tetwild_operation][.slow]")
{
    MshData msh;
    msh.load(WMT_DATA_DIR "bunny_tetwild_80.msh");
    auto vec_attrs = std::vector<VertexAttributes>(msh.get_num_tet_vertices());
    auto tets = std::vector<std::array<size_t, 4>>(msh.get_num_tets());
    msh.extract_tet_vertices(
        [&](size_t i, double x, double y, double z) { vec_attrs[i].m_posf << x, y, z; });
    msh.extract_tets([&](size_t i, size_t v0, size_t v1, size_t v2, size_t v3) {
        tets[i] = {{v0, v1, v2, v3}};
    });

    Parameters params;
    params.lr = 1 / 10.;
    params.init(Vector3d(0, 0, 0), Vector3d(1, 1, 1));

    fastEnvelope::FastEnvelope envelope;
    TetWild tetwild(params, envelope);

    tetwild.init(vec_attrs.size(), tets);
    std::vector<TetAttributes> tet_attrs(tets.size());
    tetwild.create_mesh_attributes(vec_attrs, tet_attrs);

    // tetwild.split_all_edges();
    // logger().info("Split {}"aa, tetwild.cnt_split);
    tetwild.collapse_all_edges();
    logger().info("Col {}", tetwild.cnt_collapse);
    tetwild.swap_all_edges();
    tetwild.swap_all_faces();
    logger().info("Swp {}", tetwild.cnt_swap);

    tetwild.output_mesh("bunny-tw.msh");
}