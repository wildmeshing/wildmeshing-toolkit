#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <remeshing/UniformRemeshing.h>
#include <catch2/catch_test_macros.hpp>
#include <wmtk/utils/ManifoldUtils.hpp>


using namespace wmtk;
using namespace app::remeshing;

TEST_CASE("uniform_remeshing", "[test_remeshing][.]")
{
    const std::string root(WMTK_DATA_DIR);
    const std::string path = root + "/circle.obj";
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);

    REQUIRE(ok);

    std::vector<Eigen::Vector3d> v(V.rows());
    std::vector<std::array<size_t, 3>> tri(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        v[i] = V.row(i);
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
    }
    UniformRemeshing m(v);
    std::vector<std::pair<size_t, int>> modified_v;
    m.create_mesh(V.rows(), tri, modified_v, 0);
    REQUIRE(m.check_mesh_connectivity_validity());
    REQUIRE(m.uniform_remeshing(0.01, 5));
}

TEST_CASE("split_each_edge", "[test_remeshing]")
{
    std::vector<Eigen::Vector3d> v_positions(3);
    v_positions[0] = Eigen::Vector3d(2, 3.5, 0);
    v_positions[1] = Eigen::Vector3d(4, 0, 0);

    v_positions[2] = Eigen::Vector3d(0, 0, 0);

    UniformRemeshing m(v_positions, 0);
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}};
    std::vector<std::pair<size_t, int>> modified_v;
    m.create_mesh(3, tris, modified_v, 0);
    int target_vertnum = m.vert_capacity() + 3 * m.get_edges().size() + 3 * m.tri_capacity();
    m.split_remeshing();
    m.consolidate_mesh();
    REQUIRE(target_vertnum == 15);

    REQUIRE(m.vert_capacity() == target_vertnum);
}

TEST_CASE("test_swap", "[test_remeshing]")
{
    const std::string root(WMTK_DATA_DIR);
    const std::string path = root + "/circle.obj";

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);

    REQUIRE(ok);

    std::vector<Eigen::Vector3d> v(V.rows());
    std::vector<std::array<size_t, 3>> tri(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        v[i] = V.row(i);
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
    }
    UniformRemeshing m(v);
    std::vector<std::pair<size_t, int>> modified_v;
    m.create_mesh(V.rows(), tri, modified_v, 0);
    int v_invariant = m.get_vertices().size();
    int e_invariant = m.get_edges().size();
    REQUIRE(m.check_mesh_connectivity_validity());
    auto edges = m.get_edges();
    std::vector<TriMesh::Tuple> new_e;
    int cnt = 0;
    for (auto edge : edges) {
        if (cnt > 200) break;
        if (!edge.is_valid(m)) continue;
        if (!(edge.switch_face(m)).has_value()) {
            REQUIRE_FALSE(m.swap_edge(edge, new_e));
            continue;
        }
        REQUIRE(m.swap_edge(edge, new_e));
        cnt++;
    }
    REQUIRE(m.check_mesh_connectivity_validity());
    REQUIRE(m.get_vertices().size() == v_invariant);
    REQUIRE(m.get_edges().size() == e_invariant);
}

TEST_CASE("test_split", "[test_remeshing]")
{
    const std::string root(WMTK_DATA_DIR);
    const std::string path = root + "/fan.obj";
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);

    REQUIRE(ok);

    std::vector<Eigen::Vector3d> v(V.rows());
    std::vector<std::array<size_t, 3>> tri(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        v[i] = V.row(i);
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
    }
    UniformRemeshing m(v);
    std::vector<std::pair<size_t, int>> modified_v;
    m.create_mesh(V.rows(), tri, modified_v, 0);
    m.split_remeshing();
    REQUIRE(m.check_mesh_connectivity_validity());
}

// TEST_CASE("remeshing_hanging", "[test_remeshing]")
// {
//     const std::string root(WMTK_DATA_DIR);
//     const std::string path = root + "/100071_sf.obj";
//     std::string output = "100071_out.obj";
//     double env_rel = 1e-3;
//     double len_rel = 5;
//     int thread = 0;

//     wmtk::logger().info("remeshing on {}", path);
//     Eigen::MatrixXd V;
//     Eigen::MatrixXi F;
//     bool ok = igl::read_triangle_mesh(path, V, F);
//     Eigen::VectorXi SVI, SVJ;
//     Eigen::MatrixXd temp_V = V; // for STL file
//     igl::remove_duplicate_vertices(temp_V, 0, V, SVI, SVJ);
//     for (int i = 0; i < F.rows(); i++)
//         for (int j : {0, 1, 2}) F(i, j) = SVJ[F(i, j)];
//     wmtk::logger().info("Before_vertices#: {} \n Before_tris#: {}", V.rows(), F.rows());


//     std::vector<Eigen::Vector3d> v(V.rows());
//     std::vector<std::array<size_t, 3>> tri(F.rows());
//     for (int i = 0; i < V.rows(); i++) {
//         v[i] = V.row(i);
//     }
//     for (int i = 0; i < F.rows(); i++) {
//         for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
//     }

//     const Eigen::MatrixXd box_min = V.colwise().minCoeff();
//     const Eigen::MatrixXd box_max = V.colwise().maxCoeff();
//     const double diag = (box_max - box_min).norm();
//     const double envelope_size = env_rel * diag;
//     Eigen::VectorXi dummy;
//     std::vector<std::pair<size_t, int>> modified_v;
//     if (!igl::is_edge_manifold(F) || !igl::is_vertex_manifold(F, dummy)) {
//         auto v1 = v;
//         auto tri1 = tri;
//         wmtk::separate_to_manifold(v1, tri1, v, tri, modified_v);
//     }

//     UniformRemeshing m(v, thread);
//     m.create_mesh(v.size(), tri, modified_v, envelope_size);
//     REQUIRE(m.check_edge_manifold());
//     m.get_vertices();
//     std::vector<double> properties = m.average_len_valen();
//     wmtk::logger().info(
//         "edgelen: avg max min valence:avg max min before remesh is: {}",
//         properties);
//     wmtk::logger().info("target edge length is: {}", properties[0] * len_rel);
//     m.uniform_remeshing(properties[0] * len_rel, 2);
//     m.consolidate_mesh();
//     m.write_triangle_mesh(output);
//     wmtk::logger().info(
//         "After_vertices#: {} \n\t After_tris#: {}",
//         m.vert_capacity(),
//         m.tri_capacity());
//     REQUIRE(m.check_edge_manifold());
// }

std::function<bool(std::array<double, 6>&)> is_inverted = [](auto& tri) {
    Eigen::Vector2d a, b, c;
    a << tri[0], tri[1];
    b << tri[2], tri[3];
    c << tri[4], tri[5];
    auto res = igl::predicates::orient2d(a, b, c);
    return (res != igl::predicates::Orientation::POSITIVE);
};
std::function<bool(std::array<double, 6>&)> is_dgenerate = [](auto& tri) {
    Eigen::Vector3d a, b;
    a << tri[2] - tri[0], tri[3] - tri[1], 0.;
    b << tri[4] - tri[0], tri[5] - tri[1], 0.;
    auto area = (a.cross(b)).norm();
    auto long_e = std::max(a.norm(), b.norm());
    return (std::pow(area, 2) < 1e-5 || std::pow((area / long_e - 0.01), 2) < 1e-5);
};

TEST_CASE("operation orient", "[test_remeshing]")
{
    const std::string root(WMTK_DATA_DIR);
    const std::string path = root + "/fan.obj";
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);

    REQUIRE(ok);

    std::vector<Eigen::Vector3d> v(V.rows());
    std::vector<std::array<size_t, 3>> tri(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        v[i] = V.row(i);
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
    }
    UniformRemeshing m(v);
    std::vector<std::pair<size_t, int>> modified_v;
    m.create_mesh(V.rows(), tri, modified_v, 1);
    auto fs = m.get_faces();
    for (auto f : fs) {
        std::array<double, 6> tri = {
            m.vertex_attrs[f.vid(m)].pos(0),
            m.vertex_attrs[f.vid(m)].pos(1),
            m.vertex_attrs[f.switch_vertex(m).vid(m)].pos(0),
            m.vertex_attrs[f.switch_vertex(m).vid(m)].pos(1),
            m.vertex_attrs[(f.switch_edge(m)).switch_vertex(m).vid(m)].pos(0),
            m.vertex_attrs[(f.switch_edge(m)).switch_vertex(m).vid(m)].pos(1)};
        REQUIRE(!is_inverted(tri));
    }

    m.split_remeshing();
    fs = m.get_faces();
    for (auto f : fs) {
        std::array<double, 6> tri = {
            m.vertex_attrs[f.vid(m)].pos(0),
            m.vertex_attrs[f.vid(m)].pos(1),
            m.vertex_attrs[f.switch_vertex(m).vid(m)].pos(0),
            m.vertex_attrs[f.switch_vertex(m).vid(m)].pos(1),
            m.vertex_attrs[(f.switch_edge(m)).switch_vertex(m).vid(m)].pos(0),
            m.vertex_attrs[(f.switch_edge(m)).switch_vertex(m).vid(m)].pos(1)};
        REQUIRE(!is_inverted(tri));
    }

    m.collapse_remeshing();
    fs = m.get_faces();
    for (auto f : fs) {
        std::array<double, 6> tri = {
            m.vertex_attrs[f.vid(m)].pos(0),
            m.vertex_attrs[f.vid(m)].pos(1),
            m.vertex_attrs[f.switch_vertex(m).vid(m)].pos(0),
            m.vertex_attrs[f.switch_vertex(m).vid(m)].pos(1),
            m.vertex_attrs[(f.switch_edge(m)).switch_vertex(m).vid(m)].pos(0),
            m.vertex_attrs[(f.switch_edge(m)).switch_vertex(m).vid(m)].pos(1)};
        REQUIRE(!is_inverted(tri));
    }

    m.write_triangle_mesh("old_orient.obj");
}
