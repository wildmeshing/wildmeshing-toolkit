#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <wmtk/components/isotropic_remeshing/IsotropicRemeshing.h>
#include <catch2/catch_test_macros.hpp>
#include <wmtk/utils/predicates.hpp>

using namespace wmtk;
using namespace components::isotropic_remeshing;

TEST_CASE("uniform_remeshing", "[test_remeshing][.]")
{
    const std::string root(WMTK_DATA_DIR);
    const std::string path = root + "/models/circle.obj";
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
    IsotropicRemeshing m(v);
    std::vector<size_t> modified_v;
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

    IsotropicRemeshing m(v_positions, 0);
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}};
    std::vector<size_t> modified_v;
    m.create_mesh(3, tris, modified_v, 0);
    int target_vertnum = m.vert_capacity() + 3 * m.get_edges().size() + 3 * m.tri_capacity();
    m.split_remeshing(2.7 / 2);
    m.consolidate_mesh();
    REQUIRE(target_vertnum == 15);

    REQUIRE(m.vert_capacity() == target_vertnum);
}

TEST_CASE("test_swap", "[test_remeshing]")
{
    const std::string root(WMTK_DATA_DIR);
    const std::string path = root + "/models/circle.obj";

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
    IsotropicRemeshing m(v);
    std::vector<size_t> modified_v;
    m.create_mesh(V.rows(), tri, modified_v, 0);
    // Swaps any edge, whether its quad is convex or not: this tests the connectivity.
    m.set_check_inversions(false);
    int v_invariant = m.get_vertices().size();
    int e_invariant = m.get_edges().size();
    REQUIRE(m.check_mesh_connectivity_validity());
    auto edges = m.get_edges();
    std::vector<TriMesh::Tuple> new_e;
    int cnt = 0;
    for (auto edge : edges) {
        if (cnt > 200) {
            break;
        }
        if (!edge.is_valid(m)) {
            continue;
        }
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
    const std::string path = root + "/models/fan.obj";
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
    IsotropicRemeshing m(v);
    std::vector<size_t> modified_v;
    m.create_mesh(V.rows(), tri, modified_v, 0);
    m.split_remeshing(m.average_len_valen()[0] * 0.5);
    REQUIRE(m.check_mesh_connectivity_validity());
}

TEST_CASE("remeshing_hanging", "[test_remeshing]")
{
    const std::string root(WMTK_DATA_DIR);
    const std::string path = root + "/models/100071_sf.obj";
    std::string output = "100071_out.obj";
    double env_rel = 1e-3;
    double len_rel = 5;
    int thread = 0;

    wmtk::logger().info("remeshing on {}", path);
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);
    Eigen::VectorXi SVI, SVJ;
    Eigen::MatrixXd temp_V = V; // for STL file
    igl::remove_duplicate_vertices(temp_V, 0, V, SVI, SVJ);
    for (int i = 0; i < F.rows(); i++)
        for (int j : {0, 1, 2}) F(i, j) = SVJ[F(i, j)];
    wmtk::logger().info("Before_vertices#: {} \n Before_tris#: {}", V.rows(), F.rows());


    std::vector<Eigen::Vector3d> v(V.rows());
    std::vector<std::array<size_t, 3>> tri(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        v[i] = V.row(i);
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
    }

    const Eigen::MatrixXd box_min = V.colwise().minCoeff();
    const Eigen::MatrixXd box_max = V.colwise().maxCoeff();
    const double diag = (box_max - box_min).norm();
    const double envelope_size = env_rel * diag;
    Eigen::VectorXi dummy;
    std::vector<size_t> modified_v;

    IsotropicRemeshing m(v, thread);
    m.create_mesh(v.size(), tri, modified_v, envelope_size);
    REQUIRE(m.check_edge_manifold());
    m.get_vertices();
    std::vector<double> properties = m.average_len_valen();
    wmtk::logger().info(
        "edgelen: avg max min valence:avg max min before remesh is: {}",
        properties);
    wmtk::logger().info("target edge length is: {}", properties[0] * len_rel);
    m.uniform_remeshing(properties[0] * len_rel, 2);
    m.consolidate_mesh();
    m.write_triangle_mesh(output);
    wmtk::logger().info(
        "After_vertices#: {} \n\t After_tris#: {}",
        m.vert_capacity(),
        m.tri_capacity());
    REQUIRE(m.check_edge_manifold());
}

std::function<bool(std::array<double, 6>&)> is_inverted = [](auto& tri) {
    Eigen::Vector2d a, b, c;
    a << tri[0], tri[1];
    b << tri[2], tri[3];
    c << tri[4], tri[5];
    auto res = wmtk::utils::predicates::orient2d(a, b, c);
    return (res != wmtk::utils::predicates::Orientation::POSITIVE);
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
    const std::string path = root + "/models/fan.obj";
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
    IsotropicRemeshing m(v);
    std::vector<size_t> modified_v;
    m.create_mesh(V.rows(), tri, modified_v, 1);
    auto fs = m.get_faces();
    for (const auto& f : fs) {
        std::array<double, 6> tri = {
            {m.vertex_attrs[f.vid(m)].pos(0),
             m.vertex_attrs[f.vid(m)].pos(1),
             m.vertex_attrs[f.switch_vertex(m).vid(m)].pos(0),
             m.vertex_attrs[f.switch_vertex(m).vid(m)].pos(1),
             m.vertex_attrs[(f.switch_edge(m)).switch_vertex(m).vid(m)].pos(0),
             m.vertex_attrs[(f.switch_edge(m)).switch_vertex(m).vid(m)].pos(1)}};
        REQUIRE(!is_inverted(tri));
    }

    m.split_remeshing(0.8);
    fs = m.get_faces();
    for (const auto& f : fs) {
        std::array<double, 6> tri = {
            {m.vertex_attrs[f.vid(m)].pos(0),
             m.vertex_attrs[f.vid(m)].pos(1),
             m.vertex_attrs[f.switch_vertex(m).vid(m)].pos(0),
             m.vertex_attrs[f.switch_vertex(m).vid(m)].pos(1),
             m.vertex_attrs[(f.switch_edge(m)).switch_vertex(m).vid(m)].pos(0),
             m.vertex_attrs[(f.switch_edge(m)).switch_vertex(m).vid(m)].pos(1)}};
        REQUIRE(!is_inverted(tri));
    }

    m.collapse_remeshing(1.5);
    fs = m.get_faces();
    for (const auto& f : fs) {
        std::array<double, 6> tri = {
            {m.vertex_attrs[f.vid(m)].pos(0),
             m.vertex_attrs[f.vid(m)].pos(1),
             m.vertex_attrs[f.switch_vertex(m).vid(m)].pos(0),
             m.vertex_attrs[f.switch_vertex(m).vid(m)].pos(1),
             m.vertex_attrs[(f.switch_edge(m)).switch_vertex(m).vid(m)].pos(0),
             m.vertex_attrs[(f.switch_edge(m)).switch_vertex(m).vid(m)].pos(1)}};
        REQUIRE(!is_inverted(tri));
    }

    m.write_triangle_mesh("old_orient.obj");
}

namespace {
/// Triangles of a mesh in the plane z = 0 that are clockwise, or flat.
size_t not_counter_clockwise(const IsotropicRemeshing& m)
{
    size_t n = 0;
    for (const auto& f : m.get_faces()) {
        const auto vs = m.oriented_tri_vids(f);
        const auto p = [&](int k) { return Eigen::Vector2d(m.vertex_attrs[vs[k]].pos.head<2>()); };
        if (wmtk::utils::predicates::orient2d(p(0), p(1), p(2)) !=
            wmtk::utils::predicates::Orientation::POSITIVE) {
            ++n;
        }
    }
    return n;
}
} // namespace

TEST_CASE("inversion-check-refuses-smoothing-that-folds", "[test_remeshing]")
{
    // A star with a long spike: the average of its link is out along the spike, past where the
    // vertex can still see the two edges next to it, so tangential smoothing turns those over.
    const std::vector<Eigen::Vector3d> v = {
        {0, 0, 0},
        {6, -0.1, 0},
        {6, 0.1, 0},
        {0.2, 0.2, 0},
        {-1, 1, 0},
        {-1, -1, 0},
        {0.2, -0.2, 0}};
    const std::vector<std::array<size_t, 3>> tris =
        {{{0, 1, 2}}, {{0, 2, 3}}, {{0, 3, 4}}, {{0, 4, 5}}, {{0, 5, 6}}, {{0, 6, 1}}};
    for (const bool check : {false, true}) {
        INFO("check " << check);
        IsotropicRemeshing m(v, 0);
        m.create_mesh(v.size(), tris, {}, true); // the link is the boundary, frozen
        m.set_check_inversions(check);
        REQUIRE(not_counter_clockwise(m) == 0);
        m.smooth_all_vertices();
        if (check) {
            CHECK(m.vertex_attrs[0].pos == Eigen::Vector3d::Zero());
            CHECK(m.refused_inversions() > 0);
            CHECK(not_counter_clockwise(m) == 0);
        } else {
            CHECK(m.vertex_attrs[0].pos.x() > 1);
            CHECK(not_counter_clockwise(m) > 0);
        }
    }
}

TEST_CASE("inversion-check-refuses-swap-that-folds", "[test_remeshing]")
{
    // Two triangles on the edge (0, 1) making a quad that is not convex at 1: the other diagonal
    // (2, 3) runs outside it, and one of the two triangles it makes is turned over.
    const std::vector<Eigen::Vector3d> v = {{0, 0, 0}, {2, 0, 0}, {1, 1, 0}, {3, -0.1, 0}};
    const std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}, {{1, 0, 3}}};
    for (const bool check : {false, true}) {
        INFO("check " << check);
        IsotropicRemeshing m(v, 0);
        m.create_mesh(v.size(), tris, {}, false);
        m.set_check_inversions(check);
        REQUIRE(not_counter_clockwise(m) == 0);
        std::vector<TriMesh::Tuple> new_tris;
        const bool swapped = m.swap_edge(m.tuple_from_edge(0, 1, 0), new_tris);
        CHECK(swapped == !check);
        CHECK(m.refused_inversions() == (check ? 1 : 0));
        CHECK((not_counter_clockwise(m) > 0) == !check);
    }
}

TEST_CASE("limit-collapse-length", "[test_remeshing]")
{
    // A grid of unit squares whose edges are all under 4/5 of the target: every collapse is
    // wanted, and many leave longer edges than the split threshold, 4/3 of the target.
    const int n = 10;
    std::vector<Eigen::Vector3d> v;
    std::vector<std::array<size_t, 3>> tris;
    for (int i = 0; i <= n; ++i) {
        for (int j = 0; j <= n; ++j) v.emplace_back(i, j, 0);
    }
    const auto id = [](int i, int j) { return size_t(i * (n + 1) + j); };
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            tris.push_back({{id(i, j), id(i + 1, j), id(i + 1, j + 1)}});
            tris.push_back({{id(i, j), id(i + 1, j + 1), id(i, j + 1)}});
        }
    }
    const double L = 1.8;
    for (const bool limit : {false, true}) {
        INFO("limit " << limit);
        IsotropicRemeshing m(v, 0);
        m.create_mesh(v.size(), tris);
        m.set_limit_collapse_length(limit);
        m.collapse_remeshing(L);
        REQUIRE(m.check_mesh_connectivity_validity());
        double longest = 0;
        for (const auto& e : m.get_edges()) {
            longest = std::max(
                longest,
                (m.vertex_attrs[e.vid(m)].pos - m.vertex_attrs[e.switch_vertex(m).vid(m)].pos)
                    .norm());
        }
        CHECK(m.get_vertices().size() < v.size());
        CHECK((longest <= 4. / 3. * L) == limit);
    }
}
