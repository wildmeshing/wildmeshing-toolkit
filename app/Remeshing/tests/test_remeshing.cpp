#include <igl/read_triangle_mesh.h>
#include <remeshing/UniformRemeshing.h>

#include <catch2/catch.hpp>


using namespace wmtk;
using namespace remeshing;

TEST_CASE("adaptive_remeshing", "[test_remeshing][.]")
{
    const std::string root(WMT_DATA_DIR);
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
    m.create_mesh(V.rows(), tri);
    REQUIRE(m.check_mesh_connectivity_validity());
    REQUIRE(m.uniform_remeshing(0.01, 5));
}

TEST_CASE("split_each_edge", "[test_remeshing]")
{
    std::vector<Eigen::Vector3d> v_positions(3);
    v_positions[0] = Eigen::Vector3d(-3, 3, 0);
    v_positions[1] = Eigen::Vector3d(0, 3, 0);

    v_positions[2] = Eigen::Vector3d(0, 0, 0);

    UniformRemeshing m(v_positions);
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}};
    m.create_mesh(3, tris);
    m.uniform_remeshing(0.1, 5);
}

TEST_CASE("test_swap", "[test_remeshing]")
{
    const std::string root(WMT_DATA_DIR);
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
    m.create_mesh(V.rows(), tri);
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
    const std::string root(WMT_DATA_DIR);
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
    m.create_mesh(V.rows(), tri);

    auto edges = m.get_edges();
    for (auto edge : edges) {
        if (!edge.is_valid(m)) continue;
        std::vector<TriMesh::Tuple> dummy;
        m.split_edge(edge, dummy);
    }
    // m.write_triangle_mesh("split.obj");
    REQUIRE(m.check_mesh_connectivity_validity());
}