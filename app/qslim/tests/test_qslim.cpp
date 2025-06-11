#include <igl/read_triangle_mesh.h>
#include <qslim/QSLIM.h>

#include <catch2/catch_test_macros.hpp>

using namespace wmtk;
using namespace app::qslim;


TEST_CASE("qec", "[test_qslim][.]")
{
    const std::string root(WMTK_DATA_DIR);
    const std::string path = root + "/piece_0.obj";

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);
    wmtk::logger().info("readin mesh is {}", ok);
    REQUIRE(ok);
    wmtk::logger().info("number of vertices {}", V.size());
    std::vector<Eigen::Vector3d> v(V.rows());
    std::vector<std::array<size_t, 3>> tri(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        v[i] = V.row(i);
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
    }
    QSLIM m(v);
    m.create_mesh(V.rows(), tri);
    REQUIRE(m.check_mesh_connectivity_validity());
    REQUIRE(m.collapse_qslim(V.rows() - 2));


    m.write_triangle_mesh("no_qec.obj");
}


TEST_CASE("qec_cost", "[test_qslim]")
{
    const std::string root(WMTK_DATA_DIR);
    const std::string path = root + "/upsample_box.obj";

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);
    wmtk::logger().info("readin mesh is {}", ok);
    REQUIRE(ok);
    wmtk::logger().info("number of vertices {}", V.size());
    std::vector<Eigen::Vector3d> v(V.rows());
    std::vector<std::array<size_t, 3>> tri(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        v[i] = V.row(i);
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
    }
    QSLIM m(v);
    m.create_mesh(V.rows(), tri);
    auto edges = m.get_edges();
    Eigen::MatrixXd writem(edges.size(), 7);

    for (int i = 0; i < edges.size(); i++) {
        Eigen::Vector3d v1 = V.row(edges[i].vid(m));
        Eigen::Vector3d v2 = V.row(edges[i].switch_vertex(m).vid(m));
        writem.row(i) << v1.transpose(), v2.transpose(), m.compute_cost_for_e(edges[i]);
    }
}
