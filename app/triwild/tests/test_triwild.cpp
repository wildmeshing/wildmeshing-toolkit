#include <TriWild.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <remeshing/UniformRemeshing.h>
#include <catch2/catch.hpp>
#include <wmtk/utils/ManifoldUtils.hpp>

using namespace wmtk;
using namespace triwild;

TEST_CASE("triwild_collapse", "[triwild_collapse][.]")
{
    // dummy case. Collapse 5 times. 1 tri
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
    TriWild m;
    m.create_mesh(V, F);
    m.collapse_all_edges();
    m.consolidate_mesh();
    REQUIRE(m.vertex_attrs[0].pos == Eigen::Vector2d(0., 5.));
    REQUIRE(m.vertex_attrs[1].pos == Eigen::Vector2d(-5., 2.5));
    REQUIRE(m.vertex_attrs[2].pos == Eigen::Vector2d(2.5, 1.875));
}
