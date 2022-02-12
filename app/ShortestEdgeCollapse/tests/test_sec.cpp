#include <sec/ShortestEdgeCollapse.h>

#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/read_triangle_mesh.h>
#include <Eigen/Core>
#include <catch2/catch.hpp>
#include <wmtk/utils/ManifoldUtils.hpp>
#include "wmtk/utils/Logger.hpp"
using namespace sec;
TEST_CASE("separate-manifold-patch", "[test_util]")
{
    std::vector<Eigen::Vector3d> v = {
        {Eigen::Vector3d(0, 0, 0),
         Eigen::Vector3d(1, 0, 0),
         Eigen::Vector3d(0, 1, 0),
         Eigen::Vector3d(0, 0, 1),
         Eigen::Vector3d(0, -1, 0)}};
    std::vector<std::array<size_t, 3>> tris = {{
        {{0, 1, 2}},
        {{0, 1, 3}},
        {{0, 1, 4}},
    }};

    std::vector<Eigen::Vector3d> out_v;
    std::vector<std::array<size_t, 3>> out_f;
    std::vector<size_t> freeze_v;
    wmtk::separate_to_manifold(v, tris, out_v, out_f, freeze_v);
    REQUIRE(out_v.size() == 9);
    REQUIRE(out_f.size() == 3);

    // after extraction the output should be manifold
    Eigen::VectorXi dummy;
    Eigen::MatrixXd F(out_f.size(), 3);
    for (int i = 0; i < out_f.size(); i++) {
        F.row(i) << out_f[i][0], out_f[i][1], out_f[i][2];
    }
    REQUIRE(igl::is_edge_manifold(F));

    ShortestEdgeCollapse m(out_v);
    m.create_mesh(out_v.size(), out_f, freeze_v, 0);
    m.collapse_shortest(-1);
    Eigen::MatrixXi Fafter = Eigen::MatrixXi::Constant(m.tri_capacity(), 3, -1);
    for (auto& t : m.get_faces()) {
        auto i = t.fid(m);
        auto vs = m.oriented_tri_vertices(t);
        Fafter.row(i) << vs[0].vid(m), vs[1].vid(m), vs[2].vid(m);
    }
    // after operations the mesh is manifold
    REQUIRE(igl::is_edge_manifold(Fafter));
    REQUIRE(igl::is_vertex_manifold(Fafter, dummy));
}

TEST_CASE("manifold-separate-test-37989", "[test_util]")
{
    std::string filename = WMT_DATA_DIR "/37989_sf.obj";
    wmtk::manifold_internal::Vertices V;
    wmtk::manifold_internal::Facets F;
    igl::read_triangle_mesh(filename, V, F);
    REQUIRE_FALSE(igl::is_edge_manifold(F));
    std::vector<size_t> modified_vertices;
    wmtk::manifold_internal::resolve_nonmanifoldness(V, F, modified_vertices);
    REQUIRE(modified_vertices.size() > 0);
    REQUIRE(igl::is_edge_manifold(F));
    Eigen::VectorXi VI;
    REQUIRE(igl::is_vertex_manifold(F, VI));
}