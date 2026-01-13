#include <igl/read_triangle_mesh.h>
#include <igl/split_nonmanifold.h>
#include <wmtk/TetMesh.h>
#include <catch2/catch_test_macros.hpp>

#include <wmtk/utils/ManifoldUtils.hpp>
#include "wmtk/utils/Logger.hpp"

#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <Eigen/Core>

TEST_CASE("separate-manifold-patch", "[test_util]")
{
    std::vector<Eigen::Vector3d> vertices = {
        {Eigen::Vector3d(0, 0, 0),
         Eigen::Vector3d(1, 0, 0),
         Eigen::Vector3d(0, 1, 0),
         Eigen::Vector3d(0, 0, 1),
         Eigen::Vector3d(0, -1, 0)}};
    std::vector<std::array<size_t, 3>> faces = {{
        {{0, 1, 2}},
        {{0, 1, 3}},
        {{0, 1, 4}},
    }};

    std::vector<Eigen::Vector3d> out_v;
    std::vector<std::array<size_t, 3>> out_f;
    std::vector<size_t> freeze_v;
    wmtk::separate_to_manifold(vertices, faces, out_v, out_f, freeze_v);
    REQUIRE(out_v.size() == 9);
    REQUIRE(out_f.size() == 3);
}

TEST_CASE("manifold-separate-test-37989", "[test_util]")
{
    std::string filename = WMTK_DATA_DIR "/37989_sf.obj";
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

TEST_CASE("compare_lagrange_igl", "[test_util][.]")
{
    wmtk::manifold_internal::Vertices V;
    wmtk::manifold_internal::Facets F;

    // SECTION("simple")
    //{
    //     F.resize(2, 3);
    //     F.row(0) << 0, 1, 2;
    //    F.row(1) << 1, 0, 3; // same orientation
    //
    //    V.resize(4, 3);
    //    V.setZero();
    //}
    SECTION("twisted")
    {
        F.resize(2, 3);
        F.row(0) << 0, 1, 2;
        F.row(1) << 0, 1, 3; // different orientation

        V.resize(4, 3);
        V.setZero();
    }
    SECTION("large")
    {
        std::string filename = WMTK_DATA_DIR "/37989_sf.obj";
        igl::read_triangle_mesh(filename, V, F);
        // REQUIRE_FALSE(igl::is_edge_manifold(F));
        // std::vector<size_t> modified_vertices;
        // wmtk::manifold_internal::resolve_nonmanifoldness(V, F, modified_vertices);
        // REQUIRE(modified_vertices.size() > 0);
        // REQUIRE(igl::is_edge_manifold(F));
        // Eigen::VectorXi VI;
        // REQUIRE(igl::is_vertex_manifold(F, VI));
    }

    {
        Eigen::VectorXi SVI;
        wmtk::manifold_internal::Facets SF;
        igl::split_nonmanifold(F, SF, SVI);
        // std::cout << "IGL:\n" << SF << std::endl;

        wmtk::manifold_internal::Vertices V_new;
        V_new.resize(SVI.size(), V.cols());
        std::map<uint64_t, uint64_t> used_vids; // map old vertex ID to first new ID
        std::set<int> modified_vertices;
        for (int i = 0; i < SVI.size(); ++i) {
            auto old_vid = SVI[i];

            V_new.row(i) = V.row(old_vid);
            if (used_vids.count(old_vid) > 0) {
                // vertex was duplicated
                modified_vertices.insert(used_vids[old_vid]);
                modified_vertices.insert(i);
            } else {
                used_vids[old_vid] = i; // store first occurance
            }
        }

        // F = SF;

        // std::cout << SVI.transpose() << std::endl;
        // std::cout << "IGL MOD: ";
        // for (const auto v : modified_vertices) {
        //     std::cout << v << " ";
        // }
        // std::cout << std::endl;
        std::cout << "IGL " << modified_vertices.size() << std::endl;

        CHECK(igl::is_edge_manifold(SF));
        Eigen::VectorXi VI;
        CHECK(igl::is_vertex_manifold(SF, VI));
    }
    {
        std::vector<size_t> modified_vertices;
        wmtk::manifold_internal::resolve_nonmanifoldness(V, F, modified_vertices);
        // std::cout << "LAG:\n" << F << std::endl;
        // std::cout << "LAG MOD: ";
        // for (const auto v : modified_vertices) {
        //     std::cout << v << " ";
        // }
        // std::cout << std::endl;
        std::cout << "LAG " << modified_vertices.size() << std::endl;

        CHECK(igl::is_edge_manifold(F));
        Eigen::VectorXi VI;
        CHECK(igl::is_vertex_manifold(F, VI));
    }
}
