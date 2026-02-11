

#include <igl/write_triangle_mesh.h>
#include <sec/ShortestEdgeCollapse.h>
#include <wmtk/TetMesh.h>
#include <wmtk/components/tetwild/Parameters.h>
#include <wmtk/components/tetwild/TetWildMesh.h>

#include <catch2/catch_test_macros.hpp>

#include <wmtk/envelope/Envelope.hpp>
#include "spdlog/common.h"
#include "wmtk/utils/InsertTriangleUtils.hpp"

#include <igl/read_triangle_mesh.h>
#include <wmtk/utils/Partitioning.h>

using namespace wmtk;
using namespace components::tetwild;

TEST_CASE("triangle-insertion", "[tetwild_operation][.]")
{
    // Eigen::MatrixXd V;
    // Eigen::MatrixXd F;
    //std::string input_path = WMTK_DATA_DIR "/37322.stl";
    // igl::read_triangle_mesh(input_path, V, F);
    // wmtk::logger().info("Read Mesh V={}, F={}", V.rows(), F.rows());
    //
    // std::vector<Vector3d> vertices(V.rows());
    // std::vector<std::array<size_t, 3>> faces(F.rows());
    // for (int i = 0; i < V.rows(); i++) {
    //     vertices[i] = V.row(i);
    // }
    //std::vector<Eigen::Vector3i> env_faces(F.rows()); // todo: add new api for envelope
    // for (int i = 0; i < F.rows(); i++) {
    //     for (int j = 0; j < 3; j++) {
    //         faces[i][j] = F(i, j);
    //         env_faces[i][j] = F(i, j);
    //     }
    // }
    //
    // int NUM_THREADS = 1;
    // Parameters params;
    //params.lr = 1 / 15.0;
    // params.init(vertices, faces);
    // wmtk::ExactEnvelope envelope;
    // wmtk::logger().info("input_surface.params.eps {}", params.eps);
    // envelope.init(vertices, env_faces, params.eps);
    //
    // wmtk::remove_duplicates(vertices, faces, params.diag_l);
    // std::vector<size_t> partition_id(vertices.size(), 0);
    ////
    ////
    // tetwild::TetWild mesh(params, envelope);
    //
    // mesh.init_from_input_surface(vertices, faces, partition_id);
    // REQUIRE(mesh.check_attributes());
}


TEST_CASE("triangle-insertion-parallel", "[tetwild_operation][.]")
{
    // Eigen::MatrixXd V;
    // Eigen::MatrixXd F;
    //std::string input_path = WMTK_DATA_DIR "/Octocat.obj";
    // igl::read_triangle_mesh(input_path, V, F);
    // wmtk::logger().info("Read Mesh V={}, F={}", V.rows(), F.rows());
    //
    // std::vector<Vector3d> vertices(V.rows());
    // std::vector<std::array<size_t, 3>> faces(F.rows());
    // for (int i = 0; i < V.rows(); i++) {
    //     vertices[i] = V.row(i);
    // }
    //std::vector<Eigen::Vector3i> env_faces(F.rows()); // todo: add new api for envelope
    // for (int i = 0; i < F.rows(); i++) {
    //     for (int j = 0; j < 3; j++) {
    //         faces[i][j] = F(i, j);
    //         env_faces[i][j] = F(i, j);
    //     }
    // }
    //
    // int NUM_THREADS = 16;
    //
    // Parameters params;
    //params.lr = 1 / 30.0;
    // params.init(vertices, faces);
    //
    // wmtk::ExactEnvelope envelope;
    // envelope.init(vertices, env_faces, params.eps);
    //
    // wmtk::remove_duplicates(vertices, faces, params.diag_l);
    // Eigen::MatrixXd new_F(faces.size(), 3);
    // for (int i = 0; i < faces.size(); i++) {
    //     new_F(i, 0) = faces[i][0];
    //     new_F(i, 1) = faces[i][1];
    //     new_F(i, 2) = faces[i][2];
    // }
    // auto partitioned_v = partition_mesh_vertices(new_F, NUM_THREADS);
    // std::vector<size_t> partition_id(partitioned_v.rows());
    //
    // std::vector<int> cnt_id(NUM_THREADS);
    // for (int i = 0; i < partitioned_v.rows(); i++) {
    //     partition_id[i] = partitioned_v(i, 0);
    //     cnt_id[partition_id[i]]++;
    // }
    //
    ////
    ////
    // tetwild::TetWild mesh(params, envelope, NUM_THREADS);
    //
    // wmtk::logger().info("start insertion");
    // mesh.init_from_input_surface(vertices, faces, partition_id);
    // wmtk::logger().info("end insertion");
}

TEST_CASE("vertex_order", "[tetwild][.]")
{
    using Tuple = TetMesh::Tuple;

    MatrixXd V;
    MatrixXi F;

    size_t nvo3 = 0; // number of order 3 vertices
    SECTION("shark-fin")
    {
        // one-sided shark fin
        V.resize(9, 3);
        V.row(0) = Vector3d(0, 0, 0);
        V.row(1) = Vector3d(2, 0, 0);
        V.row(2) = Vector3d(3, 0, 0);
        V.row(3) = Vector3d(1, 1, 0);
        V.row(4) = Vector3d(3, 1, 0);
        V.row(5) = Vector3d(0, 2, 0);
        V.row(6) = Vector3d(2, 2, 0);
        V.row(7) = Vector3d(3, 2, 0);
        V.row(8) = Vector3d(2, 1, 1);
        F.resize(8, 3);
        F.row(0) = Vector3i(0, 1, 3);
        F.row(1) = Vector3i(1, 4, 3);
        F.row(2) = Vector3i(1, 2, 4);
        F.row(3) = Vector3i(0, 3, 5);
        F.row(4) = Vector3i(3, 6, 5);
        F.row(5) = Vector3i(3, 4, 6);
        F.row(6) = Vector3i(4, 7, 6);
        F.row(7) = Vector3i(3, 4, 8);

        nvo3 = 2;
    }
    SECTION("two-triangles")
    {
        // two triangles touching in one vertex
        V.resize(5, 3);
        V.row(0) = Vector3d(0, 0, 0);
        V.row(1) = Vector3d(2, 0, 0);
        V.row(2) = Vector3d(1, 1, 0);
        V.row(3) = Vector3d(0, 2, 0);
        V.row(4) = Vector3d(2, 2, 0);
        F.resize(2, 3);
        F.row(0) = Vector3i(0, 1, 2);
        F.row(1) = Vector3i(2, 4, 3);

        nvo3 = 1;
    }
    SECTION("two-triangles-disconnected")
    {
        // two triangles touching in one vertex
        V.resize(6, 3);
        V.row(0) = Vector3d(0, 0, 0);
        V.row(1) = Vector3d(2, 0, 0);
        V.row(2) = Vector3d(1, 1, 0);
        V.row(3) = Vector3d(0, 2, 0);
        V.row(4) = Vector3d(2, 2, 0);
        V.row(5) = Vector3d(1, 1.1, 0);
        F.resize(2, 3);
        F.row(0) = Vector3i(0, 1, 2);
        F.row(1) = Vector3i(3, 4, 5);

        nvo3 = 0;
    }
    SECTION("hour-glass")
    {
        V.resize(7, 3);
        V.row(0) = Vector3d(0, 0, 0);
        V.row(1) = Vector3d(0, -1, 0);
        V.row(2) = Vector3d(-1, 0, 0);
        V.row(3) = Vector3d(0, 0, -1);
        V.row(4) = Vector3d(1, 0, 0);
        V.row(5) = Vector3d(0, 0, 1);
        V.row(6) = Vector3d(0, 1, 0);
        F.resize(8, 3);
        F.row(0) = Vector3i(0, 2, 1);
        F.row(1) = Vector3i(0, 1, 3);
        F.row(2) = Vector3i(2, 0, 3);
        F.row(3) = Vector3i(1, 2, 3);
        F.row(4) = Vector3i(4, 5, 0);
        F.row(5) = Vector3i(0, 5, 6);
        F.row(6) = Vector3i(4, 0, 6);
        F.row(7) = Vector3i(6, 5, 4);

        nvo3 = 1;
    }

    std::vector<Vector3d> vertices;
    std::vector<std::array<size_t, 3>> faces;

    VF_to_vectors(V, F, vertices, faces);

    Parameters params;
    params.init(vertices, faces);

    app::sec::ShortestEdgeCollapse surf_mesh(vertices, 0);
    {
        std::vector<size_t> frozen_verts;
        surf_mesh.create_mesh(vertices.size(), faces, frozen_verts, 0.1);
    }
    REQUIRE(surf_mesh.check_mesh_connectivity_validity());


    std::vector<Vector3r> v_rational;
    std::vector<std::array<size_t, 3>> facets;
    std::vector<bool> is_v_on_input;
    std::vector<std::array<size_t, 4>> tets;
    std::vector<bool> tet_face_on_input_surface;
    {
        TetWildMesh mesh_insertion(params, surf_mesh.m_envelope, 0);
        mesh_insertion.insertion_by_volumeremesher_old(
            vertices,
            faces,
            v_rational,
            facets,
            is_v_on_input,
            tets,
            tet_face_on_input_surface);
    }

    // generate new mesh
    TetWildMesh mesh(params, surf_mesh.m_envelope, 0);

    mesh.init_from_Volumeremesher(
        v_rational,
        facets,
        is_v_on_input,
        tets,
        tet_face_on_input_surface);

    mesh.init_vertex_order();

    // mesh.save_paraview("debug_shark_fin", false);

    // check order of vertices
    size_t vo3_count = 0;
    for (const Tuple& t : mesh.get_vertices()) {
        const size_t vid = t.vid(mesh);
        const size_t vo = mesh.get_order_of_vertex(vid);
        if (vo == 3) {
            ++vo3_count;
        }

        if (vo == 2) {
            // must have order 2 edges incident
            const auto vs = mesh.get_one_ring_vids_for_vertex(vid);
            size_t eo2_count = 0;
            for (const size_t v : vs) {
                if (mesh.get_order_of_edge({{vid, v}}) == 2) {
                    ++eo2_count;
                }
            }
            CHECK(eo2_count == 2);
        }
    }

    CHECK(nvo3 == vo3_count);
}