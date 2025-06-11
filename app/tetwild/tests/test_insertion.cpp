

#include <TetWild.h>
#include <igl/write_triangle_mesh.h>
#include <wmtk/TetMesh.h>

#include <catch2/catch_test_macros.hpp>
#include "Parameters.h"
#include "sec/envelope/SampleEnvelope.hpp"
#include "spdlog/common.h"
#include "wmtk/utils/InsertTriangleUtils.hpp"

#include <igl/read_triangle_mesh.h>
#include <wmtk/utils/Partitioning.h>

using namespace wmtk;
using namespace tetwild;

TEST_CASE("triangle-insertion", "[tetwild_operation]")
{
    Eigen::MatrixXd V;
    Eigen::MatrixXd F;
    std::string input_path = WMTK_DATA_DIR "/37322.stl";
    igl::read_triangle_mesh(input_path, V, F);
    wmtk::logger().info("Read Mesh V={}, F={}", V.rows(), F.rows());

    std::vector<Vector3d> vertices(V.rows());
    std::vector<std::array<size_t, 3>> faces(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        vertices[i] = V.row(i);
    }
    std::vector<Eigen::Vector3i> env_faces(F.rows()); // todo: add new api for envelope
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            faces[i][j] = F(i, j);
            env_faces[i][j] = F(i, j);
        }
    }

    int NUM_THREADS = 1;
    Parameters params;
    params.lr = 1 / 15.0;
    params.init(vertices, faces);
    wmtk::ExactEnvelope envelope;
    wmtk::logger().info("input_surface.params.eps {}", params.eps);
    envelope.init(vertices, env_faces, params.eps);

    wmtk::remove_duplicates(vertices, faces, params.diag_l);
    std::vector<size_t> partition_id(vertices.size(), 0);
    //
    //
    sample_envelope::SampleEnvelope sample_env;
    tetwild::TetWild mesh(params, envelope, sample_env);

    mesh.init_from_input_surface(vertices, faces, partition_id);
    REQUIRE(mesh.check_attributes());
}


TEST_CASE("triangle-insertion-parallel", "[tetwild_operation][.]")
{
    Eigen::MatrixXd V;
    Eigen::MatrixXd F;
    std::string input_path = WMTK_DATA_DIR "/Octocat.obj";
    igl::read_triangle_mesh(input_path, V, F);
    wmtk::logger().info("Read Mesh V={}, F={}", V.rows(), F.rows());

    std::vector<Vector3d> vertices(V.rows());
    std::vector<std::array<size_t, 3>> faces(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        vertices[i] = V.row(i);
    }
    std::vector<Eigen::Vector3i> env_faces(F.rows()); // todo: add new api for envelope
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            faces[i][j] = F(i, j);
            env_faces[i][j] = F(i, j);
        }
    }

    int NUM_THREADS = 16;

    Parameters params;
    params.lr = 1 / 30.0;
    params.init(vertices, faces);

    wmtk::ExactEnvelope envelope;
    envelope.init(vertices, env_faces, params.eps);

    wmtk::remove_duplicates(vertices, faces, params.diag_l);
    Eigen::MatrixXd new_F(faces.size(), 3);
    for (int i = 0; i < faces.size(); i++) {
        new_F(i, 0) = faces[i][0];
        new_F(i, 1) = faces[i][1];
        new_F(i, 2) = faces[i][2];
    }
    auto partitioned_v = partition_mesh_vertices(new_F, NUM_THREADS);
    std::vector<size_t> partition_id(partitioned_v.rows());

    std::vector<int> cnt_id(NUM_THREADS);
    for (int i = 0; i < partitioned_v.rows(); i++) {
        partition_id[i] = partitioned_v(i, 0);
        cnt_id[partition_id[i]]++;
    }

    //
    //
    sample_envelope::SampleEnvelope sample_env;
    tetwild::TetWild mesh(params, envelope, sample_env, NUM_THREADS);

    wmtk::logger().info("start insertion");
    mesh.init_from_input_surface(vertices, faces, partition_id);
    wmtk::logger().info("end insertion");
}


TEST_CASE("point-insertion")
{
    // class PointInserter : public wmtk::ConcurrentTetMesh
    // {
    // public:
    //     struct VertexAttributes
    //     {
    //         Eigen::Vector3d pos;
    //         size_t partition_id = 0;
    //     };
    //     using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    //     VertAttCol vertex_attrs;
    //     PointInserter(
    //         const std::vector<Eigen::Vector3d>& _vertex_attribute,
    //         const std::vector<std::array<size_t, 4>>& tets,
    //         int num_threads = 1)
    //     {
    //         p_vertex_attrs = &vertex_attrs;

    //         vertex_attrs.resize(_vertex_attribute.size());

    //         for (auto i = 0; i < _vertex_attribute.size(); i++)
    //             vertex_attrs[i].pos = _vertex_attribute[i];

    //         init(_vertex_attribute.size(), tets);
    //     }

    //     struct PointInsertCache
    //     {
    //         Eigen::Vector3d pos;
    //         int flag;
    //     };
    //     tbb::enumerable_thread_specific<PointInsertCache> point_cache;

    //     // input Tet where the point is.
    //     virtual bool insert_point_before(const Tuple& t) override { return true; }
    //     virtual bool insert_point_after(std::vector<Tuple>& t) override
    //     {
    //         if (flag == 0) return
    //             {}
    //         return true;
    //     }
    //     std::tuple<Tuple, int> containing_tet(Eigen::Vector3d);

    //     void insert_point_list(const std::vector<Eigen::Vector3d>& points)
    //     {
    //         for (auto& p : points) {
    //             // 1. find containing tets, use AABB or TetHash
    //             auto [t, flag] = containing_tet(p);
    //             // 2. change topology.
    //             point_cache.local().pos = p;
    //             point_cache.local().flag = flag;
    //             if (flag == 0) continue; // vert
    //             std::vector<Tuple> new_tets;
    //             if (flag == 1) split_edge(t, new_tets);
    //             if (flag == 2) split_face(t, new_tets);
    //             if (flag == 3)
    //                 split_tet(t, new_tets);
    //             else
    //                 assert(false);
    //         }
    //     }
    // };
}
