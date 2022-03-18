

#include <TetWild.h>
#include <igl/write_triangle_mesh.h>
#include <wmtk/TetMesh.h>

#include <catch2/catch.hpp>
#include "spdlog/common.h"
#include "wmtk/ConcurrentTetMesh.h"

#include <igl/read_triangle_mesh.h>

using namespace wmtk;
using namespace tetwild;

TEST_CASE("triangle-insertion", "[tetwild_operation]")
{
    Eigen::MatrixXd V;
    Eigen::MatrixXd F;
    std::string input_path = WMT_DATA_DIR "/37322.stl";
    igl::read_triangle_mesh(input_path, V, F);
    wmtk::logger().info("Read Mesh V={}, F={}", V.rows(), F.rows());

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

    int NUM_THREADS = 1;
    tetwild::TetWild::InputSurface input_surface;
    input_surface.params.lr = 1 / 15.0;
    input_surface.init(vertices, faces);
    input_surface.remove_duplicates(input_surface.params.diag_l);
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
    wmtk::logger().info("input_surface.params.eps {}", input_surface.params.eps);
    envelope.init(vertices, env_faces, input_surface.params.eps);
    //
    tetwild::TetWild mesh(input_surface.params, envelope);


    mesh.triangle_insertion(input_surface);
    mesh.check_attributes();
}


TEST_CASE("triangle-insertion-parallel", "[tetwild_operation]")
{
    Eigen::MatrixXd V;
    Eigen::MatrixXd F;
    std::string input_path = WMT_DATA_DIR "/Octocat.obj";
    igl::read_triangle_mesh(input_path, V, F);
    wmtk::logger().info("Read Mesh V={}, F={}", V.rows(), F.rows());

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

    int NUM_THREADS = 16;

    tetwild::TetWild::InputSurface input_surface;
    input_surface.params.lr = 1 / 30.0;
    input_surface.init(vertices, faces);
    input_surface.remove_duplicates(input_surface.params.diag_l);
    Eigen::MatrixXd new_F(input_surface.faces.size(), 3);
    for (int i = 0; i < input_surface.faces.size(); i++) {
        new_F(i, 0) = input_surface.faces[i][0];
        new_F(i, 1) = input_surface.faces[i][1];
        new_F(i, 2) = input_surface.faces[i][2];
    }
    auto partitioned_v = partition_mesh_vertices(new_F, NUM_THREADS);
    std::vector<int> partition_id(partitioned_v.rows());

    std::vector<int> cnt_id(NUM_THREADS);
    for (int i = 0; i < partitioned_v.rows(); i++) {
        partition_id[i] = partitioned_v(i, 0);
        // std::cout<<partition_id[i]<<" ";
        cnt_id[partition_id[i]]++;
    }
    for (int i = 0; i < NUM_THREADS; i++) {
        std::cout << i << ": " << cnt_id[i] << std::endl;
    }
    input_surface.partition_id = partition_id;


    // exit(0);
    //
    fastEnvelope::FastEnvelope envelope;
    envelope.init(vertices, env_faces, input_surface.params.eps);
    //
    tetwild::TetWild mesh(input_surface.params, envelope, NUM_THREADS);

    wmtk::logger().info("start insertion");
    mesh.triangle_insertion(input_surface);
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
    //     virtual bool single_point_insertion_before(const Tuple& t) override { return true; }
    //     virtual bool single_point_insertion_after(std::vector<Tuple>& t) override
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