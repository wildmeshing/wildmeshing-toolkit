#pragma once
#include <igl/per_face_normals.h>
#include <wmtk/ConcurrentTriMesh.h>
#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <wmtk/AttributeCollection.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <igl/write_triangle_mesh.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <tbb/task_arena.h>
#include <fastenvelope/FastEnvelope.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <atomic>
#include <memory>
#include <queue>

namespace app::qslim {

struct Quadrics
{
    Eigen::Matrix3d A;
    Eigen::Vector3d b;
    double c;
};

struct VertexAttributes
{
    Eigen::Vector3d pos;
    size_t partition_id = 0;
    bool freeze = false;
    Quadrics Q;
};

struct FaceAttributes
{
    Quadrics Q;
    Eigen::Vector3d n = Eigen::Vector3d::Zero(); // for quadrics computation
};

struct EdgeAttributes
{
    Eigen::Vector3d vbar; // for quadrics computation
};

class QSLIM : public wmtk::ConcurrentTriMesh
{
public:
    fastEnvelope::FastEnvelope m_envelope;
    bool m_has_envelope = false;
    wmtk::AttributeCollection<VertexAttributes> vertex_attrs;
    wmtk::AttributeCollection<FaceAttributes> face_attrs;
    wmtk::AttributeCollection<EdgeAttributes> edge_attrs;

    int retry_limit = 10;
    QSLIM(std::vector<Eigen::Vector3d> _m_vertex_positions, int num_threads = 1);
    void set_freeze(TriMesh::Tuple& v);

    ~QSLIM() {}

    void create_mesh(
        size_t n_vertices,
        const std::vector<std::array<size_t, 3>>& tris,
        const std::vector<size_t>& frozen_verts = std::vector<size_t>(),
        double eps = 0);

    void initiate_quadrics_for_face();

    void initiate_quadrics_for_vertices();

    void partition_mesh()
    {
        auto m_vertex_partition_id = partition_TriMesh(*this, NUM_THREADS);
        for (auto i = 0; i < m_vertex_partition_id.size(); i++)
            vertex_attrs[i].partition_id = m_vertex_partition_id[i];
    }

    // TODO: This should not be exposed to the application, but hidden in wmtk
    void partition_mesh_morton();
    
public:
    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;
    bool collapse_qslim(int target_vertex_count);
    bool write_triangle_mesh(std::string path);
    bool invariants(const std::vector<Tuple>& new_tris) override;
    double compute_cost_for_e(const TriMesh::Tuple& v_tuple);
    Quadrics compute_quadric_for_face(const TriMesh::Tuple& f_tuple);
    void update_quadrics(const TriMesh::Tuple& v_tuple);

private:
    struct InfoCache
    {
        Eigen::Vector3d v1p;
        Eigen::Vector3d v2p;
        Eigen::Vector3d vbar;
        Quadrics Q1;
        Quadrics Q2;
        int partition_id;
    };
    tbb::enumerable_thread_specific<InfoCache> cache;

    std::vector<TriMesh::Tuple> new_edges_after(const std::vector<TriMesh::Tuple>& t) const;
};

} // namespace app::qslim
