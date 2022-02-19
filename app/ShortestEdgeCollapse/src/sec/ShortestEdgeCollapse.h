#pragma once
#include <wmtk/ConcurrentTriMesh.h>
#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <wmtk/AttributeCollection.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <igl/write_triangle_mesh.h>
#include <tbb/concurrent_priority_queue.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/task_group.h>
#include <fastenvelope/FastEnvelope.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <atomic>
#include <memory>
#include <queue>

namespace sec {

struct VertexAttributes
{
    Eigen::Vector3d pos;
    size_t partition_id = 0;
    bool freeze = false;
};

class ShortestEdgeCollapse : public wmtk::ConcurrentTriMesh
{
public:
    fastEnvelope::FastEnvelope m_envelope;
    bool m_has_envelope = false;
    wmtk::AttributeCollection<VertexAttributes> vertex_attrs;

    int NUM_THREADS = 1;
    int retry_limit = 10;
    ShortestEdgeCollapse(std::vector<Eigen::Vector3d> _m_vertex_positions, int num_threads = 1)
        : NUM_THREADS(num_threads)
    {
        p_vertex_attrs = &vertex_attrs;

        vertex_attrs.resize(_m_vertex_positions.size());

        for (auto i = 0; i < _m_vertex_positions.size(); i++)
            vertex_attrs[i] = {_m_vertex_positions[i], 0, false};
    }

    void set_freeze(TriMesh::Tuple& v)
    {
        for (auto e : get_one_ring_edges_for_vertex(v)) {
            if (is_boundary_edge(e)) {
                vertex_attrs[v.vid()].freeze = true;
                continue;
            }
        }
    }

    void create_mesh(
        size_t n_vertices,
        const std::vector<std::array<size_t, 3>>& tris,
        const std::vector<size_t>& frozen_verts = std::vector<size_t>(),
        double eps = 0)
    {
        wmtk::ConcurrentTriMesh::create_mesh(n_vertices, tris);

        if (eps > 0) {
            std::vector<Eigen::Vector3d> V(n_vertices);
            std::vector<Eigen::Vector3i> F(tris.size());
            for (auto i = 0; i < V.size(); i++) {
                V[i] = vertex_attrs[i].pos;
            }
            for (int i = 0; i < F.size(); ++i) F[i] << tris[i][0], tris[i][1], tris[i][2];
            m_envelope.init(V, F, eps);
            m_has_envelope = true;
        }
        partition_mesh();
        for (auto v : frozen_verts) vertex_attrs[v].freeze = true;
        for (auto v : get_vertices()) { // the better way is to iterate through edges.
            set_freeze(v);
        }
    }


    ~ShortestEdgeCollapse() {}

    void partition_mesh()
    {
        auto m_vertex_partition_id = partition_TriMesh(*this, NUM_THREADS);
        for (auto i = 0; i < m_vertex_partition_id.size(); i++)
            vertex_attrs[i].partition_id = m_vertex_partition_id[i];
    }

public:
    bool collapse_before(const Tuple& t) override;
    bool collapse_after(const Tuple& t) override;
    bool collapse_shortest(int target_vertex_count);
    bool write_triangle_mesh(std::string path);
    bool invariants(const std::vector<Tuple>& new_tris) override;

private:
    struct PositionInfoCache
    {
        Eigen::Vector3d v1p;
        Eigen::Vector3d v2p;
    };
    tbb::enumerable_thread_specific<PositionInfoCache> position_cache;

    std::vector<TriMesh::Tuple> new_edges_after(const std::vector<TriMesh::Tuple>& t) const;
};

} // namespace sec
