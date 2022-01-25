#pragma once
#include <wmtk/ConcurrentTriMesh.h>
#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/utils/VectorUtils.h>

// clang-format off
#include <memory>
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
#include <queue>
#include "wmtk/AttributeCollection.hpp"

namespace sec {

struct VertexAttributes
{
    Eigen::Vector3d pos;
    // TODO: in fact, partition id should not be vertex attribute, it is a fixed marker to distinguish tuple/operations.
    size_t partition_id;
    bool freeze;
};

class ShortestEdgeCollapse : public wmtk::ConcurrentTriMesh
{
public:
    fastEnvelope::FastEnvelope m_envelope;
    bool m_has_envelope = false;
    using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    std::shared_ptr<VertAttCol> vertex_attrs;

    int NUM_THREADS = 1;
    int retry_limit = 10;
    ShortestEdgeCollapse(std::vector<Eigen::Vector3d> _m_vertex_positions, int num_threads = 1)
        : NUM_THREADS(num_threads)
    {
        vertex_attrs = std::make_shared<VertAttCol>();
        TriMesh::vertex_attrs = vertex_attrs;

        vertex_attrs->resize(_m_vertex_positions.size());

        for (auto i = 0; i < _m_vertex_positions.size(); i++)
            vertex_attrs->m_attributes[i] = {_m_vertex_positions[i], 0, false};
    }

    void
    create_mesh(size_t n_vertices, const std::vector<std::array<size_t, 3>>& tris, double eps = 0)
    {
        wmtk::ConcurrentTriMesh::create_mesh(n_vertices, tris);

        if (eps > 0) {
            std::vector<Eigen::Vector3d> V(n_vertices);
            std::vector<Eigen::Vector3i> F(tris.size());
            for (auto i = 0; i < V.size(); i++) {
                V[i] = vertex_attrs->m_attributes[i].pos;
            }
            for (int i = 0; i < F.size(); ++i) F[i] << tris[i][0], tris[i][1], tris[i][2];
            m_envelope.init(V, F, eps);
            m_has_envelope = true;
        }
        partition_mesh();
        for (auto v : get_vertices()) {
            for (auto e : get_one_ring_edges_for_vertex(v)) {
                if (is_boundary_edge(e)) {
                    vertex_attrs->m_attributes[v.vid()].freeze = true;
                    continue;
                }
            }
        }
    }


    ~ShortestEdgeCollapse() {}

    struct PositionInfoCache
    {
        Eigen::Vector3d v1p;
        Eigen::Vector3d v2p;
    };
    tbb::enumerable_thread_specific<PositionInfoCache> position_cache;

    void cache_edge_positions(const Tuple& t)
    {
        position_cache.local().v1p = vertex_attrs->m_attributes[t.vid()].pos;
        position_cache.local().v2p = vertex_attrs->m_attributes[t.switch_vertex(*this).vid()].pos;
    }

    bool invariants(const std::vector<Tuple>& new_tris) override
    {
        if (m_has_envelope) {
            for (auto& t : new_tris) {
                std::array<Eigen::Vector3d, 3> tris;
                auto vs = t.oriented_tri_vertices(*this);
                for (auto j = 0; j < 3; j++) tris[j] = vertex_attrs->m_attributes[vs[j].vid()].pos;
                if (m_envelope.is_outside(tris)) return false;
            }
        }
        return true;
    }

    bool update_position_to_edge_midpoint(const Tuple& t)
    {
        const Eigen::Vector3d p = (position_cache.local().v1p + position_cache.local().v2p) / 2.0;
        if (m_has_envelope) {
            if (m_envelope.is_outside(p)) return false;
        }
        vertex_attrs->m_attributes[t.vid()].pos = p;
        return true;
    }

    void partition_mesh()
    {
        auto m_vertex_partition_id = partition_TriMesh(*this, NUM_THREADS);
        for (auto i = 0; i < m_vertex_partition_id.size(); i++)
            vertex_attrs->m_attributes[i].partition_id = m_vertex_partition_id[i];
    }


    // write the collapsed mesh into a obj
    bool write_triangle_mesh(std::string path)
    {
        Eigen::MatrixXd V = Eigen::MatrixXd::Zero(vertex_attrs->m_attributes.size(), 3);
        for (auto& t : get_vertices()) {
            auto i = t.vid();
            V.row(i) = vertex_attrs->m_attributes[i].pos;
        }

        Eigen::MatrixXi F = Eigen::MatrixXi::Constant(tri_capacity(), 3, -1);
        for (auto& t : get_faces()) {
            auto i = t.fid();
            auto vs = oriented_tri_vertices(t);
            for (int j = 0; j < 3; j++) {
                F(i, j) = vs[j].vid();
            }
        }

        return igl::write_triangle_mesh(path, V, F);
    }

    bool collapse_before(const Tuple& t) override
    {
        if (!TriMesh::collapse_before(t)) return false;
        if (vertex_attrs->m_attributes[t.vid()].freeze ||
            vertex_attrs->m_attributes[t.switch_vertex(*this).vid()].freeze)
            return false;
        cache_edge_positions(t);
        return true;
    }

    bool collapse_after(const Tuple& t) override;

    std::vector<TriMesh::Tuple> new_edges_after(const std::vector<TriMesh::Tuple>& t) const;

    bool collapse_shortest(int target_vertex_count);
};

} // namespace sec
