#pragma once
#include <wmtk/ConcurrentTriMesh.h>
#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/utils/VectorUtils.h>

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
#include <queue>

namespace Edge2d {

class EdgeOperations2d : public wmtk::ConcurrentTriMesh
{
public:
    tbb::concurrent_vector<Eigen::Vector3d> m_vertex_positions;
    tbb::concurrent_vector<size_t> m_vertex_partition_id;
    fastEnvelope::FastEnvelope m_envelope;
    bool m_has_envelope = false;

    int NUM_THREADS = 1;
    int retry_limit = 10;
    EdgeOperations2d(std::vector<Eigen::Vector3d> _m_vertex_positions, int num_threads = 1)
        : NUM_THREADS(num_threads)
    {
        m_vertex_positions.resize(_m_vertex_positions.size());
        for (auto i = 0; i < _m_vertex_positions.size(); i++)
            m_vertex_positions[i] = _m_vertex_positions[i];
    }

    void
    create_mesh(size_t n_vertices, const std::vector<std::array<size_t, 3>>& tris, double eps = 0)
    {
        wmtk::ConcurrentTriMesh::create_mesh(n_vertices, tris);

        if (eps > 0) {
            std::vector<Eigen::Vector3d> V(m_vertex_positions.size());
            std::vector<Eigen::Vector3i> F(tris.size());
            std::copy(m_vertex_positions.begin(), m_vertex_positions.end(), std::back_inserter(V));
            for (int i = 0; i < F.size(); ++i) F[i] << tris[i][0], tris[i][1], tris[i][2];
            m_envelope.init(V, F, eps);
            m_has_envelope = true;
        }
        partition_mesh();
    }


    ~EdgeOperations2d() {}

    struct PositionInfoCache
    {
        Eigen::Vector3d v1p;
        Eigen::Vector3d v2p;
    };
    tbb::enumerable_thread_specific<PositionInfoCache> position_cache;

    void cache_edge_positions(const Tuple& t)
    {
        position_cache.local().v1p = m_vertex_positions[t.vid()];
        position_cache.local().v2p = m_vertex_positions[t.switch_vertex(*this).vid()];
    }

    bool invariants(const std::vector<Tuple>& new_tris)
    {
        if (m_has_envelope) {
            for (auto& t : new_tris) {
                std::array<Eigen::Vector3d, 3> tris;
                auto vs = t.oriented_tri_vertices(*this);
                for (auto j = 0; j < 3; j++) tris[j] = m_vertex_positions[vs[j].vid()];
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
        m_vertex_positions[t.vid()] = p;
        return true;

    }

    void partition_mesh() { m_vertex_partition_id = partition_TriMesh(*this, NUM_THREADS); }

    Eigen::Vector3d smooth(const Tuple& t)
    {
        auto one_ring_edges = get_one_ring_edges_for_vertex(t);
        if (one_ring_edges.size() < 3) return m_vertex_positions[t.vid()];
        Eigen::Vector3d after_smooth(0, 0, 0);
        Eigen::Vector3d after_smooth_boundary(0, 0, 0);
        int boundary = 0;
        for (auto e : one_ring_edges) {
            if (is_boundary_edge(e)) {
                after_smooth_boundary += m_vertex_positions[e.vid()];
                boundary++;
                continue;
            }
            after_smooth += m_vertex_positions[e.vid()];
        }

        if (boundary)
            after_smooth = after_smooth_boundary / boundary;
        else
            after_smooth /= one_ring_edges.size();
        return after_smooth;
    }


    Eigen::Vector3d tangential_smooth(const Tuple& t);

    void move_vertex_attribute(size_t from, size_t to) override
    {
        m_vertex_positions[to] = m_vertex_positions[from];
    }

    // write the collapsed mesh into a obj
    bool write_triangle_mesh(std::string path)
    {
        Eigen::MatrixXd V = Eigen::MatrixXd::Zero(m_vertex_positions.size(), 3);
        for (auto& t : get_vertices()) {
            auto i = t.vid();
            V.row(i) = m_vertex_positions[i];
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
        cache_edge_positions(t);
        return true;
    }

    bool collapse_after(const Tuple& t) override;
    bool swap_after(const Tuple& t) override;

    std::vector<TriMesh::Tuple> new_edges_after(const std::vector<TriMesh::Tuple>& t) const;
    std::vector<TriMesh::Tuple> new_edges_after_swap(const TriMesh::Tuple& t) const;
    // std::vector<TriMesh::Tuple> new_edges_after_collapse_split(const TriMesh::Tuple& t) const;

    bool collapse_shortest(int target_vertex_count);

    Eigen::MatrixXd compute_Q_f(wmtk::TriMesh::Tuple& f_tuple);
    Eigen::MatrixXd compute_Q_v(wmtk::TriMesh::Tuple& v_tuple);
    double compute_cost_for_v(wmtk::TriMesh::Tuple& v_tuple);
    bool collapse_qec(int target_vertcies);

    bool split_before(const Tuple& t) override
    {
        cache_edge_positions(t);
        return true;
    }

    bool split_after(const Tuple& t) override;
    // methods for adaptive remeshing
    double compute_edge_cost_collapse_ar(const TriMesh::Tuple& t, double L) const;
    double compute_edge_cost_split_ar(const TriMesh::Tuple& t, double L) const;
    double compute_vertex_valence_ar(const TriMesh::Tuple& t) const;
    std::vector<double> average_len_valen();
    bool split_remeshing(double L);
    bool collapse_remeshing(double L);
    bool swap_remeshing();
    bool adaptive_remeshing(double L, int interations, int sm);
    void resize_vertex_attributes(size_t v) override
    {
        ConcurrentTriMesh::resize_vertex_attributes(v);
        m_vertex_positions.grow_to_at_least(v);
        m_vertex_partition_id.grow_to_at_least(v);
    }
};

} // namespace Edge2d
