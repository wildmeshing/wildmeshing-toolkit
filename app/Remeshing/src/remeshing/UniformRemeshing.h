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
#include <tbb/parallel_sort.h>
#include <fastenvelope/FastEnvelope.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <Eigen/Core>
#include <Eigen/Geometry>


#include <atomic>
#include <queue>
#include "wmtk/AttributeCollection.hpp"
namespace remeshing {

struct VertexAttributes
{
    Eigen::Vector3d pos;
    // TODO: in fact, partition id should not be vertex attribute, it is a fixed marker to distinguish tuple/operations.
    size_t partition_id;
    bool freeze = false;
};

class UniformRemeshing : public wmtk::ConcurrentTriMesh
{
public:
    fastEnvelope::FastEnvelope m_envelope;
    bool m_has_envelope = false;
    using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    VertAttCol vertex_attrs;

    int NUM_THREADS = 1;
    int retry_limit = 10;
    UniformRemeshing(std::vector<Eigen::Vector3d> _m_vertex_positions, int num_threads = 1)
        : NUM_THREADS(num_threads)
    {
        p_vertex_attrs = &vertex_attrs;

        vertex_attrs.resize(_m_vertex_positions.size());

        for (auto i = 0; i < _m_vertex_positions.size(); i++)
            vertex_attrs[i] = {_m_vertex_positions[i], 0};
    }

    void create_mesh(
        size_t n_vertices,
        const std::vector<std::array<size_t, 3>>& tris,
        const std::vector<size_t>& frozen_verts = std::vector<size_t>(),
        bool m_freeze = true,
        double eps = 0)
    {
        wmtk::ConcurrentTriMesh::create_mesh(n_vertices, tris);
        std::vector<Eigen::Vector3d> V(n_vertices);
        std::vector<Eigen::Vector3i> F(tris.size());
        for (auto i = 0; i < V.size(); i++) {
            V[i] = vertex_attrs[i].pos;
        }
        for (int i = 0; i < F.size(); ++i) F[i] << tris[i][0], tris[i][1], tris[i][2];
        if (eps > 0) {
            m_envelope.init(V, F, eps);
            m_has_envelope = true;
        } else
            m_envelope.init(V, F, 0.0);


        partition_mesh_morton();
        for (auto v : frozen_verts) vertex_attrs[v].freeze = true;
        if (m_freeze) {
            for (auto e : get_edges()) {
                if (is_boundary_edge(e)) {
                    vertex_attrs[e.vid(*this)].freeze = true;
                    vertex_attrs[e.switch_vertex(*this).vid(*this)].freeze = true;
                }
            }
        }
    }


    ~UniformRemeshing() {}

    struct PositionInfoCache
    {
        Eigen::Vector3d v1p;
        Eigen::Vector3d v2p;
    };
    tbb::enumerable_thread_specific<PositionInfoCache> position_cache;

    void cache_edge_positions(const Tuple& t)
    {
        position_cache.local().v1p = vertex_attrs[t.vid(*this)].pos;
        position_cache.local().v2p = vertex_attrs[t.switch_vertex(*this).vid(*this)].pos;
    }

    bool invariants(const std::vector<Tuple>& new_tris) override
    {
        if (m_has_envelope) {
            for (auto& t : new_tris) {
                std::array<Eigen::Vector3d, 3> tris;
                auto vs = t.oriented_tri_vertices(*this);
                for (auto j = 0; j < 3; j++) tris[j] = vertex_attrs[vs[j].vid(*this)].pos;
                if (m_envelope.is_outside(tris)) {
                    return false;
                }
            }
        }
        return true;
    }


    void partition_mesh()
    {
        auto m_vertex_partition_id = partition_TriMesh(*this, NUM_THREADS);
        for (auto i = 0; i < m_vertex_partition_id.size(); i++)
            vertex_attrs[i].partition_id = m_vertex_partition_id[i];
    }

    void partition_mesh_morton()
    {
        if (NUM_THREADS == 0) return;
        wmtk::logger().info("Number of parts: {} by morton", NUM_THREADS);

        tbb::task_arena arena(NUM_THREADS);

        arena.execute([&] {
            std::vector<Eigen::Vector3d> V_v(vertex_attrs.size());

            tbb::parallel_for(
                tbb::blocked_range<int>(0, V_v.size()),
                [&](tbb::blocked_range<int> r) {
                    for (int i = r.begin(); i < r.end(); i++) {
                        V_v[i] = vertex_attrs[i].pos;
                    }
                });

            struct sortstruct
            {
                int order;
                Resorting::MortonCode64 morton;
            };

            std::vector<sortstruct> list_v;
            list_v.resize(V_v.size());
            const int multi = 1000;
            // since the morton code requires a correct scale of input vertices,
            //  we need to scale the vertices if their coordinates are out of range
            std::vector<Eigen::Vector3d> V = V_v; // this is for rescaling vertices
            Eigen::Vector3d vmin, vmax;
            vmin = V.front();
            vmax = V.front();

            for (size_t j = 0; j < V.size(); j++) {
                for (int i = 0; i < 3; i++) {
                    vmin(i) = std::min(vmin(i), V[j](i));
                    vmax(i) = std::max(vmax(i), V[j](i));
                }
            }

            Eigen::Vector3d center = (vmin + vmax) / 2;

            tbb::parallel_for(tbb::blocked_range<int>(0, V.size()), [&](tbb::blocked_range<int> r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    V[i] = V[i] - center;
                }
            });

            Eigen::Vector3d scale_point =
                vmax - center; // after placing box at origin, vmax and vmin are symetric.

            double xscale, yscale, zscale;
            xscale = fabs(scale_point[0]);
            yscale = fabs(scale_point[1]);
            zscale = fabs(scale_point[2]);
            double scale = std::max(std::max(xscale, yscale), zscale);
            if (scale > 300) {
                tbb::parallel_for(
                    tbb::blocked_range<int>(0, V.size()),
                    [&](tbb::blocked_range<int> r) {
                        for (int i = r.begin(); i < r.end(); i++) {
                            V[i] = V[i] / scale;
                        }
                    });
            }

            tbb::parallel_for(tbb::blocked_range<int>(0, V.size()), [&](tbb::blocked_range<int> r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    list_v[i].morton = Resorting::MortonCode64(
                        int(V[i][0] * multi),
                        int(V[i][1] * multi),
                        int(V[i][2] * multi));
                    list_v[i].order = i;
                }
            });

            const auto morton_compare = [](const sortstruct& a, const sortstruct& b) {
                return (a.morton < b.morton);
            };

            tbb::parallel_sort(list_v.begin(), list_v.end(), morton_compare);

            int interval = list_v.size() / NUM_THREADS + 1;

            tbb::parallel_for(
                tbb::blocked_range<int>(0, list_v.size()),
                [&](tbb::blocked_range<int> r) {
                    for (int i = r.begin(); i < r.end(); i++) {
                        vertex_attrs[list_v[i].order].partition_id = i / interval;
                    }
                });
        });
    }


    Eigen::Vector3d smooth(const Tuple& t);


    Eigen::Vector3d tangential_smooth(const Tuple& t);


    bool is_edge_freeze(const Tuple& t)
    {
        if (vertex_attrs[t.vid(*this)].freeze ||
            vertex_attrs[t.switch_vertex(*this).vid(*this)].freeze)
            return true;
        return false;
    }

    bool collapse_edge_before(const Tuple& t) override
    {
        if (!TriMesh::collapse_edge_before(t)) return false;
        if (is_edge_freeze(t)) return false;
        cache_edge_positions(t);
        return true;
    }
    bool collapse_edge_after(const Tuple& t) override;

    bool swap_edge_before(const Tuple& t) override
    {
        if (!TriMesh::swap_edge_before(t)) return false;
        if (is_edge_freeze(t)) return false;
        return true;
    }
    bool swap_edge_after(const Tuple& t) override;

    std::vector<TriMesh::Tuple> new_edges_after(const std::vector<TriMesh::Tuple>& t) const;
    std::vector<TriMesh::Tuple> new_edges_after_swap(const TriMesh::Tuple& t) const;

    bool split_edge_before(const Tuple& t) override
    {
        if (!TriMesh::split_edge_before(t)) return false;
        if (is_edge_freeze(t)) return false;
        cache_edge_positions(t);
        return true;
    }
    bool split_edge_after(const Tuple& t) override;

    bool smooth_before(const Tuple& t) override
    {
        if (vertex_attrs[t.vid(*this)].freeze) return false;
        return true;
    }
    bool smooth_after(const Tuple& t) override;

    double compute_edge_cost_collapse(const TriMesh::Tuple& t, double L) const;
    double compute_edge_cost_split(const TriMesh::Tuple& t, double L) const;
    double compute_vertex_valence(const TriMesh::Tuple& t) const;
    std::vector<double> average_len_valen();
    bool split_remeshing(double L);
    bool collapse_remeshing(double L);
    bool swap_remeshing();
    bool uniform_remeshing(double L, int interations);
    bool write_triangle_mesh(std::string path);
};

} // namespace remeshing
