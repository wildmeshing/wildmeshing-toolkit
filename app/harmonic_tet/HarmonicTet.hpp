#pragma once

#include <wmtk/ConcurrentTetMesh.h>
#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/AttributeCollection.hpp>

#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>
#include <Eigen/Core>

// for morton partition
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <tbb/task_arena.h>
#include <wmtk/utils/Morton.h>
#include "wmtk/utils/Logger.hpp"

#include <atomic>
#include <memory>

namespace harmonic_tet {

class HarmonicTet : public wmtk::ConcurrentTetMesh
{
public:
    struct VertexAttributes
    {
        Eigen::Vector3d pos;
        size_t partition_id = 0;
    };
    using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    VertAttCol vertex_attrs;

    HarmonicTet(
        const std::vector<Eigen::Vector3d>& _vertex_attribute,
        const std::vector<std::array<size_t, 4>>& tets,
        int num_threads = 1)
    {
        p_vertex_attrs = &vertex_attrs;

        vertex_attrs.resize(_vertex_attribute.size());

        for (auto i = 0; i < _vertex_attribute.size(); i++)
            vertex_attrs[i].pos = _vertex_attribute[i];

        NUM_THREADS = num_threads;
        init(_vertex_attribute.size(), tets);

        // compute_vertex_partition();
        compute_vertex_partition_morton();
    }
    HarmonicTet(){};
    ~HarmonicTet(){};

    ////// Attributes related

    void compute_vertex_partition_morton()
    {
        if (NUM_THREADS==0) return;
       
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
            // std::vector<sortstruct> list;
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

            // get_bb_corners(V, vmin, vmax);
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

    void output_mesh(std::string file) const;
    void compute_vertex_partition()
    {
        auto partition_id = partition_TetMesh(*this, NUM_THREADS);
        for (auto i = 0; i < vertex_attrs.size(); i++)
            vertex_attrs[i].partition_id = partition_id[i];
    }
    size_t get_partition_id(const Tuple& loc) const
    {
        return vertex_attrs[loc.vid(*this)].partition_id;
    }

    // parallel containers
    ////// Operations

    struct SwapInfoCache
    {
        double total_energy = 0.;
    };
    tbb::enumerable_thread_specific<SwapInfoCache> edgeswap_cache, faceswap_cache;

    void smooth_all_vertices(bool interior_only = false);
    bool smooth_after(const Tuple& t) override;

    void swap_all_edges(bool parallel = false);
    bool swap_edge_before(const Tuple& t) override;
    bool swap_edge_after(const Tuple& t) override;

    int swap_all();
    void swap_all_faces(bool parallel = false);
    bool swap_face_before(const Tuple& t) override;
    bool swap_face_after(const Tuple& t) override;

    bool is_inverted(const Tuple& loc);
    double get_quality(const Tuple& loc) const;
    double get_quality(const std::array<size_t, 4>& vids) const;

    bool invariants(const std::vector<Tuple>&) override;
};

} // namespace harmonic_tet
