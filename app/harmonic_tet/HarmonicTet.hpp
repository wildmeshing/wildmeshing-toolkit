#pragma once

#include <wmtk/ConcurrentTetMesh.h>
#include <wmtk/AttributeCollection.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <tbb/enumerable_thread_specific.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include "wmtk/utils/Logger.hpp"
#include <wmtk/utils/partition_utils.hpp>

#include <Eigen/Core>
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

    void compute_vertex_partition_morton() const
    {
        if (NUM_THREADS==0) return;
        wmtk::logger().info("Number of parts: {} by morton", NUM_THREADS);
        std::vector<size_t> par;
        wmtk::partition_vertex_morton(*this, [&](auto vid){
            return vertex_attrs[i].pos;
        }, NUM_THREDS, par
        );
        for_each_vertex([&](auto& v){
            auto vid = v.vid(*this);
            vertex_attrs[vid].partition_id = par[vid];
        });
    }

    void output_mesh(std::string file) const;
    void compute_vertex_partition()
    {
        auto partition_id = partition_TetMesh(*this, NUM_THREADS);
        for (auto i = 0; i < vert_capacity(); i++)
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
