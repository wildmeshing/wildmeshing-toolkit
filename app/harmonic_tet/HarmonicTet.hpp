#pragma once

#include <wmtk/ConcurrentTetMesh.h>
#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/AttributeCollection.hpp>

#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>
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

        compute_vertex_partition();

    }
    HarmonicTet(){};
    ~HarmonicTet(){};

    ////// Attributes related

    void output_mesh(std::string file) const;
    void compute_vertex_partition() {
        auto partition_id = partition_TetMesh(*this, NUM_THREADS);
        for (auto i = 0; i < vertex_attrs.size(); i++)
            vertex_attrs[i].partition_id = partition_id[i];
    }
    size_t get_partition_id(const Tuple& loc) const {
        return vertex_attrs[loc.vid(*this)].partition_id;
    }

    // parallel containers
    int NUM_THREADS = 1;
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

    void swap_all();
    void swap_all_faces(bool parallel = false);
    bool swap_face_before(const Tuple& t) override;
    bool swap_face_after(const Tuple& t) override;

    bool is_inverted(const Tuple& loc);
    double get_quality(const Tuple& loc) const;
    double get_quality(const std::array<size_t, 4>& vids) const;

    bool invariants(const std::vector<Tuple>&) override;
};

} // namespace harmonic_tet
