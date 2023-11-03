#pragma once
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
#include <fastenvelope/FastEnvelope.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <atomic>
#include <memory>
#include <queue>
#include "envelope/SampleEnvelope.hpp"

namespace app::sec {

struct VertexAttributes
{
    Eigen::Vector3d pos;
    size_t partition_id = 0;
    bool freeze = false;
};

class ShortestEdgeCollapse : public wmtk::TriMesh
{
public:
    sample_envelope::SampleEnvelope m_envelope;
    bool m_has_envelope = false;
    wmtk::AttributeCollection<VertexAttributes> vertex_attrs;

    int retry_limit = 10;
    ShortestEdgeCollapse(
        std::vector<Eigen::Vector3d> _m_vertex_positions,
        int num_threads = 1,
        bool use_exact_envelope = true);

    void set_freeze(TriMesh::Tuple& v);

    void create_mesh_nofreeze(size_t n_vertices, const std::vector<std::array<size_t, 3>>& tris);

    void create_mesh(
        size_t n_vertices,
        const std::vector<std::array<size_t, 3>>& tris,
        const std::vector<size_t>& frozen_verts = std::vector<size_t>(),
        double eps = 0);

    ~ShortestEdgeCollapse() {}

    void partition_mesh();

public:

    bool collapse_edge_before(const Tuple& t) ;
    bool collapse_edge_after(const Tuple& t) ;
    bool collapse_shortest(int target_vertex_count);
    bool write_triangle_mesh(std::string path);
    bool invariants(const wmtk::TriMeshOperation& op) override;
    std::map<std::string, std::shared_ptr<wmtk::TriMeshOperation>> get_operations() const override;

private:
    struct PositionInfoCache
    {
        Eigen::Vector3d v1p;
        Eigen::Vector3d v2p;
    };
    tbb::enumerable_thread_specific<PositionInfoCache> position_cache;

    std::vector<TriMesh::Tuple> new_edges_after(const std::vector<TriMesh::Tuple>& t) const;
};


} // namespace app::sec
