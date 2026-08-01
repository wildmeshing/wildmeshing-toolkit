#pragma once
#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <wmtk/AttributeCollection.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <igl/write_triangle_mesh.h>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include <fastenvelope/FastEnvelope.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <atomic>
#include <memory>
#include <queue>

#include <wmtk/envelope/Envelope.hpp>

namespace wmtk::components::shortest_edge_collapse {

struct VertexAttributes
{
    Eigen::Vector3d pos;
    size_t partition_id = 0;
    bool freeze = false;
};

class ShortestEdgeCollapse : public wmtk::TriMesh
{
public:
    wmtk::SampleEnvelope m_envelope;
    bool m_has_envelope = false;

    // Envelope around the open boundary of the input, used only when the boundary is not
    // frozen. The surface envelope cannot stand in for it: it is a containment test, so a
    // boundary sliding inwards along the surface stays inside it and would go unnoticed.
    // Same construction tetwild uses for its open-boundary edges. Always sampled, never
    // exact -- SampleEnvelope::is_outside throws for edges when use_exact is set.
    wmtk::SampleEnvelope m_boundary_envelope;
    bool m_has_boundary_envelope = false;

    wmtk::AttributeCollection<VertexAttributes> vertex_attrs;

    int retry_limit = 10;
    ShortestEdgeCollapse(
        std::vector<Eigen::Vector3d> _m_vertex_positions,
        int num_threads = 1,
        bool use_exact_envelope = true);

    void freeze_boundary();

    /**
     * @param eps         envelope thickness; 0 disables the envelope entirely
     * @param freeze_bnd  pin the vertices of the open boundary so the outline cannot move.
     *                    When cleared, the boundary is simplified like the rest of the
     *                    surface and is kept in place by m_boundary_envelope instead.
     *
     * Note freeze_bnd comes after eps rather than before it, unlike the otherwise
     * equivalent IsotropicRemeshing::create_mesh: every existing caller passes eps
     * positionally as the fourth argument, and a bool inserted ahead of it would silently
     * swallow that value (double -> bool) and disable the envelope.
     */
    void create_mesh(
        size_t n_vertices,
        const std::vector<std::array<size_t, 3>>& tris,
        const std::vector<size_t>& frozen_verts = {},
        double eps = 0,
        bool freeze_bnd = true);

    ~ShortestEdgeCollapse() {}

    void partition_mesh();

    size_t get_partition_id(const Tuple& loc) const
    {
        return vertex_attrs[loc.vid(*this)].partition_id;
    }

    void write_vtu(const std::string& path);

public:
    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;
    bool collapse_shortest(int target_vertex_count);
    bool write_triangle_mesh(std::string path);
    bool invariants(const std::vector<Tuple>& new_tris) override;

private:
    struct PositionInfoCache
    {
        Eigen::Vector3d v1p;
        Eigen::Vector3d v2p;
        // v1 is the endpoint the collapse removes, v2 the one it keeps.
        bool v1_frozen = false;
        bool v2_frozen = false;
    };
    wmtk::threading::enumerable_thread_specific<PositionInfoCache> position_cache;

    std::vector<TriMesh::Tuple> new_edges_after(const std::vector<TriMesh::Tuple>& t) const;
};

} // namespace wmtk::components::shortest_edge_collapse