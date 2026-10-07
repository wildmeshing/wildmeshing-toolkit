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
    /// The vertex is, or was merged from, a vertex on a boundary edge of the input. Only set
    /// when create_mesh() gets a boundary_eps: it marks which boundary edges the boundary
    /// envelope judges.
    bool input_boundary = false;
};

class ShortestEdgeCollapse : public wmtk::TriMesh
{
public:
    wmtk::SampleEnvelope m_envelope;
    bool m_has_envelope = false;
    /// Tube around the boundary edges of the mesh given to create_mesh(), used instead of
    /// freezing the boundary when create_mesh() gets a positive boundary_eps. See there.
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
     * @param eps          surface envelope thickness; 0 = no envelope
     * @param boundary_eps how the open boundary is kept in place.
     *   0 (default): every boundary vertex is frozen. The outline survives exactly, and
     *     everything enclosed by boundary loops too close together to leave a free vertex
     *     between them survives with it.
     *   > 0: no boundary vertex is frozen; instead every boundary edge a collapse leaves that
     *     descends from the input's boundary (touches an input_boundary vertex) must lie within
     *     boundary_eps of the boundary edges of this input. The outline can then coarsen, and
     *     holes and slits smaller than the tube close. Boundary that a collapse opens elsewhere
     *     is not judged, as with a frozen boundary: with the link condition off, simplifying a
     *     dense closed patch passes through such transient tears, and refusing them leaves it
     *     stuck as a non-manifold tangle (Thingi10K 55928, the second eye).
     */
    void create_mesh(
        size_t n_vertices,
        const std::vector<std::array<size_t, 3>>& tris,
        const std::vector<size_t>& frozen_verts = {},
        double eps = 0,
        double boundary_eps = 0);

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
        bool v1_input_boundary = false;
        bool v2_input_boundary = false;
        // An input_boundary vertex that is still on the boundary. Only computed with a boundary
        // envelope; with a frozen boundary they stay false.
        bool v1_on_boundary = false;
        bool v2_on_boundary = false;
    };
    wmtk::threading::enumerable_thread_specific<PositionInfoCache> position_cache;

    std::vector<TriMesh::Tuple> new_edges_after(const std::vector<TriMesh::Tuple>& t) const;
    void init_boundary_envelope(size_t n_vertices, double boundary_eps);
    /// Every boundary edge of @p tris that touches an input_boundary vertex lies in
    /// m_boundary_envelope.
    bool boundary_edges_inside(const std::vector<Tuple>& tris) const;
};

} // namespace wmtk::components::shortest_edge_collapse