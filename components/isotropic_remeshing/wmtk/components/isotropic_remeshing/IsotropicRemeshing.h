#pragma once

#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <wmtk/envelope/BoundaryEnvelope.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include "wmtk/AttributeCollection.hpp"

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

namespace wmtk::components::isotropic_remeshing {

struct VertexAttributes
{
    Eigen::Vector3d pos;
    // TODO: in fact, partition id should not be vertex attribute, it is a fixed marker to distinguish tuple/operations.
    size_t partition_id;
    bool freeze = false;
    /// The vertex is, or descends from, a vertex on a boundary edge of the input: merged from
    /// one by a collapse, or inserted by a split on a boundary edge touching one. Only set when
    /// create_mesh() gets a boundary_eps: it marks which boundary edges the boundary envelope
    /// judges. See wmtk::BoundaryEnvelope.
    bool input_boundary = false;
};

class IsotropicRemeshing : public wmtk::TriMesh
{
public:
    wmtk::SampleEnvelope m_envelope;
    bool m_has_envelope = false;
    /// Tube around the boundary edges of the mesh given to create_mesh(), used instead of
    /// freezing the boundary when that gets a positive boundary_eps. See there.
    /// initialized() is false otherwise, and for a closed input.
    wmtk::BoundaryEnvelope m_boundary_envelope;

    using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    VertAttCol vertex_attrs;

    int retry_limit = 10;
    IsotropicRemeshing(
        std::vector<Eigen::Vector3d> _m_vertex_positions,
        int num_threads = 1,
        bool use_exact = true);

    ~IsotropicRemeshing() {}

    /**
     * @param m_freeze     freeze frozen_verts and, unless boundary_eps is positive, every
     *                     vertex of the open boundary
     * @param eps          surface envelope thickness; 0 = no envelope
     * @param boundary_eps how the open boundary is held.
     *   0 (default): m_freeze decides -- frozen, or held only by the surface envelope, which is
     *     a containment test and does not notice an outline retracting along the surface.
     *   > 0: no boundary vertex is frozen; instead every boundary edge an operation leaves that
     *     descends from the input's boundary (touches an input_boundary vertex) must lie within
     *     boundary_eps of the boundary edges of this input -- the tube ShortestEdgeCollapse
     *     uses. It judges all four operations: a smoothed boundary vertex has to keep its
     *     boundary edges in the tube, a collapse between a boundary vertex and an interior one
     *     keeps the boundary vertex in place (the midpoint would pull the outline into the
     *     surface for the tube to refuse), and the vertex splitting a boundary edge inherits
     *     the lineage. Holes and slits narrower than the tube can close when the link
     *     condition is off. Boundary an operation opens elsewhere is not judged.
     */
    void create_mesh(
        size_t n_vertices,
        const std::vector<std::array<size_t, 3>>& tris,
        const std::vector<size_t>& frozen_verts = std::vector<size_t>(),
        bool m_freeze = true,
        double eps = 0,
        double boundary_eps = 0);

    struct PositionInfoCache
    {
        Eigen::Vector3d v1p;
        Eigen::Vector3d v2p;
        size_t partition_id;
        // Collapse: of t.vid() (v1) and of the other endpoint (v2). All false without a
        // boundary envelope, where no vertex is flagged.
        bool v1_input_boundary = false;
        bool v2_input_boundary = false;
        // An input_boundary vertex that is still on the boundary.
        bool v1_on_boundary = false;
        bool v2_on_boundary = false;
        // Split: the new vertex lands on a boundary edge that touches an input_boundary vertex.
        bool split_input_boundary = false;
    };
    wmtk::threading::enumerable_thread_specific<PositionInfoCache> position_cache;

    void cache_edge_positions(const Tuple& t);

    bool invariants(const std::vector<Tuple>& new_tris) override;

    // TODO: this should not be here
    void partition_mesh();

    // TODO: morton should not be here, but inside wmtk
    void partition_mesh_morton();

    size_t get_partition_id(const Tuple& loc) const
    {
        return vertex_attrs[loc.vid(*this)].partition_id;
    }

    bool smooth_all_vertices();

    Eigen::Vector3d smooth(const Tuple& t);


    Eigen::Vector3d tangential_smooth(const Tuple& t);

    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;

    bool swap_edge_before(const Tuple& t) override;
    bool swap_edge_after(const Tuple& t) override;

    std::vector<TriMesh::Tuple> new_edges_after(const std::vector<TriMesh::Tuple>& tris) const;
    std::vector<TriMesh::Tuple> new_edges_after_swap(const TriMesh::Tuple& t) const;
    std::vector<TriMesh::Tuple> replace_edges_after_split(
        const std::vector<TriMesh::Tuple>& tris,
        const size_t vid_threshold) const;
    std::vector<TriMesh::Tuple> new_sub_edges_after_split(
        const std::vector<TriMesh::Tuple>& tris) const;


    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& t) override;

    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;

    double compute_edge_cost_collapse(const TriMesh::Tuple& t, double L) const;
    double compute_edge_cost_split(const TriMesh::Tuple& t, double L) const;
    double compute_vertex_valence(const TriMesh::Tuple& t) const;
    /**
     * @brief Report statistics.
     *
     * Returns a vector with:
     * average_length
     * max length
     * min length
     * average valence
     * max valence
     * min valence
     */
    std::vector<double> average_len_valen();
    bool split_remeshing(double L);
    bool collapse_remeshing(double L);
    bool swap_remeshing();
    bool uniform_remeshing(double L, int interations);
    bool write_triangle_mesh(std::string path);
};

} // namespace wmtk::components::isotropic_remeshing
