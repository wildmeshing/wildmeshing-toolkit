#pragma once
#include <igl/per_face_normals.h>
#include <wmtk/TriMesh.h>
#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <wmtk/AttributeCollection.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <igl/write_triangle_mesh.h>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/envelope/BoundaryEnvelope.hpp>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <atomic>
#include <memory>
#include <queue>

namespace wmtk::components::qslim {

struct Quadrics
{
    Eigen::Matrix3d A;
    Eigen::Vector3d b;
    double c;
};

struct VertexAttributes
{
    Eigen::Vector3d pos;
    size_t partition_id = 0;
    bool freeze = false;
    Quadrics Q;
    /// The vertex is, or was merged from, a vertex on a boundary edge of the input. Only set
    /// when create_mesh() gets a boundary_eps: it marks which boundary edges the boundary
    /// envelope judges. See wmtk::BoundaryEnvelope.
    bool input_boundary = false;
};

struct FaceAttributes
{
    Quadrics Q;
    Eigen::Vector3d n = Eigen::Vector3d::Zero(); // for quadrics computation
};

struct EdgeAttributes
{
    Eigen::Vector3d vbar; // for quadrics computation
};

class QSlimMesh : public wmtk::TriMesh
{
public:
    // wmtk::ExactEnvelope m_envelope;
    wmtk::SampleEnvelope m_envelope;
    bool m_has_envelope = false;
    /// Tube around the boundary edges of the mesh given to create_mesh(), when that gets a
    /// positive boundary_eps. See there. initialized() is false otherwise, and for a closed input.
    wmtk::BoundaryEnvelope m_boundary_envelope;
    wmtk::AttributeCollection<VertexAttributes> vertex_attrs;
    wmtk::AttributeCollection<FaceAttributes> face_attrs;
    wmtk::AttributeCollection<EdgeAttributes> edge_attrs;

    int retry_limit = 10;
    QSlimMesh(std::vector<Eigen::Vector3d> _m_vertex_positions, int num_threads = 1);
    void set_freeze(TriMesh::Tuple& v);

    ~QSlimMesh() {}

    /**
     * @param eps          surface envelope thickness; 0 = no envelope
     * @param boundary_eps how the open boundary is held.
     *   0 (default): only by the surface envelope, if there is one. That is a containment
     *     test, so it does not notice an outline retracting along the surface; nothing here
     *     freezes the boundary either (frozen_verts is ignored).
     *   > 0: every boundary edge a collapse leaves that descends from the input's boundary
     *     (touches an input_boundary vertex) must also lie within boundary_eps of the boundary
     *     edges of this input, the same tube ShortestEdgeCollapse uses instead of freezing. The
     *     placement follows the outline rather than the quadric's free optimum, which nothing in
     *     the face quadrics keeps on the boundary: a collapse along the boundary goes to the
     *     optimum on the edge, one between a boundary vertex and an interior one onto the
     *     boundary vertex (see compute_cost_for_e). Holes and slits narrower than the
     *     tube can close, but only with set_use_link_condition(false): the collapse that closes
     *     one is exactly what the link condition refuses, and it is on unless turned off (the
     *     qslim application does not). Boundary a collapse opens elsewhere is not judged.
     */
    void create_mesh(
        size_t n_vertices,
        const std::vector<std::array<size_t, 3>>& tris,
        const std::vector<size_t>& frozen_verts = std::vector<size_t>(),
        double eps = 0,
        double boundary_eps = 0);

    void initiate_quadrics_for_face();

    void initiate_quadrics_for_vertices();

    void partition_mesh()
    {
        auto m_vertex_partition_id = partition_TriMesh(*this, NUM_THREADS);
        for (auto i = 0; i < m_vertex_partition_id.size(); i++)
            vertex_attrs[i].partition_id = m_vertex_partition_id[i];
    }

    // TODO: This should not be exposed to the application, but hidden in wmtk
    void partition_mesh_morton();

    size_t get_partition_id(const Tuple& loc) const
    {
        return vertex_attrs[loc.vid(*this)].partition_id;
    }

public:
    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;
    bool collapse_qslim(int target_vertex_count);
    bool write_triangle_mesh(std::string path);
    bool invariants(const std::vector<Tuple>& new_tris) override;
    double compute_cost_for_e(const TriMesh::Tuple& v_tuple);
    Quadrics compute_quadric_for_face(const TriMesh::Tuple& f_tuple);
    void update_quadrics(const TriMesh::Tuple& v_tuple);

private:
    struct InfoCache
    {
        Eigen::Vector3d v1p;
        Eigen::Vector3d v2p;
        Eigen::Vector3d vbar;
        Quadrics Q1;
        Quadrics Q2;
        int partition_id;
        bool input_boundary = false; // of either endpoint, for the survivor
    };
    wmtk::threading::enumerable_thread_specific<InfoCache> cache;

    std::vector<TriMesh::Tuple> new_edges_after(const std::vector<TriMesh::Tuple>& t) const;
};

} // namespace wmtk::components::qslim
