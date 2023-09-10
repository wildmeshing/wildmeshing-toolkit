#pragma once

#include "Mesh.hpp"
#include "Tuple.hpp"

#include <Eigen/Core>

namespace wmtk {

class TriMesh : public Mesh
{
public:
    TriMesh();
    TriMesh(const TriMesh& o);
    TriMesh(TriMesh&& o);
    TriMesh& operator=(const TriMesh& o);
    TriMesh& operator=(TriMesh&& o);

    PrimitiveType top_simplex_type() const override { return PrimitiveType::Face; }
    /**
     * @brief split edge t
     *
     * The returned tuple contains the new vertex. The face lies in the region where the input tuple
     * face was, and the edge is oriented in the same direction as in the input.
     */
    Tuple split_edge(const Tuple& t, Accessor<long>& hash_accessor) override;
    /**
     * @brief collapse edge t
     *
     * The vertex in t is collapsed towards the one where the tuple is pointing to.
     * The returned tuple contains the remaining vertex. The edge represents the face of t that was
     * collapsed. The face is chosen such that the orientation of the tuple is the same as in the
     * input. If this is not possible due to a boundary, the opposite face is chosen.
     */
    Tuple collapse_edge(const Tuple& t, Accessor<long>& hash_accessor) override;

    Tuple switch_tuple(const Tuple& tuple, PrimitiveType type) const override;

    /**
     * @brief jump to the next edge by performing a switch of vertex and edge
     */
    Tuple next_edge(const Tuple& tuple) const { return switch_edge(switch_vertex(tuple)); }
    /**
     * @brief jump to the previous edge by performing a switch of edge and vertex
     */
    Tuple prev_edge(const Tuple& tuple) const { return switch_vertex(switch_edge(tuple)); }

    bool is_ccw(const Tuple& tuple) const override;
    bool is_boundary(const Tuple& tuple) const override;
    bool is_boundary_vertex(const Tuple& tuple) const override;
    bool is_boundary_edge(const Tuple& tuple) const override;

    void initialize(
        Eigen::Ref<const RowVectors3l> FV,
        Eigen::Ref<const RowVectors3l> FE,
        Eigen::Ref<const RowVectors3l> FF,
        Eigen::Ref<const VectorXl> VF,
        Eigen::Ref<const VectorXl> EF);
    void initialize(Eigen::Ref<const RowVectors3l> F);

    long _debug_id(const Tuple& tuple, PrimitiveType type) const;
    long _debug_id(const Simplex& simplex) const
    {
        return _debug_id(simplex.tuple(), simplex.primitive_type());
    }

    bool is_valid(const Tuple& tuple, ConstAccessor<long>& hash_accessor) const override;

    bool is_connectivity_valid() const override;

protected:
    long id(const Tuple& tuple, PrimitiveType type) const override;
    long id(const Simplex& simplex) const { return id(simplex.tuple(), simplex.primitive_type()); }

    long id_vertex(const Tuple& tuple) const { return id(tuple, PrimitiveType::Vertex); }
    long id_edge(const Tuple& tuple) const { return id(tuple, PrimitiveType::Edge); }
    long id_face(const Tuple& tuple) const { return id(tuple, PrimitiveType::Face); }

    /**
     * @brief internal function that returns the tuple of requested type, and has the global index
     * cid
     *
     * @param gid
     * @return Tuple
     */
    Tuple tuple_from_id(const PrimitiveType type, const long gid) const override;

protected:
    attribute::MeshAttributeHandle<long> m_vf_handle;
    attribute::MeshAttributeHandle<long> m_ef_handle;

    attribute::MeshAttributeHandle<long> m_fv_handle;
    attribute::MeshAttributeHandle<long> m_fe_handle;
    attribute::MeshAttributeHandle<long> m_ff_handle;

    Tuple vertex_tuple_from_id(long id) const;
    Tuple edge_tuple_from_id(long id) const;
    Tuple face_tuple_from_id(long id) const;

    // internal structure that encapsulations the actual execution of split and collapse
    class TriMeshOperationExecutor;
    static Tuple with_different_cid(const Tuple& t, long cid);
};

} // namespace wmtk
