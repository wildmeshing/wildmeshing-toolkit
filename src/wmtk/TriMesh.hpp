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

    Tuple split_edge(const Tuple& t) override;
    Tuple collapse_edge(const Tuple& t) override;

    Tuple switch_tuple(const Tuple& tuple, PrimitiveType type) const override;

    Tuple switch_vertex(const Tuple& tuple) const
    {
        return switch_tuple(tuple, PrimitiveType::Vertex);
    }
    Tuple switch_edge(const Tuple& tuple) const { return switch_tuple(tuple, PrimitiveType::Edge); }
    Tuple switch_face(const Tuple& tuple) const { return switch_tuple(tuple, PrimitiveType::Face); }

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

    void initialize(
        Eigen::Ref<const RowVectors3l> FV,
        Eigen::Ref<const RowVectors3l> FE,
        Eigen::Ref<const RowVectors3l> FF,
        Eigen::Ref<const VectorXl> VF,
        Eigen::Ref<const VectorXl> EF);
    void initialize(Eigen::Ref<const RowVectors3l> F);

    long _debug_id(const Tuple& tuple, PrimitiveType type) const;

    bool is_valid(const Tuple& tuple) const override;

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
    MeshAttributeHandle<long> m_vf_handle;
    MeshAttributeHandle<long> m_ef_handle;

    MeshAttributeHandle<long> m_fv_handle;
    MeshAttributeHandle<long> m_fe_handle;
    MeshAttributeHandle<long> m_ff_handle;

    Tuple vertex_tuple_from_id(long id) const;
    Tuple edge_tuple_from_id(long id) const;
    Tuple face_tuple_from_id(long id) const;

    // internal structure that encapsulations the actual execution of split and collapse
    class TriMeshOperationExecutor;
    static Tuple with_different_cid(const Tuple& t, long cid);
};

} // namespace wmtk
