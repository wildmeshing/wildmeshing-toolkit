#pragma once

#include <wmtk/operations/tri_mesh/EdgeOperationData.hpp>
#include "Mesh.hpp"
#include "Tuple.hpp"

#include <Eigen/Core>

namespace wmtk {
namespace operations::utils {
class MultiMeshEdgeSplitFunctor;
class MultiMeshEdgeCollapseFunctor;
class UpdateEdgeOperationMultiMeshMapFunctor;
} // namespace operations::utils


class TriMesh : public Mesh
{
public:
    friend class operations::utils::MultiMeshEdgeCollapseFunctor;
    friend class operations::utils::MultiMeshEdgeSplitFunctor;
    friend class operations::utils::UpdateEdgeOperationMultiMeshMapFunctor;
    TriMesh();
    TriMesh(const TriMesh& o);
    TriMesh(TriMesh&& o);
    TriMesh& operator=(const TriMesh& o);
    TriMesh& operator=(TriMesh&& o);

    int64_t top_cell_dimension() const override { return 2; }

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
    using Mesh::is_boundary;
    bool is_boundary(const Tuple& tuple, PrimitiveType pt) const override;
    bool is_boundary_vertex(const Tuple& tuple) const override;
    bool is_boundary_edge(const Tuple& tuple) const override;

    void initialize(
        Eigen::Ref<const RowVectors3l> FV,
        Eigen::Ref<const RowVectors3l> FE,
        Eigen::Ref<const RowVectors3l> FF,
        Eigen::Ref<const VectorXl> VF,
        Eigen::Ref<const VectorXl> EF);
    void initialize(Eigen::Ref<const RowVectors3l> F);

    bool is_valid(const Tuple& tuple, ConstAccessor<int64_t>& hash_accessor) const override;

    bool is_connectivity_valid() const override;

    std::vector<std::vector<TypedAttributeHandle<int64_t>>> connectivity_attributes()
        const override;


#if defined(MTAO_PUBLICIZING_ID)
public: // TODO remove
#else
protected:
#endif
    int64_t id(const Tuple& tuple, PrimitiveType type) const override;
    int64_t id(const simplex::Simplex& simplex) const
    {
        return id(simplex.tuple(), simplex.primitive_type());
    }

    int64_t id_vertex(const Tuple& tuple) const { return id(tuple, PrimitiveType::Vertex); }
    int64_t id_edge(const Tuple& tuple) const { return id(tuple, PrimitiveType::Edge); }
    int64_t id_face(const Tuple& tuple) const { return id(tuple, PrimitiveType::Face); }

    /**
     * @brief internal function that returns the tuple of requested type, and has the global index
     * cid
     *
     * @param gid
     * @return Tuple
     */
    Tuple tuple_from_id(const PrimitiveType type, const int64_t gid) const override;
    Tuple tuple_from_global_ids(int64_t fid, int64_t eid, int64_t vid) const;

protected:
    attribute::TypedAttributeHandle<int64_t> m_vf_handle;
    attribute::TypedAttributeHandle<int64_t> m_ef_handle;

    attribute::TypedAttributeHandle<int64_t> m_fv_handle;
    attribute::TypedAttributeHandle<int64_t> m_fe_handle;
    attribute::TypedAttributeHandle<int64_t> m_ff_handle;

    Tuple vertex_tuple_from_id(int64_t id) const;
    Tuple edge_tuple_from_id(int64_t id) const;
    Tuple face_tuple_from_id(int64_t id) const;


    class TriMeshOperationExecutor;
    static Tuple with_different_cid(const Tuple& t, int64_t cid);
};

} // namespace wmtk
