#pragma once

#include <wmtk/operations/tet_mesh/EdgeOperationData.hpp>
#include "Mesh.hpp"

namespace wmtk {
namespace operations::utils {
class MultiMeshEdgeSplitFunctor;
class MultiMeshEdgeCollapseFunctor;
class UpdateEdgeOperationMultiMeshMapFunctor;
} // namespace operations::utils
class TetMesh : public Mesh
{
public:
    friend class operations::utils::MultiMeshEdgeSplitFunctor;
    friend class operations::utils::MultiMeshEdgeCollapseFunctor;
    friend class operations::utils::UpdateEdgeOperationMultiMeshMapFunctor;
    TetMesh();
    TetMesh(const TetMesh& o);
    TetMesh(TetMesh&& o);
    TetMesh& operator=(const TetMesh& o);
    TetMesh& operator=(TetMesh&& o);

    int64_t top_cell_dimension() const override { return 3; }
    Tuple switch_tuple(const Tuple& tuple, PrimitiveType type) const override;
    bool is_ccw(const Tuple& tuple) const override;
    using Mesh::is_boundary;
    bool is_boundary(PrimitiveType pt, const Tuple& tuple ) const override;
    bool is_boundary_vertex(const Tuple& tuple) const ;
    bool is_boundary_edge(const Tuple& tuple) const ;
    bool is_boundary_face(const Tuple& tuple) const;

    bool is_valid(const Tuple& tuple, ConstAccessor<int64_t>& hash_accessor) const override;

    void initialize(
        Eigen::Ref<const RowVectors4l> TV,
        Eigen::Ref<const RowVectors6l> TE,
        Eigen::Ref<const RowVectors4l> TF,
        Eigen::Ref<const RowVectors4l> TT,
        Eigen::Ref<const VectorXl> VT,
        Eigen::Ref<const VectorXl> ET,
        Eigen::Ref<const VectorXl> FT);
    void initialize(Eigen::Ref<const RowVectors4l> T);

    bool is_connectivity_valid() const override;

    std::vector<std::vector<TypedAttributeHandle<int64_t>>> connectivity_attributes()
        const override;

protected:
    int64_t id(const Tuple& tuple, PrimitiveType type) const override;
    int64_t id(const simplex::Simplex& simplex) const
    {
        return id(simplex.tuple(), simplex.primitive_type());
    }


    int64_t id_vertex(const Tuple& tuple) const { return id(tuple, PrimitiveType::Vertex); }
    int64_t id_edge(const Tuple& tuple) const { return id(tuple, PrimitiveType::Edge); }
    int64_t id_face(const Tuple& tuple) const { return id(tuple, PrimitiveType::Face); }
    int64_t id_tet(const Tuple& tuple) const { return id(tuple, PrimitiveType::Tetrahedron); }

    /**
     * @brief internal function that returns the tuple of requested type, and has the global index
     * cid
     *
     * @param gid
     * @return Tuple
     */
    Tuple tuple_from_id(const PrimitiveType type, const int64_t gid) const override;
    Tuple tuple_from_global_ids(int64_t tid, int64_t fid, int64_t eid, int64_t vid) const;

    // private:
protected:
    class TetMeshOperationExecutor;
    TypedAttributeHandle<int64_t> m_vt_handle;
    TypedAttributeHandle<int64_t> m_et_handle;
    TypedAttributeHandle<int64_t> m_ft_handle;

    TypedAttributeHandle<int64_t> m_tv_handle;
    TypedAttributeHandle<int64_t> m_te_handle;
    TypedAttributeHandle<int64_t> m_tf_handle;
    TypedAttributeHandle<int64_t> m_tt_handle;

    Tuple vertex_tuple_from_id(int64_t id) const;
    Tuple edge_tuple_from_id(int64_t id) const;
    Tuple face_tuple_from_id(int64_t id) const;
    Tuple tet_tuple_from_id(int64_t id) const;
};

} // namespace wmtk
