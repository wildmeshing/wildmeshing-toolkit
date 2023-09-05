#pragma once

#include "Mesh.hpp"

namespace wmtk {
class TetMesh : public Mesh
{
public:
    TetMesh();

    Tuple split_edge(const Tuple& t, Accessor<long>& hash_accessor) override;
    Tuple collapse_edge(const Tuple& t, Accessor<long>& hash_accessor) override;
    PrimitiveType top_simplex_type() const override { return PrimitiveType::Tetrahedron; }
    Tuple switch_tuple(const Tuple& tuple, PrimitiveType type) const override;
    bool is_ccw(const Tuple& tuple) const override;
    bool is_boundary(const Tuple& tuple) const override;
    bool is_boundary_vertex(const Tuple& tuple) const override;
    bool is_boundary_edge(const Tuple& tuple) const override;

    bool is_valid(const Tuple& tuple, ConstAccessor<long>& hash_accessor) const override;

    void initialize(
        Eigen::Ref<const RowVectors4l> TV,
        Eigen::Ref<const RowVectors6l> TE,
        Eigen::Ref<const RowVectors4l> TF,
        Eigen::Ref<const RowVectors4l> TT,
        Eigen::Ref<const VectorXl> VT,
        Eigen::Ref<const VectorXl> ET,
        Eigen::Ref<const VectorXl> FT);
    void initialize(Eigen::Ref<const RowVectors4l> T);

    long _debug_id(const Tuple& tuple, PrimitiveType type) const;

    bool is_connectivity_valid() const override;

protected:
    long id(const Tuple& tuple, PrimitiveType type) const override;

    /**
     * @brief internal function that returns the tuple of requested type, and has the global index
     * cid
     *
     * @param gid
     * @return Tuple
     */
    Tuple tuple_from_id(const PrimitiveType type, const long gid) const override;

private:
    MeshAttributeHandle<long> m_vt_handle;
    MeshAttributeHandle<long> m_et_handle;
    MeshAttributeHandle<long> m_ft_handle;

    MeshAttributeHandle<long> m_tv_handle;
    MeshAttributeHandle<long> m_te_handle;
    MeshAttributeHandle<long> m_tf_handle;
    MeshAttributeHandle<long> m_tt_handle;

    Tuple vertex_tuple_from_id(long id) const;
    Tuple edge_tuple_from_id(long id) const;
    Tuple face_tuple_from_id(long id) const;
    Tuple tet_tuple_from_id(long id) const;
};

} // namespace wmtk
