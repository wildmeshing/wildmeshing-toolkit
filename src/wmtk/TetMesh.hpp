#pragma once

#include "Mesh.hpp"

namespace wmtk {
class TetMesh : public Mesh
{
private:
    MeshAttributeHandle<long> m_vt_handle;
    MeshAttributeHandle<long> m_et_handle;
    MeshAttributeHandle<long> m_ft_handle;

    MeshAttributeHandle<long> m_tv_handle;
    MeshAttributeHandle<long> m_te_handle;
    MeshAttributeHandle<long> m_tf_handle;
    MeshAttributeHandle<long> m_tt_handle;

    Tuple vertex_tuple_from_id() const;
    Tuple edge_tuple_from_id(long id) const;
    Tuple face_tuple_from_id(long id) const;
    Tuple tuple_from_id(PrimitiveType ptype, long id) const override;

public:
    TetMesh();

    std::vector<Tuple> get_all(const PrimitiveType& type) const override;

    void split_edge(const Tuple& t) override;
    void collapse_edge(const Tuple& t) override;

    Tuple switch_tuple(const Tuple& tuple, const PrimitiveType& type) const override;
    bool is_ccw(const Tuple& tuple) const override;
    bool is_boundary(const Tuple& tuple) const override;
    bool is_valid(const Tuple& tuple) const override;

    void initialize(
        Eigen::Ref<const RowVectors4l> TV,
        Eigen::Ref<const RowVectors6l> TE,
        Eigen::Ref<const RowVectors4l> TF,
        Eigen::Ref<const RowVectors4l> TT,
        Eigen::Ref<const VectorXl> VT,
        Eigen::Ref<const VectorXl> ET,
        Eigen::Ref<const VectorXl> FT);
    void initialize(Eigen::Ref<const RowVectors4l> T);

protected:
    long id(const Tuple& tuple, const PrimitiveType& type) const override;
};

} // namespace wmtk