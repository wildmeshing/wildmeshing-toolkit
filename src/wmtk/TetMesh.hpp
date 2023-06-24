#pragma once
#include "Mesh.hpp"
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
    std::vector<Tuple> get_vertices() const;
    std::vector<Tuple> get_edges() const;
    std::vector<Tuple> get_faces() const;
    std::vector<Tuple> get_tetrahedrons() const;

public:
    TetMesh();

    long id(const Tuple& tuple, const PrimitiveType& type) const override;
    Tuple switch_tuple(const Tuple& tuple, const PrimitiveType& type) const override;
    bool is_ccw(const Tuple& tuple) const override;
    bool is_boundary(const Tuple& tuple) const override;
    void initialize(
        Eigen::Ref<const RowVectors4l>& TV,
        Eigen::Ref<const RowVectors4l>& TE,
        Eigen::Ref<const RowVectors4l>& TF,
        Eigen::Ref<const RowVectors4l>& TT,
        Eigen::Ref<const VectorXl>& VT,
        Eigen::Ref<const VectorXl>& ET,
        Eigen::Ref<const VectorXl>& FT) const;
};
// TODO: this tetmesh_topology_initialization should belong in a detail folder or something, not
// part of the tetmesh class
void tetmesh_topology_initialization(
    Eigen::Ref<const Mesh::RowVectors3d> V,
    Eigen::Ref<const Mesh::RowVectors4l> F,
    TetMesh& mesh);
