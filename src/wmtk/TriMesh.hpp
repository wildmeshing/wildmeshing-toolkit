#pragma once

#include "Mesh.hpp"
#include "Tuple.hpp"

#include <Eigen/Core>

namespace wmtk {
class TriMesh : public Mesh
{
private:
    MeshAttributeHandle<long> m_vf_handle;
    MeshAttributeHandle<long> m_ef_handle;

    MeshAttributeHandle<long> m_fv_handle;
    MeshAttributeHandle<long> m_fe_handle;
    MeshAttributeHandle<long> m_ff_handle;

    std::vector<Tuple> get_all_vertices() const;
    std::vector<Tuple> get_all_edges() const;
    std::vector<Tuple> get_all_faces() const;

public:
    TriMesh();

    std::vector<Tuple> get_all(const PrimitiveType& type) const override;

    void split_edge(const Tuple& t) override;
    void collapse_edge(const Tuple& t) override;

    Tuple switch_tuple(const Tuple& tuple, const PrimitiveType& type) const override;
    bool is_ccw(const Tuple& tuple) const override;
    bool is_boundary(const Tuple& tuple) const override;

    void initialize(
        Eigen::Ref<const RowVectors3l> FV,
        Eigen::Ref<const RowVectors3l> FE,
        Eigen::Ref<const RowVectors3l> FF,
        Eigen::Ref<const VectorXl> VF,
        Eigen::Ref<const VectorXl> EF);
    void initialize(Eigen::Ref<const RowVectors3l> F);

    long _debug_id(const Tuple& tuple, const PrimitiveType& type) const
    {
#ifndef WMTK_USE_DEBUG_FUNCTIONS
        throw "Function can only be used for debugging!";
#endif // !WMTK_USE_DEBUG_FUNCTIONS

        return id(tuple, type);
    }

protected:
    long id(const Tuple& tuple, const PrimitiveType& type) const override;
};

} // namespace wmtk
