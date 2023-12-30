#pragma once
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/EdgeMeshOperationExecutor.hpp>
#include "DEBUG_MultiMeshManager.hpp"

namespace wmtk::tests {
class DEBUG_EdgeMesh : public EdgeMesh
{
public:
    using EdgeMesh::EdgeMesh;
    DEBUG_EdgeMesh(const EdgeMesh& m);
    DEBUG_EdgeMesh(EdgeMesh&& m);
    using EdgeMesh::operator=;


    bool operator==(const DEBUG_EdgeMesh& o) const;
    bool operator!=(const DEBUG_EdgeMesh& o) const;

    // uses spdlog to print out a variety of information about the mesh
    void print_state() const;

    void print_ve() const;
    Eigen::Matrix<long, 2, 1> ev_from_eid(const long eid) const;

    auto edge_tuple_from_vids(const long v1, const long v2) const -> Tuple;
    auto tuple_from_edge_id(const long eid) const -> Tuple;

    DEBUG_MultiMeshManager& multi_mesh_manager()
    {
        return reinterpret_cast<DEBUG_MultiMeshManager&>(m_multi_mesh_manager);
    }

    template <typename T>
    attribute::AccessorBase<T> create_base_accessor(const TypedAttributeHandle<T>& handle)
    {
        return attribute::AccessorBase<T>(*this, handle);
    }

    template <typename T>
    attribute::AccessorBase<T> create_const_base_accessor(
        const TypedAttributeHandle<T>& handle) const
    {
        return attribute::AccessorBase<T>(const_cast<DEBUG_EdgeMesh&>(*this), handle);
    }
    template <typename T>
    attribute::AccessorBase<T> create_base_accessor(const TypedAttributeHandle<T>& handle) const
    {
        return create_const_base_accessor(handle);
    }

    const TypedAttributeHandle<long>& e_handle(const PrimitiveType type) const;

    const TypedAttributeHandle<long>& ve_handle() const;

    const TypedAttributeHandle<long>& ev_handle() const;


    void reserve_attributes(PrimitiveType type, long size);


    long id(const Tuple& tuple, PrimitiveType type) const override;
    long id(const simplex::Simplex& s) const;
    /**
     * @brief returns the TriMeshOperationExecutor
     */
    using EdgeMesh::tuple_from_id;

    Accessor<long> get_cell_hash_accessor();

    EdgeMeshOperationExecutor get_emoe(const Tuple& t, Accessor<long>& hash_accessor);

    bool is_simplex_deleted(PrimitiveType type, const long id) const;
};

} // namespace wmtk::tests
