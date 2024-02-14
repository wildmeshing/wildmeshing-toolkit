#pragma once
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/EdgeMeshOperationExecutor.hpp>
#include "DEBUG_MultiMeshManager.hpp"

namespace wmtk::tests {
class DEBUG_EdgeMesh : public EdgeMesh
{
public:
    using EdgeMesh::EdgeMesh;
    DEBUG_EdgeMesh(EdgeMesh&& m);
    using EdgeMesh::operator=;


    bool operator==(const DEBUG_EdgeMesh& o) const;
    bool operator!=(const DEBUG_EdgeMesh& o) const;

    // uses spdlog to print out a variety of information about the mesh
    void print_state() const;

    void print_ve() const;
    Eigen::Matrix<int64_t, 2, 1> ev_from_eid(const int64_t eid) const;

    auto edge_tuple_from_vids(const int64_t v1, const int64_t v2) const -> Tuple;
    auto tuple_from_edge_id(const int64_t eid) const -> Tuple;

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

    const TypedAttributeHandle<int64_t>& e_handle(const PrimitiveType type) const;

    const TypedAttributeHandle<int64_t>& ve_handle() const;

    const TypedAttributeHandle<int64_t>& ev_handle() const;


    void reserve_attributes(PrimitiveType type, int64_t size);


    int64_t id(const Tuple& tuple, PrimitiveType type) const;
    int64_t id(const simplex::Simplex& s) const;
    /**
     * @brief returns the TriMeshOperationExecutor
     */
    using EdgeMesh::tuple_from_id;

    attribute::Accessor<int64_t> get_cell_hash_accessor();

    EdgeMeshOperationExecutor get_emoe(const Tuple& t, attribute::Accessor<int64_t>& hash_accessor);

    bool is_simplex_deleted(PrimitiveType type, const int64_t id) const;
};

} // namespace wmtk::tests
