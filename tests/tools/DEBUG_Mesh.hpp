
#pragma once
#include <wmtk/Mesh.hpp>
#include "DEBUG_Mesh.hpp"
#include "DEBUG_MultiMeshManager.hpp"

namespace wmtk::tests {
class DEBUG_Mesh : public virtual Mesh
{
public:
    using Mesh::Mesh;
    bool operator==(const DEBUG_Mesh& o) const;
    bool operator!=(const DEBUG_Mesh& o) const;

    // uses spdlog to print out a variety of information about the mesh
    void print_state() const;
    DEBUG_MultiMeshManager& multi_mesh_manager()
    {
        return reinterpret_cast<DEBUG_MultiMeshManager&>(m_multi_mesh_manager);
    }
    const DEBUG_MultiMeshManager& multi_mesh_manager() const
    {
        return reinterpret_cast<const DEBUG_MultiMeshManager&>(m_multi_mesh_manager);
    }

    template <typename T>
    attribute::AccessorBase<T> create_base_accessor(const MeshAttributeHandle<T>& handle)
    {
        return attribute::AccessorBase<T>(*this, handle);
    }

    template <typename T>
    attribute::AccessorBase<T> create_const_base_accessor(
        const MeshAttributeHandle<T>& handle) const
    {
        return attribute::AccessorBase<T>(const_cast<DEBUG_Mesh&>(*this), handle);
    }
    template <typename T>
    attribute::AccessorBase<T> create_base_accessor(const MeshAttributeHandle<T>& handle) const
    {
        return create_const_base_accessor(handle);
    }

    void reserve_attributes(PrimitiveType type, int64_t size);


    using Mesh::tuple_from_id;

    Accessor<int64_t> get_cell_hash_accessor();
};

} // namespace wmtk::tests
