
#pragma once
#include <wmtk/Mesh.hpp>
#include "DEBUG_Mesh.hpp"
#include "DEBUG_MultiMeshManager.hpp"

namespace wmtk::tests {
class DEBUG_Mesh : public Mesh
{
public:
    using Mesh::Mesh;
    bool operator==(const DEBUG_Mesh& o) const;
    bool operator!=(const DEBUG_Mesh& o) const;

    // uses spdlog to print out a variety of information about the mesh
    void print_state() const;
    static DEBUG_MultiMeshManager& multi_mesh_manager(Mesh& m)
    {
        return reinterpret_cast<DEBUG_MultiMeshManager&>(m.m_multi_mesh_manager);
    }
    static const DEBUG_MultiMeshManager& multi_mesh_manager(const Mesh& m)
    {
        return reinterpret_cast<const DEBUG_MultiMeshManager&>(m.m_multi_mesh_manager);
    }
    static wmtk::attribute::AttributeManager& attribute_manager(Mesh& m)
    {
        return m.m_attribute_manager;
    }
    static const wmtk::attribute::AttributeManager& attribute_manager(const Mesh& m)
    {
        return m.m_attribute_manager;
    }
    DEBUG_MultiMeshManager& multi_mesh_manager() { return multi_mesh_manager(*this); }
    const DEBUG_MultiMeshManager& multi_mesh_manager() const { return multi_mesh_manager(*this); }

    void reserve_attributes(PrimitiveType type, int64_t size);


    using Mesh::id;
    using Mesh::tuple_from_id;
};

} // namespace wmtk::tests
