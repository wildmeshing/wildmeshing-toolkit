#include "MultiMeshManager.hpp"
#include "Mesh.hpp"
namespace wmtk
{
    MultiMeshManager::MultiMeshManager()
        : is_parent_mesh(false)
        , child_id(-1)
    {
    }

    MultiMeshManager::~MultiMeshManager() = default;
    MultiMeshManager::MultiMeshManager(const MultiMeshManager& o) = default;
    MultiMeshManager::MultiMeshManager(MultiMeshManager&& o) = default;
    MultiMeshManager& MultiMeshManager::operator=(const MultiMeshManager& o) = default;
    MultiMeshManager& MultiMeshManager::operator=(MultiMeshManager&& o) = default;

    void MultiMeshManager::write_tuple_map_attribute(MeshAttributeHandle<long> map_handle, Mesh& source_mesh, const Tuple& source_tuple, const Tuple& target_tuple)
    {
        auto map_accessor = source_mesh.create_accessor(map_handle);
        auto map = map_accessor.vector_attribute(source_tuple);

        map(0) = source_tuple.m_local_vid;
        map(1) = source_tuple.m_local_eid;
        map(2) = source_tuple.m_local_fid;
        map(3) = source_tuple.m_global_cid;
        map(4) = source_tuple.m_hash;

        map(5) = target_tuple.m_local_vid;
        map(6) = target_tuple.m_local_eid;
        map(7) = target_tuple.m_local_fid;
        map(8) = target_tuple.m_global_cid;
        map(9) = target_tuple.m_hash; 
    }
    
    std::tuple<Tuple, Tuple> MultiMeshManager::read_tuple_map_attribute(MeshAttributeHandle<long> map_handle, const Mesh& source_mesh, const Tuple& source_tuple)
    {
        auto map_accessor = source_mesh.create_accessor(map_handle);
        auto map = map_accessor.vector_attribute(source_tuple);

        return std::make_tuple(Tuple(map(0), map(1), map(2), map(3), map(4)), Tuple(map(5), map(6), map(7), map(8), map(9)));
    }
    void MultiMeshManager::register_child_mesh(Mesh&parent_mesh, std::shared_ptr<Mesh> child_mesh, const std::vector<std::array<Tuple,2>>&child_mesh_simplex_map)
    {
        // TODO: implement this
        child_mesh->register_attribute<long>("map_to_parent", PrimitiveType::Face, 10);
        // register child_mesh
        child_meshes.push_back(child_mesh);
        // register map_to_child

        // register map_to_parent

        // set map_to_child

        // set map_to_parent
       
        // update child_id
    }

    bool MultiMeshManager::is_child_mesh_valid(const Mesh& parent_mesh) const
    {
        // TODO: 
        return true;
    }
}