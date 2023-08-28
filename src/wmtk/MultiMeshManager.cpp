#include "MultiMeshManager.hpp"
#include "Mesh.hpp"
namespace wmtk
{
    MultiMeshManager::MultiMeshManager()
        : m_is_parent_mesh(false)
        , m_child_id(-1)
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

    void MultiMeshManager::register_child_mesh(Mesh& parent_mesh, std::shared_ptr<Mesh> child_mesh, const std::vector<std::array<Tuple,2>>& child_mesh_simplex_map)
    {
        PrimitiveType map_ptype = child_mesh->top_simplex_type();
        long cur_child_id = long(parent_mesh.multi_mesh_manager.child_meshes.size());

        auto map_to_parent_handle = child_mesh->register_attribute<long>("map_to_parent", map_ptype, 10);
        auto map_to_child_handle = parent_mesh.register_attribute<long>(fmt::format("map_to_child_{}", cur_child_id), map_ptype, 10);
        for (long id = 0; id < parent_mesh.capacity(map_ptype); ++id)
        {
            write_tuple_map_attribute(map_to_child_handle, parent_mesh, Tuple(), Tuple());
        }

        // register maps
        for (long id = 0; id < child_mesh->capacity(map_ptype); ++id)
        {
            write_tuple_map_attribute(map_to_parent_handle, *child_mesh, child_mesh_simplex_map[id][0], child_mesh_simplex_map[id][1]);

            write_tuple_map_attribute(map_to_child_handle, parent_mesh, child_mesh_simplex_map[id][1], child_mesh_simplex_map[id][0]);
        }
        
        // update on child_mesh
        child_mesh->multi_mesh_manager.map_to_parent_handle = map_to_parent_handle;
        child_mesh->multi_mesh_manager.m_is_parent_mesh = false;
        child_mesh->multi_mesh_manager.m_child_id = long(parent_mesh.multi_mesh_manager.child_meshes.size());

        // update on parent_mesh
        parent_mesh.multi_mesh_manager.child_meshes.push_back(child_mesh);
        parent_mesh.multi_mesh_manager.map_to_child_handles.push_back(map_to_child_handle);
        parent_mesh.multi_mesh_manager.m_is_parent_mesh = true;
       
    }

    Tuple MultiMeshManager::map_tuple_between_meshes(const Mesh& source_mesh, const Mesh& target_mesh, MeshAttributeHandle<long> source_map_handle, const Tuple& source_tuple)
    {
        PrimitiveType source_mesh_ptype = source_mesh.top_simplex_type();
        PrimitiveType target_mesh_ptype = target_mesh.top_simplex_type();

        auto [source_mesh_base_tuple, target_mesh_base_tuple] = read_tuple_map_attribute(source_map_handle, source_mesh, source_tuple);

        if (source_mesh_base_tuple.is_null_tuple())
        {
            return Tuple(); // return null tuple
        }
        
        if (source_mesh_ptype == PrimitiveType::Face && target_mesh_ptype == PrimitiveType::Face)
        {
            Tuple cur_tuple = source_tuple;
            std::vector<char> record_switch_operations;
            while (cur_tuple != source_mesh_base_tuple)
            {
                cur_tuple = source_mesh.switch_tuple(cur_tuple, PrimitiveType::Vertex);
                record_switch_operations.push_back('v');
                if (cur_tuple == source_mesh_base_tuple)
                {
                    break;
                }
                cur_tuple = source_mesh.switch_tuple(cur_tuple, PrimitiveType::Edge);
                record_switch_operations.push_back('e');
            }

            Tuple ret_tuple = target_mesh_base_tuple;
            for (int i = record_switch_operations.size() - 1; i >= 0; --i)
            {
                if (record_switch_operations[i] == 'v')
                {
                    ret_tuple = target_mesh.switch_tuple(ret_tuple, PrimitiveType::Vertex);
                }
                else // record_switch_operations[i] == 'e'
                {
                    ret_tuple = target_mesh.switch_tuple(ret_tuple, PrimitiveType::Edge);
                }
            }

            return ret_tuple;
        }
        else
        {
            // TODO: implement this for other cases later
            // TODO: maybe use a seperate function for each case?
            return Tuple();
        }
    }
    bool MultiMeshManager::is_child_mesh_valid(const Mesh& parent_mesh)
    {
        // TODO: 
        return true;
    }
}