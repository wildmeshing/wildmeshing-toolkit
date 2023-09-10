#include "MultiMeshManager.hpp"
#include "Mesh.hpp"
#include "Types.hpp"
#include "SimplicialComplex.hpp"
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

    bool MultiMeshManager::is_parent_mesh() const
    {
        return m_is_parent_mesh;
    }

    long MultiMeshManager::child_id() const
    {
        return m_child_id;
    }

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
        auto map = map_accessor.const_vector_attribute(source_tuple);

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
            write_tuple_map_attribute(map_to_child_handle, parent_mesh, parent_mesh.tuple_from_id(map_ptype, id), Tuple());
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

    void MultiMeshManager::register_child_mesh(Mesh& parent_mesh, std::shared_ptr<Mesh> child_mesh, const std::vector<long>& child_mesh_simplex_id_map)
    {
        PrimitiveType map_type = child_mesh->top_simplex_type();        
        std::vector<std::array<Tuple,2>> child_mesh_simplex_map;
        
        for (long child_cell_id = 0; child_cell_id < long(child_mesh_simplex_id_map.size()); ++child_cell_id)
        {
            long parent_cell_id = child_mesh_simplex_id_map[child_cell_id];
            child_mesh_simplex_map.push_back({child_mesh->tuple_from_id(map_type, child_cell_id), parent_mesh.tuple_from_id(map_type, parent_cell_id)});
        }
        register_child_mesh(parent_mesh, child_mesh, child_mesh_simplex_map);
    }

    Tuple MultiMeshManager::map_tuple_between_meshes(const Mesh& source_mesh, const Mesh& target_mesh, MeshAttributeHandle<long> source_map_handle, const Tuple& source_tuple)
    {
        PrimitiveType source_mesh_ptype = source_mesh.top_simplex_type();
        PrimitiveType target_mesh_ptype = target_mesh.top_simplex_type();

        auto [source_mesh_base_tuple, target_mesh_base_tuple] = read_tuple_map_attribute(source_map_handle, source_mesh, source_tuple);

        if (source_mesh_base_tuple.is_null() || target_mesh_base_tuple.is_null())
        {
            return Tuple(); // return null tuple
        }

        if (source_mesh_ptype == PrimitiveType::Face && target_mesh_ptype == PrimitiveType::Face)
        {
            Tuple cur_tuple = source_tuple;
            std::vector<PrimitiveType> record_switch_operations;
            while (cur_tuple != source_mesh_base_tuple)
            {
                cur_tuple = source_mesh.switch_tuple(cur_tuple, PrimitiveType::Vertex);
                record_switch_operations.push_back(PrimitiveType::Vertex);
                if (cur_tuple == source_mesh_base_tuple)
                {
                    break;
                }
                cur_tuple = source_mesh.switch_tuple(cur_tuple, PrimitiveType::Edge);
                record_switch_operations.push_back(PrimitiveType::Edge);
            }

            Tuple ret_tuple = target_mesh_base_tuple;
            for (int i = record_switch_operations.size() - 1; i >= 0; --i)
            {
                    ret_tuple = target_mesh.switch_tuple(ret_tuple, record_switch_operations[i]);
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

    std::vector<Simplex> MultiMeshManager::find_all_simplices_in_child_mesh(const Mesh& parent_mesh, long child_id, const Simplex& simplex_parent)
    {
        auto child_mesh_ptr = parent_mesh.multi_mesh_manager.child_meshes[child_id];
        auto map_to_child_handle = parent_mesh.multi_mesh_manager.map_to_child_handles[child_id];
        PrimitiveType simplex_ptype = simplex_parent.primitive_type();
        PrimitiveType childmesh_ptype = child_mesh_ptr->top_simplex_type();

        if (simplex_ptype > childmesh_ptype)
        {
            // Can't find higher-dimensional simplex in child_mesh
            return std::vector<Simplex>();
        }

        // Find all dim(child_mesh) simplex in open_star(simplex_parent)) in parent_mesh
        auto top_simplex_in_open_star = SimplicialComplex::open_star(parent_mesh, simplex_parent).get_simplices(childmesh_ptype);

        // map tuples to child_mesh and collect all distinct simplices
        SimplicialComplex ret_sc(*child_mesh_ptr);
        for (auto s : top_simplex_in_open_star)
        {
            auto child_tuple = map_tuple_between_meshes(parent_mesh, *child_mesh_ptr, map_to_child_handle, s.tuple());
            if (!child_tuple.is_null())
            {
                ret_sc.add_simplex(Simplex(simplex_ptype, child_tuple));
            }
        }
        
        return ret_sc.get_simplex_vector();
    }

    bool MultiMeshManager::is_child_mesh_valid(const Mesh& parent_mesh, const Mesh& child_mesh)
    {
        // TODO: implement this
        
        return true;
    }

    std::vector<Tuple> MultiMeshManager::map_edge_tuple_to_all_children(const Mesh& parent_mesh, const Tuple& edge_tuple)
    {
        std::vector<Tuple> ret;
        for (auto child_mesh_ptr : parent_mesh.multi_mesh_manager.child_meshes)
        {
            long child_id = child_mesh_ptr->multi_mesh_manager.child_id();
            auto map_to_child_handle = parent_mesh.multi_mesh_manager.map_to_child_handles[child_id];
            Tuple child_tuple = map_tuple_between_meshes(parent_mesh, *child_mesh_ptr, map_to_child_handle, edge_tuple);
            ret.push_back(child_tuple);
        }
        return ret;
    }

    bool MultiMeshManager::is_map_valid(const Mesh& parent_mesh) const
    {
        for (auto child_mesh_ptr : child_meshes)
        {
            long child_id = child_mesh_ptr->multi_mesh_manager.child_id();
            PrimitiveType map_type = child_mesh_ptr->top_simplex_type();

            auto parent_to_child_handle = map_to_child_handles[child_id];
            auto child_to_parent_handle = child_mesh_ptr->multi_mesh_manager.map_to_parent_handle;
            auto child_cell_flag_accessor = child_mesh_ptr->get_flag_accessor(map_type);

            for (long id = 0; id < child_mesh_ptr->capacity(map_type); ++id)
            {
                if (child_cell_flag_accessor.scalar_attribute(child_mesh_ptr->tuple_from_id(map_type, id)) == 0)
                {
                    continue;
                }

                // 1. test if all maps in child_mesh exisits
                // 2. test if tuples in maps are valid (and up_to_date)
                // 3. test if map is symmetric
                // 4. test switch_top_simplex operation

                auto [child_tuple, parent_tuple] = read_tuple_map_attribute(child_to_parent_handle, *child_mesh_ptr, child_mesh_ptr->tuple_from_id(map_type, id));

                if (!child_mesh_ptr->is_valid_slow(child_tuple))
                {
                    return false;
                }
                if (!parent_mesh.is_valid_slow(parent_tuple))
                {
                    return false;
                }
                
                auto [parent_tuple_test, child_tuple_test] = read_tuple_map_attribute(parent_to_child_handle, parent_mesh, parent_tuple);

                if (child_tuple_test != child_tuple || parent_tuple_test != parent_tuple)
                {
                    return false;
                }

                // for 4, current code support only mapping between triangle meshes
                if (map_type == PrimitiveType::Face && parent_mesh.top_simplex_type() == PrimitiveType::Face)
                {
                    Tuple cur_child_tuple = child_tuple;
                    Tuple cur_parent_tuple = parent_tuple;

                    for (int i = 0; i < 3; i++)
                    {
                        if (!child_mesh_ptr->is_boundary(cur_child_tuple))
                        {
                            if (parent_mesh.is_boundary(cur_parent_tuple))
                            {
                                return false;
                            }

                            Tuple child_tuple_opp = child_mesh_ptr->switch_face(cur_child_tuple);
                            Tuple parent_tuple_opp = parent_mesh.switch_face(cur_parent_tuple);

                            if (parent_tuple_opp != map_tuple_between_meshes(*child_mesh_ptr, parent_mesh, child_to_parent_handle, child_tuple_opp))
                            {
                                return false;
                            }

                        }
                        cur_child_tuple = child_mesh_ptr->switch_edge(child_mesh_ptr->switch_vertex(cur_child_tuple));
                        cur_parent_tuple = parent_mesh.switch_edge(parent_mesh.switch_vertex(cur_parent_tuple));
                    }
                }
                else
                {
                    // TODO: implement other cases
                    continue;
                }
                
            }
            
        }
        return true;
    }
}
