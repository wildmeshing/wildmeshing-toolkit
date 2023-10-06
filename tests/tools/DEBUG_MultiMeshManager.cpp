
bool DEBUG_MultiMeshManager::is_child_mesh_valid(const Mesh& my_mesh, const Mesh& child_mesh) const
{
    assert(&my_mesh.m_multi_mesh_manager == this);
    // TODO: implement this

    return true;
}

std::vector<Tuple> DEBUG_MultiMeshManager::map_edge_tuple_to_all_children(
    const Mesh& my_mesh,
    const Tuple& edge_tuple)
{
    assert(&my_mesh.m_multi_mesh_manager == this);
    std::vector<Tuple> ret;
    for (const auto& child_data : m_children) {
        const Mesh& child_mesh = *child_data.mesh;
        const auto map_to_child_handle = child_data.map_handle;
        Tuple child_tuple =
            map_tuple_between_meshes(my_mesh, child_mesh, map_to_child_handle, edge_tuple);
        ret.push_back(child_tuple);
    }
    return ret;
}

bool DEBUG_MultiMeshManager::is_map_valid(const Mesh& my_mesh) const
{
    assert(&my_mesh.m_multi_mesh_manager == this);
    for (size_t index = 0; index < m_children.size(); ++index) {
        const auto& child_data = m_children[index];
        if (child_data.mesh->mesh_manager.m_child_id != index) {
            return false;
        }
        if (!is_child_map_valid(my_mesh, child_data)) {
            return false;
        }
    }
    return true;
}
bool DEBUG_MultiMeshManager::is_child_map_valid(const Mesh& my_mesh, const ChildData& child_data)
    const
{
    assert(&my_mesh.m_multi_mesh_manager == this);
    const Mesh& child_mesh = *child_data.mesh;
    const auto parent_to_child_handle = child_data.map_handle;
    PrimitiveType map_type = child_mesh.top_simplex_type();

    auto child_to_parent_handle = child_mesh.m_multi_mesh_manager.map_to_parent_handle;
    auto child_cell_flag_accessor = child_mesh.get_flag_accessor(map_type);

    for (const child_tuple : child_mesh.get_simplices(map_type)) {
        // 1. test if all maps in child_mesh exisits
        auto [child_tuple_from_child, parent_tuple_from_child] =
            read_tuple_map_attribute(child_to_parent_handle, child_mesh, child_tuple);

        // 2. test if tuples in maps are valid (and up_to_date)
        {
            if (!child_mesh.is_valid_slow(child_tuple_from_child)) {
                return false;
            }
            if (!my_mesh.is_valid_slow(parent_tuple_from_child)) {
                return false;
            }
        }

        // 3. test if map is symmetric
        {
            auto [parent_tuple_from_parent, child_tuple_from_parent] =
                read_tuple_map_attribute(parent_to_child_handle, my_mesh, parent_tuple);

            if (child_tuple_from_child != child_tuple_from_parent ||
                parent_tuple_from_child != parent_tuple_from_parent) {
                return false;
            }
        }

        // 4. test switch_top_simplex operation
        // for 4, current code support only mapping between triangle meshes
        if (map_type == PrimitiveType::Face && my_mesh.top_simplex_type() == PrimitiveType::Face) {
            Tuple cur_child_tuple = child_tuple;
            Tuple cur_parent_tuple = parent_tuple;

            for (int i = 0; i < 3; i++) {
                if (!child_mesh_ptr->is_boundary(cur_child_tuple)) {
                    if (my_mesh.is_boundary(cur_parent_tuple)) {
                        return false;
                    }

                    Tuple child_tuple_opp = child_mesh.switch_face(cur_child_tuple);
                    Tuple parent_tuple_opp = my_mesh.switch_face(cur_parent_tuple);

                    if (parent_tuple_opp != map_tuple_between_meshes(
                                                *child_mesh_ptr,
                                                my_mesh,
                                                child_to_parent_handle,
                                                child_tuple_opp)) {
                        return false;
                    }
                }
                cur_child_tuple =
                    child_mesh_ptr->switch_edge(child_mesh_ptr->switch_vertex(cur_child_tuple));
                cur_parent_tuple = my_mesh.switch_edge(my_mesh.switch_vertex(cur_parent_tuple));
            }
        } else {
            // TODO: implement other cases
            continue;
        }
    }
    return true;
}
