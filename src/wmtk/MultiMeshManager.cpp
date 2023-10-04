#include "MultiMeshManager.hpp"
#include <wmtk/simplex/top_level_cofaces.hpp>
#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>
#include "Mesh.hpp"
#include "SimplicialComplex.hpp"
#include "multimesh/utils/transport_tuple.hpp"
#include "multimesh/utils/tuple_map_attribute_io.hpp"
namespace wmtk {

namespace {} // namespace

Tuple MultiMeshManager::map_tuple_between_meshes(
    const Mesh& source_mesh,
    const Mesh& target_mesh,
    const ConstAccessor<long>& map_accessor,
    const Tuple& source_tuple)
{
    PrimitiveType source_mesh_primitive_type = source_mesh.top_simplex_type();
    PrimitiveType target_mesh_primitive_type = target_mesh.top_simplex_type();
    PrimitiveType min_primitive_type =
        std::min(source_mesh_primitive_type, target_mesh_primitive_type);

    auto [source_mesh_base_tuple, target_mesh_base_tuple] =
        multimesh::utils::read_tuple_map_attribute(map_accessor, source_tuple);

    if (source_mesh_base_tuple.is_null() || target_mesh_base_tuple.is_null()) {
        return Tuple(); // return null tuple
    }

    // we want to repeat switches from source_base_tuple -> source_tuple to
    // target_base _tuple -> return value
    //
    return multimesh::utils::transport_tuple(
        source_mesh_base_tuple,
        source_tuple,
        source_mesh_primitive_type,
        target_mesh_base_tuple,
        target_mesh_primitive_type);
}


MultiMeshManager::MultiMeshManager() = default;

MultiMeshManager::~MultiMeshManager() = default;
MultiMeshManager::MultiMeshManager(const MultiMeshManager& o) = default;
MultiMeshManager::MultiMeshManager(MultiMeshManager&& o) = default;
MultiMeshManager& MultiMeshManager::operator=(const MultiMeshManager& o) = default;
MultiMeshManager& MultiMeshManager::operator=(MultiMeshManager&& o) = default;

bool MultiMeshManager::is_root() const
{
    return m_parent == nullptr;
}

long MultiMeshManager::child_id() const
{
    return m_child_id;
}

std::vector<long> MultiMeshManager::absolute_id() const
{
    if (is_root()) {
        return {};
    } else {
        auto id = m_parent->m_multi_mesh_manager.absolute_id();
        id.emplace_back(m_child_id);
        return id;
    }
}


void MultiMeshManager::register_child_mesh(
    Mesh& my_mesh,
    const std::shared_ptr<Mesh>& child_mesh_ptr,
    const std::vector<std::array<Tuple, 2>>& child_mesh_simplex_map)
{
    assert(&my_mesh.m_multi_mesh_manager == this);
    assert(bool(child_mesh_ptr));

    Mesh& child_mesh = *child_mesh_ptr;

    PrimitiveType child_primitive_type = child_mesh.top_simplex_type();
    long new_child_id = long(m_children.size());

    auto child_to_parent_handle =
        child_mesh.register_attribute<long>("map_to_parent", child_primitive_type, 10);

    // TODO: make sure that this attribute doesnt already exist
    auto parent_to_child_handle = my_mesh.register_attribute<long>(
        fmt::format("map_to_child_{}", new_child_id),
        child_primitive_type,
        10);


    auto child_to_parent_accessor = child_mesh.create_accessor(child_to_parent_handle);
    auto parent_to_child_accessor = my_mesh.create_accessor(parent_to_child_handle);

    // default initialize the parent to child map which can have entries missing
    for (long id = 0; id < my_mesh.capacity(child_primitive_type); ++id) {
        multimesh::utils::write_tuple_map_attribute(parent_to_child_accessor, Tuple(), Tuple());
    }
    // register maps
    for (const auto& [my_tuple, child_tuple] : child_mesh_simplex_map) {
        multimesh::utils::write_tuple_map_attribute(
            child_to_parent_accessor,
            my_tuple,
            child_tuple);
        multimesh::utils::write_tuple_map_attribute(
            parent_to_child_accessor,
            child_tuple,
            my_tuple);
    }

    MultiMeshManager& child_manager = child_mesh.m_multi_mesh_manager;

    // update on child_mesh
    child_manager.map_to_parent_handle = child_to_parent_handle;
    child_manager.m_child_id = new_child_id;
    child_manager.m_parent = &my_mesh;

    // update myself
    m_children.emplace_back(ChildData{child_mesh_ptr, parent_to_child_handle});
}

/*
 * TODO: It is the consumer's responsibility to generate teh identity map via a utility function
void MultiMeshManager::register_child_mesh(
    Mesh& my_mesh,
    std::shared_ptr<Mesh> child_mesh,
    const std::vector<long>& child_mesh_simplex_id_map)
{
    PrimitiveType map_type = child_mesh->top_simplex_type();
    std::vector<std::array<Tuple, 2>> child_mesh_simplex_map;

    for (long child_cell_id = 0; child_cell_id < long(child_mesh_simplex_id_map.size());
         ++child_cell_id) {
        long parent_cell_id = child_mesh_simplex_id_map[child_cell_id];
        child_mesh_simplex_map.push_back(
            {child_mesh->tuple_from_id(map_type, child_cell_id),
             my_mesh.tuple_from_id(map_type, parent_cell_id)});
    }
    register_child_mesh(my_mesh, child_mesh, child_mesh_simplex_map);
}
*/

std::vector<Simplex>
MultiMeshManager::map(const Mesh& my_mesh, const Mesh& other_mesh, const Simplex& my_simplex) const
{
    const auto ret_tups = map_to_tuples(my_mesh, other_mesh, my_simplex);
    return simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
        ret_tups,
        my_simplex.primitive_type());
}
std::vector<Tuple> MultiMeshManager::map_tuples(
    const Mesh& my_mesh,
    const Mesh& other_mesh,
    const Simplex& my_simplex) const
{
    throw "Not implemented";
    assert(&my_mesh.m_multi_mesh_manager == this);
    // TODO: construct relative positions
    std::vector<Tuple> equivalent_tuples = simplex::top_level_cofaces_tuples(my_mesh, my_simplex);
    // TODO: construct visitor class that maps up and down
    // MultiMeshMapVisitor visitor(my_mesh, other_mesh);
    // TODO: visitor runs along meshes traversing the path

    // visitor.map(equivalent_tuples, my_simplex.primitive_type());

    return {};
}


Simplex MultiMeshManager::map_to_parent(const Mesh& my_mesh, const Simplex& my_simplex) const
{
    return Simplex(
        my_simplex.primitive_type(),
        map_tuple_to_parent_tuple(my_mesh, my_simplex.tuple()));
}

Tuple MultiMeshManager::map_tuple_to_parent_tuple(const Mesh& my_mesh, const Tuple& my_tuple) const
{
    assert(&my_mesh.m_multi_mesh_manager == this);
    assert(!is_root());

    const Mesh& parent_mesh = *m_parent;

    const auto& map_handle = map_to_parent_handle;
    // assert(!map_handle.is_null());

    auto map_accessor = my_mesh.create_accessor(map_handle);
    return map_tuple_between_meshes(my_mesh, parent_mesh, map_accessor, my_tuple);
}
std::vector<Tuple> MultiMeshManager::map_to_child_tuples(
    const Mesh& my_mesh,
    const ChildData& child_data,
    const Simplex& my_simplex) const
{
    assert(&my_mesh.m_multi_mesh_manager == this);

    const Mesh& child_mesh = *child_data.mesh;
    const auto map_handle = child_data.map_handle;
    // we will rewrite these tuples inline with the mapped ones
    std::vector<Tuple> tuples = simplex::top_level_cofaces_tuples(my_mesh, my_simplex);

    auto map_accessor = my_mesh.create_accessor(map_handle);
    for (Tuple& tuple : tuples) {
        tuple = map_tuple_between_meshes(my_mesh, child_mesh, map_accessor, tuple);
    }
    tuples.erase(
        std::remove_if(
            tuples.begin(),
            tuples.end(),
            [](const Tuple& t) -> bool { return t.is_null(); }),
        tuples.end());
    return tuples;
}

std::vector<Tuple> MultiMeshManager::map_to_child_tuples(
    const Mesh& my_mesh,
    const Mesh& child_mesh,
    const Simplex& my_simplex) const
{
    return map_to_child_tuples(my_mesh, child_mesh.m_multi_mesh_manager.child_id(), my_simplex);
}

std::vector<Tuple> MultiMeshManager::map_to_child_tuples(
    const Mesh& my_mesh,
    long child_id,
    const Simplex& my_simplex) const
{
    // this is just to do a little redirection for simpplifying map_to_child (and potentially for a
    // visitor pattern)
    return map_to_child_tuples(my_mesh, m_children.at(child_id), my_simplex);
}

std::vector<Simplex> MultiMeshManager::map_to_child(
    const Mesh& my_mesh,
    const Mesh& child_mesh,
    const Simplex& my_simplex) const
{
    auto tuples = map_to_child_tuples(my_mesh, child_mesh, my_simplex);
    return simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
        tuples,
        my_simplex.primitive_type());
}


// bool MultiMeshManager::is_child_mesh_valid(const Mesh& my_mesh, const Mesh& child_mesh) const
//{
//     assert(&my_mesh.m_multi_mesh_manager == this);
//    // TODO: implement this
//
//    return true;
//}
//
// std::vector<Tuple> MultiMeshManager::map_edge_tuple_to_all_children(
//    const Mesh& my_mesh,
//    const Tuple& edge_tuple)
//{
//    assert(&my_mesh.m_multi_mesh_manager == this);
//    std::vector<Tuple> ret;
//    for (const auto& child_data : m_children) {
//        const Mesh& child_mesh = *child_data.mesh;
//        const auto map_to_child_handle = child_data.map_handle;
//        Tuple child_tuple =
//            map_tuple_between_meshes(my_mesh, child_mesh, map_to_child_handle, edge_tuple);
//        ret.push_back(child_tuple);
//    }
//    return ret;
//}

/*
bool MultiMeshManager::is_map_valid(const Mesh& my_mesh) const
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
bool MultiMeshManager::is_child_map_valid(const Mesh& my_mesh, const ChildData& child_data) const
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
*/
} // namespace wmtk
