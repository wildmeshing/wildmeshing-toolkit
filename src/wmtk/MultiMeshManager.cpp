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
    const std::vector<std::array<Tuple, 2>>& child_tuple_my_tuple_map)
{
    assert(&my_mesh.m_multi_mesh_manager == this);
    assert(bool(child_mesh_ptr));

    Mesh& child_mesh = *child_mesh_ptr;

    PrimitiveType child_primitive_type = child_mesh.top_simplex_type();
    long new_child_id = long(m_children.size());

    auto child_to_parent_handle = child_mesh.register_attribute<long>(
        child_to_parent_map_attribute_name(),
        child_primitive_type,
        10,
        false,
        -1);

    // TODO: make sure that this attribute doesnt already exist
    auto parent_to_child_handle = my_mesh.register_attribute<long>(
        parent_to_child_map_attribute_name(new_child_id),
        child_primitive_type,
        10,
        false,
        -1);


    auto child_to_parent_accessor = child_mesh.create_accessor(child_to_parent_handle);
    auto parent_to_child_accessor = my_mesh.create_accessor(parent_to_child_handle);

    // register maps
    for (const auto& [child_tuple, my_tuple] : child_tuple_my_tuple_map) {
        multimesh::utils::write_tuple_map_attribute(
            parent_to_child_accessor,
            my_tuple,
            child_tuple);
        multimesh::utils::write_tuple_map_attribute(
            child_to_parent_accessor,
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
    std::vector<std::array<Tuple, 2>> child_tuple_my_tuple_map;

    for (long child_cell_id = 0; child_cell_id < long(child_mesh_simplex_id_map.size());
         ++child_cell_id) {
        long parent_cell_id = child_mesh_simplex_id_map[child_cell_id];
        child_tuple_my_tuple_map.push_back(
            {child_mesh->tuple_from_id(map_type, child_cell_id),
             my_mesh.tuple_from_id(map_type, parent_cell_id)});
    }
    register_child_mesh(my_mesh, child_mesh, child_tuple_my_tuple_map);
}
*/

const Mesh& MultiMeshManager::get_root_mesh(const Mesh& my_mesh) const
{
    if (m_parent == nullptr) {
        return my_mesh;
    } else {
        return m_parent->m_multi_mesh_manager.get_root_mesh(*m_parent);
    }
}
Mesh& MultiMeshManager::get_root_mesh(Mesh& my_mesh)
{
    if (m_parent == nullptr) {
        return my_mesh;
    } else {
        return m_parent->m_multi_mesh_manager.get_root_mesh(*m_parent);
    }
}
std::vector<Simplex>
MultiMeshManager::map(const Mesh& my_mesh, const Mesh& other_mesh, const Simplex& my_simplex) const
{
    const auto ret_tups = map_tuples(my_mesh, other_mesh, my_simplex);
    return simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
        ret_tups,
        my_simplex.primitive_type());
}
std::vector<Tuple> MultiMeshManager::map_tuples(
    const Mesh& my_mesh,
    const Mesh& other_mesh,
    const Simplex& my_simplex) const
{
    const PrimitiveType pt = my_simplex.primitive_type();
    assert(&my_mesh.m_multi_mesh_manager == this);
    // TODO: construct relative positions
    std::vector<Tuple> equivalent_tuples = simplex::top_level_cofaces_tuples(my_mesh, my_simplex);
    // TODO: construct visitor class that maps up and down
    // MultiMeshMapVisitor visitor(my_mesh, other_mesh);
    // const auto my_id = absolute_id(); someday could be used to map down
    const auto other_id = other_mesh.absolute_multi_mesh_id();
    // TODO: visitor runs along meshes traversing the path

    // get a root tuple
    Tuple cur_tuple = my_simplex.tuple();
    const Mesh* cur_mesh = &my_mesh;
    while (cur_mesh != nullptr) {
        cur_tuple = cur_mesh->m_multi_mesh_manager.map_tuple_to_parent_tuple(*cur_mesh, cur_tuple);
        cur_mesh = cur_mesh->m_multi_mesh_manager.m_parent;
    }

    // bieng lazy about how i set cur_mesh to nullptr above - could simplify the loop to optimize
    cur_mesh = &get_root_mesh(other_mesh);


    // note that (cur_mesh, tuples) always match (i.e tuples are tuples from cur_mesh)
    std::vector<Tuple> tuples;
    tuples.emplace_back(cur_tuple);

    for (auto it = other_id.rbegin(); it != other_id.rend(); ++it) {
        // get the select ID from the child map
        long child_index = *it;
        const ChildData& cd = cur_mesh->m_multi_mesh_manager.m_children.at(child_index);

        // for every tuple we have try to collect all versions
        std::vector<Tuple> new_tuples;
        for (const Tuple& t : tuples) {
            // get new tuples for every version that exists
            std::vector<Tuple> n =
                cur_mesh->m_multi_mesh_manager.map_to_child_tuples(*cur_mesh, cd, Simplex(pt, t));
            // append to teh current set of new tuples
            new_tuples.insert(new_tuples.end(), n.begin(), n.end());
        }
        // update teh (mesh,tuples) pair
        tuples = std::move(new_tuples);
        cur_mesh = cd.mesh.get();

        // the front id of the current mesh should be the child index from this iteration
        assert(cur_mesh->m_multi_mesh_manager.m_child_id == child_index);
    }

    // visitor.map(equivalent_tuples, my_simplex.primitive_type());

    return tuples;
}


Simplex MultiMeshManager::map_to_parent(const Mesh& my_mesh, const Simplex& my_simplex) const
{
    return Simplex(
        my_simplex.primitive_type(),
        map_tuple_to_parent_tuple(my_mesh, my_simplex.tuple()));
}
Tuple MultiMeshManager::map_to_parent_tuple(const Mesh& my_mesh, const Simplex& my_simplex) const
{
    return map_tuple_to_parent_tuple(my_mesh, my_simplex.tuple());
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


std::vector<std::array<Tuple, 2>> MultiMeshManager::same_simplex_dimension_surjection(
    const Mesh& parent,
    const Mesh& child,
    const std::vector<long>& parent_simplices)
{
    PrimitiveType primitive_type = parent.top_simplex_type();
    assert(primitive_type == child.top_simplex_type());

    long size = child.capacity(primitive_type);
    assert(size == long(parent_simplices.size()));
    std::vector<std::array<Tuple, 2>> ret;
    ret.reserve(size);

    auto parent_flag_accessor = parent.get_const_flag_accessor(primitive_type);
    auto child_flag_accessor = child.get_const_flag_accessor(primitive_type);

    for (long index = 0; index < size; ++index) {
        const Tuple ct = child.tuple_from_id(primitive_type, index);
        const Tuple pt = parent.tuple_from_id(primitive_type, parent_simplices.at(index));
        if ((parent_flag_accessor.const_scalar_attribute(pt) & 1) == 0) {
            continue;
        }
        if ((child_flag_accessor.const_scalar_attribute(ct) & 1) == 0) {
            continue;
        }

        ret.emplace_back(std::array<Tuple, 2>{{ct, pt}});
    }
    return ret;
}

std::string MultiMeshManager::parent_to_child_map_attribute_name(long index)
{
    return fmt::format("map_to_child_{}", index);
}
std::string MultiMeshManager::child_to_parent_map_attribute_name()
{
    return "map_to_parent";
}
} // namespace wmtk
