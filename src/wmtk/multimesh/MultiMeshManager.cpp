#include "MultiMeshManager.hpp"
#include <cassert>
#include <wmtk/utils/vector_hash.hpp>
//#include <fmt/ranges.h>
#include <functional>
#include <wmtk/Mesh.hpp>
#include <wmtk/attribute/internal/hash.hpp>
#include <wmtk/simplex/closed_star.hpp>
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/cofaces_single_dimension_iterable.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/simplex/utils/make_unique.hpp>
#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleInspector.hpp>
#include <wmtk/utils/vector_hash.hpp>
#include "utils/local_switch_tuple.hpp"
#include "utils/transport_tuple.hpp"
#include "utils/tuple_map_attribute_io.hpp"

namespace wmtk::multimesh {

namespace {} // namespace


void MultiMeshManager::detach_children()
{
    for (auto& data : m_children) {
        auto& m = *data.mesh;
        m.m_multi_mesh_manager.m_parent = nullptr;
        m.m_multi_mesh_manager.m_child_id = -1;
        // TODO: delete attributes
    }

    m_children.clear();
}

Tuple MultiMeshManager::map_tuple_between_meshes(
    const Mesh& source_mesh,
    const Mesh& target_mesh,
    const wmtk::attribute::Accessor<int64_t>& map_accessor,
    const Tuple& source_tuple)
{
    assert(source_mesh.is_valid(source_tuple));

    PrimitiveType source_mesh_primitive_type = source_mesh.top_simplex_type();
    PrimitiveType target_mesh_primitive_type = target_mesh.top_simplex_type();
    PrimitiveType min_primitive_type =
        std::min(source_mesh_primitive_type, target_mesh_primitive_type);
    Tuple source_mesh_target_tuple = source_tuple;
    auto [source_mesh_base_tuple, target_mesh_base_tuple] =
        multimesh::utils::read_tuple_map_attribute(map_accessor, source_tuple);

    if (source_mesh_base_tuple.is_null() || target_mesh_base_tuple.is_null()) {
        return Tuple(); // return null tuple
    }

    // assert(source_mesh.is_valid(source_mesh_base_tuple));
    // assert(target_mesh.is_valid(target_mesh_base_tuple));


    if (source_mesh_base_tuple.m_global_cid != source_mesh_target_tuple.m_global_cid) {
        // guarantee that the source and target tuples refer to the same simplex
        assert(source_mesh_primitive_type > target_mesh_primitive_type);
        const std::vector<Tuple> equivalent_tuples = simplex::top_dimension_cofaces_tuples(
            source_mesh,
            simplex::Simplex(source_mesh, target_mesh_primitive_type, source_tuple));
        for (const Tuple& t : equivalent_tuples) {
            if (t.m_global_cid == source_mesh_base_tuple.m_global_cid) {
                // specific for tet->edge
                if (source_mesh_primitive_type == PrimitiveType::Tetrahedron &&
                    target_mesh_primitive_type == PrimitiveType::Edge) {
                    if (t.m_local_fid == source_mesh_base_tuple.m_local_fid) {
                        source_mesh_target_tuple = t;
                        break;
                    } else {
                        source_mesh_target_tuple =
                            source_mesh.switch_tuple(t, PrimitiveType::Triangle);
                        break;
                    }
                } else {
                    source_mesh_target_tuple = t;
                    break;
                }
            }
        }
    }

    if (source_mesh_primitive_type == PrimitiveType::Tetrahedron &&
        target_mesh_primitive_type == PrimitiveType::Edge) {
        if (source_mesh_target_tuple.m_local_fid != source_mesh_base_tuple.m_local_fid) {
            source_mesh_target_tuple =
                source_mesh.switch_tuple(source_mesh_target_tuple, PrimitiveType::Triangle);
        }
    }

    assert(
        source_mesh_base_tuple.m_global_cid ==
        source_mesh_target_tuple
            .m_global_cid); // make sure that local tuple operations will find a valid sequence

    // we want to repeat switches from source_base_tuple -> source_tuple to
    // target_base _tuple -> return value
    //
    return multimesh::utils::transport_tuple(
        source_mesh_base_tuple,
        source_mesh_target_tuple,
        source_mesh_primitive_type,
        target_mesh_base_tuple,
        target_mesh_primitive_type);
}


MultiMeshManager::MultiMeshManager(int64_t dimension)
    : m_has_child_mesh_in_dimension(dimension, false)
{}

MultiMeshManager::~MultiMeshManager() = default;
MultiMeshManager::MultiMeshManager(const MultiMeshManager& o) = default;
MultiMeshManager::MultiMeshManager(MultiMeshManager&& o) = default;
MultiMeshManager& MultiMeshManager::operator=(const MultiMeshManager& o) = default;
MultiMeshManager& MultiMeshManager::operator=(MultiMeshManager&& o) = default;

// attribute directly hashes its "children" components so it overrides "child_hashes"
std::map<std::string, const wmtk::utils::Hashable*> MultiMeshManager::child_hashables() const
{
    std::map<std::string, const wmtk::utils::Hashable*> ret;
    for (const auto& c : m_children) {
        assert(bool(c.mesh));
        auto id = c.mesh->absolute_multi_mesh_id();
        std::string name = fmt::format("child_map_[{}]", fmt::join(id, ","));
        ret[name] = c.mesh.get();
    }
    return ret;
}
std::map<std::string, std::size_t> MultiMeshManager::child_hashes() const
{
    // default implementation pulls the child attributes (ie the attributes)
    std::map<std::string, std::size_t> ret = wmtk::utils::MerkleTreeInteriorNode::child_hashes();
    ret["child_id"] = m_child_id;

    if (m_parent != nullptr) {
        auto id = m_parent->absolute_multi_mesh_id();
        ret["parent_map"] = wmtk::utils::vector_hash(id);
    } else {
        ret["parent_map"] = 0;
    }


    const std::hash<TypedAttributeHandle<int64_t>> attr_hasher;
    ret["parent_map_handle"] = attr_hasher(map_to_parent_handle);
    for (const auto& c : m_children) {
        assert(bool(c.mesh));
        auto id = c.mesh->absolute_multi_mesh_id();
        std::string name = fmt::format("child_map_[{}]", fmt::join(id, ","));
        ret[name] = attr_hasher(c.map_handle);
    }
    return ret;
}

bool MultiMeshManager::is_root() const
{
    return m_parent == nullptr;
}

int64_t MultiMeshManager::child_id() const
{
    if (is_root()) {
        throw std::runtime_error("Tried to access the child id of a mesh that is in fact a root");
    }
    return m_child_id;
}

std::vector<int64_t> MultiMeshManager::absolute_id() const
{
    if (is_root()) {
        return {};
    } else {
        auto id = m_parent->m_multi_mesh_manager.absolute_id();
        id.emplace_back(m_child_id);
        return id;
    }
}
std::vector<int64_t> MultiMeshManager::relative_id(const Mesh& my_mesh, const Mesh& parent) const
{
    assert(is_child(my_mesh, parent));
    if (&parent == &my_mesh) {
        return {};
    } else {
        assert(!is_root());
        auto id = m_parent->m_multi_mesh_manager.relative_id(*m_parent, parent);
        id.emplace_back(m_child_id);
        return id;
    }
}

bool MultiMeshManager::is_child(const Mesh& my_mesh, const Mesh& parent_mesh) const
{
    if (&parent_mesh == &my_mesh) {
        return true;
    } else {
        if (is_root()) {
            return false;
        } else {
            return m_parent->m_multi_mesh_manager.is_child(*m_parent, parent_mesh);
        }
    }
}


void MultiMeshManager::register_child_mesh(
    Mesh& my_mesh,
    const std::shared_ptr<Mesh>& child_mesh_ptr,
    const std::vector<std::array<Tuple, 2>>& child_tuple_my_tuple_map)
{
    assert((&my_mesh.m_multi_mesh_manager) == this);
    assert(bool(child_mesh_ptr));

    Mesh& child_mesh = *child_mesh_ptr;

    const PrimitiveType child_primitive_type = child_mesh.top_simplex_type();
    const int64_t new_child_id = int64_t(m_children.size());

    m_has_child_mesh_in_dimension[child_mesh.top_cell_dimension()] = true;

    auto child_to_parent_handle = child_mesh.register_attribute_typed<int64_t>(
        child_to_parent_map_attribute_name(),
        child_primitive_type,
        wmtk::multimesh::utils::TWO_TUPLE_SIZE,
        false,
        wmtk::multimesh::utils::DEFAULT_TUPLES_VALUES);

    // TODO: make sure that this attribute doesnt already exist
    auto parent_to_child_handle = my_mesh.register_attribute_typed<int64_t>(
        parent_to_child_map_attribute_name(new_child_id),
        child_primitive_type,
        wmtk::multimesh::utils::TWO_TUPLE_SIZE,
        false,
        wmtk::multimesh::utils::DEFAULT_TUPLES_VALUES);


    auto child_to_parent_accessor = child_mesh.create_accessor(child_to_parent_handle);
    auto parent_to_child_accessor = my_mesh.create_accessor(parent_to_child_handle);


    MultiMeshManager& child_manager = child_mesh.m_multi_mesh_manager;

    // update on child_mesh
    child_manager.map_to_parent_handle = child_to_parent_handle;
    child_manager.m_child_id = new_child_id;
    child_manager.m_parent = &my_mesh;

    // update myself
    m_children.emplace_back(ChildData{child_mesh_ptr, parent_to_child_handle});

    // register maps
    for (const auto& [child_tuple, my_tuple] : child_tuple_my_tuple_map) {
        assert(my_mesh.is_valid(my_tuple));
        assert(child_mesh_ptr->is_valid(child_tuple));
        wmtk::multimesh::utils::symmetric_write_tuple_map_attributes(
            parent_to_child_accessor,
            child_to_parent_accessor,
            my_tuple,
            child_tuple);
    }
}

void MultiMeshManager::deregister_child_mesh(
    Mesh& my_mesh,
    const std::shared_ptr<Mesh>& child_mesh_ptr)
{
    Mesh& child_mesh = *child_mesh_ptr;

    MultiMeshManager& child_manager = child_mesh.m_multi_mesh_manager;
    MultiMeshManager& parent_manager = my_mesh.m_multi_mesh_manager;


    auto& child_to_parent_handle = child_manager.map_to_parent_handle;

    const int64_t child_id = child_manager.child_id();
    ChildData& child_data = parent_manager.m_children[child_id];
    auto& parent_to_child_handle = child_data.map_handle;

    assert(child_data.mesh == child_mesh_ptr);
    assert(child_manager.children()
               .empty()); // The current implementation does not update the attributes properly for
                          // the case that the child also has children

    // remove map attribute from parent
    my_mesh.m_attribute_manager.get<int64_t>(child_mesh.top_simplex_type())
        .remove_attributes({parent_to_child_handle.base_handle()});

    // remove map attribute from child
    child_mesh.m_attribute_manager.get<int64_t>(child_mesh.top_simplex_type())
        .remove_attributes({child_to_parent_handle.base_handle()});

    // set child_id to -1 --> make it a root
    child_manager.m_child_id = -1;
    // remove parent pointer
    child_manager.m_parent = nullptr;

    // remove child_data from parent
    auto& children = parent_manager.m_children;
    children.erase(children.begin() + child_id);

    update_child_handles(my_mesh);
}


std::vector<wmtk::attribute::TypedAttributeHandle<int64_t>> MultiMeshManager::map_handles() const
{
    std::vector<wmtk::attribute::TypedAttributeHandle<int64_t>> handles;
    if (map_to_parent_handle.is_valid()) {
        handles.emplace_back(map_to_parent_handle);
    }
    for (const auto& cd : m_children) {
        handles.emplace_back(cd.map_handle);
    }
    return handles;
}

/*
 * TODO: It is the consumer's responsibility to generate the identity map via a utility function
void MultiMeshManager::register_child_mesh(
    Mesh& my_mesh,
    std::shared_ptr<Mesh> child_mesh,
    const std::vector<int64_t>& child_mesh_simplex_id_map)
{
    PrimitiveType map_type = child_mesh->top_simplex_type();
    std::vector<std::array<Tuple, 2>> child_tuple_my_tuple_map;

    for (int64_t child_cell_id = 0; child_cell_id < int64_t(child_mesh_simplex_id_map.size());
         ++child_cell_id) {
        int64_t parent_cell_id = child_mesh_simplex_id_map[child_cell_id];
        child_tuple_my_tuple_map.push_back(
            {child_mesh->tuple_from_id(map_type, child_cell_id),
             my_mesh.tuple_from_id(map_type, parent_cell_id)});
    }
    register_child_mesh(my_mesh, child_mesh, child_tuple_my_tuple_map);
}
*/

const Mesh& MultiMeshManager::get_root_mesh(const Mesh& my_mesh) const
{
    if (is_root()) {
        return my_mesh;
    } else {
        return m_parent->m_multi_mesh_manager.get_root_mesh(*m_parent);
    }
}
Mesh& MultiMeshManager::get_root_mesh(Mesh& my_mesh)
{
    if (is_root()) {
        return my_mesh;
    } else {
        return m_parent->m_multi_mesh_manager.get_root_mesh(*m_parent);
    }
}

const Mesh& MultiMeshManager::get_child_mesh(
    const Mesh& my_mesh,
    const std::vector<int64_t>& relative_id) const
{
    assert((&my_mesh.m_multi_mesh_manager) == this);

    const Mesh* cur_mesh = &my_mesh;

    for (auto it = relative_id.cbegin(); it != relative_id.cend(); ++it) {
        // get the select ID from the child map
        int64_t child_index = *it;
        const ChildData& cd = cur_mesh->m_multi_mesh_manager.m_children.at(child_index);

        cur_mesh = cd.mesh.get();

        // the front id of the current mesh should be the child index from this iteration
        assert(cur_mesh->m_multi_mesh_manager.m_child_id == child_index);
    }

    return *cur_mesh;
}
Mesh& MultiMeshManager::get_child_mesh(Mesh& my_mesh, const std::vector<int64_t>& relative_id)
{
    return const_cast<Mesh&>(get_child_mesh(const_cast<const Mesh&>(my_mesh), relative_id));
}
const Mesh& MultiMeshManager::get_mesh(const Mesh& my_mesh, const std::vector<int64_t>& absolute_id)
    const
{
    const Mesh& root = get_root_mesh(my_mesh);
    return root.m_multi_mesh_manager.get_child_mesh(root, absolute_id);
}

Mesh& MultiMeshManager::get_mesh(Mesh& my_mesh, const std::vector<int64_t>& absolute_id)
{
    Mesh& root = get_root_mesh(my_mesh);
    return root.m_multi_mesh_manager.get_child_mesh(root, absolute_id);
}
std::vector<std::shared_ptr<Mesh>> MultiMeshManager::get_child_meshes() const
{
    std::vector<std::shared_ptr<Mesh>> ret;
    ret.reserve(m_children.size());
    for (const ChildData& cd : m_children) {
        ret.emplace_back(cd.mesh);
    }
    return ret;
}

std::vector<simplex::Simplex> MultiMeshManager::map(
    const Mesh& my_mesh,
    const Mesh& other_mesh,
    const simplex::Simplex& my_simplex) const
{
    const auto ret_tups = map_tuples(my_mesh, other_mesh, my_simplex);
    return simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
        other_mesh,
        ret_tups,
        my_simplex.primitive_type());
}
std::vector<simplex::Simplex> MultiMeshManager::lub_map(
    const Mesh& my_mesh,
    const Mesh& other_mesh,
    const simplex::Simplex& my_simplex) const
{
    if (&my_mesh == &other_mesh) {
        return {my_simplex};
    }
    const auto ret_tups = lub_map_tuples(my_mesh, other_mesh, my_simplex);
    return simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
        other_mesh,
        ret_tups,
        my_simplex.primitive_type());
}

std::pair<const Mesh&, Tuple> MultiMeshManager::map_up_to_tuples(
    const Mesh& my_mesh,
    const simplex::Simplex& my_simplex,
    int64_t depth) const
{
    assert((&my_mesh.m_multi_mesh_manager) == this);
    const PrimitiveType pt = my_simplex.primitive_type();

    // get a root tuple by converting the tuple up parent meshes until root is found
    Tuple cur_tuple = my_simplex.tuple();
    const Mesh* cur_mesh = &my_mesh;
    for (int64_t d = 0; d < depth; ++d) {
        cur_tuple = cur_mesh->m_multi_mesh_manager.map_tuple_to_parent_tuple(*cur_mesh, cur_tuple);
        cur_mesh = cur_mesh->m_multi_mesh_manager.m_parent;
        assert(cur_mesh != nullptr);
    }
    // assert(cur_mesh->m_multi_mesh_manager
    //            .is_root()); // cur_mesh == nullptr if we just walked past the root node so we stop

    // bieng lazy about how i set cur_mesh to nullptr above - could simplify the loop to optimize
    return std::pair<const Mesh&, Tuple>(*cur_mesh, cur_tuple);
}

std::vector<Tuple> MultiMeshManager::map_down_relative_tuples(
    const Mesh& my_mesh,
    const simplex::Simplex& my_simplex,
    const std::vector<int64_t>& relative_id) const
{
    assert((&my_mesh.m_multi_mesh_manager) == this);

    const PrimitiveType pt = my_simplex.primitive_type();
    // note that (cur_mesh, tuples) always match (i.e tuples are tuples from cur_mesh)
    std::vector<Tuple> tuples;
    tuples.emplace_back(my_simplex.tuple());
    const Mesh* cur_mesh = &my_mesh;

    for (auto it = relative_id.cbegin(); it != relative_id.cend(); ++it) {
        // get the select ID from the child map
        int64_t child_index = *it;
        const ChildData& cd = cur_mesh->m_multi_mesh_manager.m_children.at(child_index);

        // for every tuple we have try to collect all versions
        std::vector<Tuple> new_tuples;
        for (const Tuple& t : tuples) {
            // get new tuples for every version that exists
            std::vector<Tuple> n = cur_mesh->m_multi_mesh_manager.map_to_child_tuples(
                *cur_mesh,
                cd,
                simplex::Simplex(*cur_mesh, pt, t));
            // append to the current set of new tuples
            new_tuples.insert(new_tuples.end(), n.begin(), n.end());
        }
        // update the (mesh,tuples) pair
        tuples = std::move(new_tuples);
        cur_mesh = cd.mesh.get();

        // the front id of the current mesh should be the child index from this iteration
        assert(cur_mesh->m_multi_mesh_manager.m_child_id == child_index);
    }

    // visitor.map(equivalent_tuples, my_simplex.primitive_type());

    return tuples;
}

std::vector<Tuple> MultiMeshManager::map_tuples(
    const Mesh& my_mesh,
    const Mesh& other_mesh,
    const simplex::Simplex& my_simplex) const
{
    const auto my_id = absolute_id();
    const auto other_id = other_mesh.absolute_multi_mesh_id();

    int64_t depth = my_id.size();

    auto [root_ref, tuple] = map_up_to_tuples(my_mesh, my_simplex, depth);
    const simplex::Simplex simplex(root_ref, my_simplex.primitive_type(), tuple);

    return root_ref.m_multi_mesh_manager.map_down_relative_tuples(root_ref, simplex, other_id);
}

std::vector<Tuple> MultiMeshManager::lub_map_tuples(
    const Mesh& my_mesh,
    const Mesh& other_mesh,
    const simplex::Simplex& my_simplex) const
{
    if (&my_mesh == &other_mesh) {
        return {my_simplex.tuple()};
    }
    const auto my_id = absolute_id();
    const auto other_id = other_mesh.absolute_multi_mesh_id();
    const auto lub_id = least_upper_bound_id(my_id, other_id);

    int64_t depth = my_id.size() - lub_id.size();

    auto [local_root_ref, tuple] = map_up_to_tuples(my_mesh, my_simplex, depth);
    assert(other_mesh.m_multi_mesh_manager.is_child(other_mesh, local_root_ref));

    const simplex::Simplex simplex(local_root_ref, my_simplex.primitive_type(), tuple);

    auto other_relative_id = relative_id(lub_id, other_id);
    return local_root_ref.m_multi_mesh_manager.map_down_relative_tuples(
        local_root_ref,
        simplex,
        other_relative_id);
}

simplex::Simplex MultiMeshManager::map_to_root(
    const Mesh& my_mesh,
    const simplex::Simplex& my_simplex) const
{
    return simplex::Simplex(my_simplex.primitive_type(), map_to_root_tuple(my_mesh, my_simplex));
}

Tuple MultiMeshManager::map_to_root_tuple(const Mesh& my_mesh, const simplex::Simplex& my_simplex)
    const
{
    const Tuple t = map_tuple_to_root_tuple(my_mesh, my_simplex.tuple());
    assert(get_root_mesh(my_mesh).is_valid(t));
    return t;
}
Tuple MultiMeshManager::map_tuple_to_root_tuple(const Mesh& my_mesh, const Tuple& my_tuple) const
{
    assert(&my_mesh.m_multi_mesh_manager == this);
    if (my_mesh.m_multi_mesh_manager.is_root()) {
        assert(my_mesh.is_valid(my_tuple));
        return my_tuple;
    } else {
        const Tuple ptup = map_tuple_to_parent_tuple(my_mesh, my_tuple);
        assert(m_parent->is_valid(ptup));
        return m_parent->m_multi_mesh_manager.map_tuple_to_root_tuple(*m_parent, ptup);
    }
}


simplex::Simplex MultiMeshManager::map_to_parent(
    const Mesh& my_mesh,
    const simplex::Simplex& my_simplex) const
{
    return simplex::Simplex(
        my_simplex.primitive_type(),
        map_tuple_to_parent_tuple(my_mesh, my_simplex.tuple()));
}
Tuple MultiMeshManager::map_to_parent_tuple(const Mesh& my_mesh, const simplex::Simplex& my_simplex)
    const
{
    return map_tuple_to_parent_tuple(my_mesh, my_simplex.tuple());
}

Tuple MultiMeshManager::map_tuple_to_parent_tuple(const Mesh& my_mesh, const Tuple& my_tuple) const
{
    assert((&my_mesh.m_multi_mesh_manager) == this);
    assert(!is_root());

    const Mesh& parent_mesh = *m_parent;

    const auto& map_handle = map_to_parent_handle;
    // assert(!map_handle.is_null());

    auto map_accessor = my_mesh.create_const_accessor(map_handle);
    return map_tuple_between_meshes(my_mesh, parent_mesh, map_accessor, my_tuple);
}


std::vector<Tuple> MultiMeshManager::map_to_child_tuples(
    const Mesh& my_mesh,
    const ChildData& child_data,
    const simplex::Simplex& my_simplex) const
{
    assert((&my_mesh.m_multi_mesh_manager) == this);

    const Mesh& child_mesh = *child_data.mesh;
    if (child_mesh.top_simplex_type() < my_simplex.primitive_type()) {
        return {};
    }
    const auto map_handle = child_data.map_handle;
    auto map_accessor = my_mesh.create_const_accessor(map_handle);
    // we will overwrite these tuples inline with the mapped ones while running down the map
    // functionalities

    std::vector<Tuple> tuples_original;
    // original version
    {
        tuples_original = simplex::cofaces_single_dimension_tuples(
            my_mesh,
            my_simplex,
            child_mesh.top_simplex_type());

        /*
            get all tuples of child mesh top simplex type that contain my_simplex
        */

        for (Tuple& tuple : tuples_original) {
            tuple = map_tuple_between_meshes(my_mesh, child_mesh, map_accessor, tuple);
        }
        tuples_original.erase(
            std::remove_if(
                tuples_original.begin(),
                tuples_original.end(),
                [](const Tuple& t) -> bool { return t.is_null(); }),
            tuples_original.end());
        tuples_original = wmtk::simplex::utils::make_unique_tuples(
            child_mesh,
            tuples_original,
            my_simplex.primitive_type());
    }
    std::vector<Tuple> tuples_new_wout_sorting;
    std::vector<Tuple> tuples_new_with_sorting;
    {
        simplex::SimplexCollection sc(my_mesh);
        for (const Tuple& tuple : simplex::cofaces_single_dimension_iterable(
                 my_mesh,
                 my_simplex,
                 child_mesh.top_simplex_type())) {
            sc.add(child_mesh.top_simplex_type(), tuple);
        }

        // unsorted
        {
            std::vector<Tuple> child_tuples_wout;
            child_tuples_wout.reserve(sc.size());
            for (const simplex::Simplex& s : sc.simplex_vector()) {
                const Tuple tuple =
                    map_tuple_between_meshes(my_mesh, child_mesh, map_accessor, s.tuple());
                if (!tuple.is_null()) {
                    child_tuples_wout.emplace_back(tuple);
                }
            }
            tuples_new_wout_sorting = wmtk::simplex::utils::make_unique_tuples(
                child_mesh,
                child_tuples_wout,
                my_simplex.primitive_type());
        }
        sc.sort();

        // sorted
        {
            std::vector<Tuple> child_tuples;
            child_tuples.reserve(sc.size());
            for (const simplex::Simplex& s : sc.simplex_vector()) {
                const Tuple tuple =
                    map_tuple_between_meshes(my_mesh, child_mesh, map_accessor, s.tuple());
                if (!tuple.is_null()) {
                    child_tuples.emplace_back(tuple);
                }
            }
            tuples_new_with_sorting = wmtk::simplex::utils::make_unique_tuples(
                child_mesh,
                child_tuples,
                my_simplex.primitive_type());
        }
    }

    // return tuples_original;
    return tuples_new_with_sorting;
    // return tuples_new_wout_sorting;
}

std::vector<Tuple> MultiMeshManager::map_to_child_tuples(
    const Mesh& my_mesh,
    const Mesh& child_mesh,
    const simplex::Simplex& my_simplex) const
{
    return map_to_child_tuples(my_mesh, child_mesh.m_multi_mesh_manager.child_id(), my_simplex);
}

std::vector<Tuple> MultiMeshManager::map_to_child_tuples(
    const Mesh& my_mesh,
    int64_t child_id,
    const simplex::Simplex& my_simplex) const
{
    // this is just to do a little redirection for simpplifying map_to_child (and potentially for a
    // visitor pattern)
    return map_to_child_tuples(my_mesh, m_children.at(child_id), my_simplex);
}

std::vector<simplex::Simplex> MultiMeshManager::map_to_child(
    const Mesh& my_mesh,
    const Mesh& child_mesh,
    const simplex::Simplex& my_simplex) const
{
    auto tuples = map_to_child_tuples(my_mesh, child_mesh, my_simplex);

    return simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
        child_mesh,
        tuples,
        my_simplex.primitive_type());
}


std::vector<std::array<Tuple, 2>> MultiMeshManager::same_simplex_dimension_surjection(
    const Mesh& parent,
    const Mesh& child,
    const std::vector<int64_t>& parent_simplices)
{
    PrimitiveType primitive_type = parent.top_simplex_type();
#if !defined(NDEBUG)
    if (primitive_type != child.top_simplex_type()) {
        throw std::runtime_error(
            "Cannot use same_simplex_dimension_bijection on meshes with simplex dimensions");
    }
#endif

    int64_t size = child.capacity(primitive_type);
    assert(size == int64_t(parent_simplices.size()));
    std::vector<std::array<Tuple, 2>> ret;
    ret.reserve(size);

    auto parent_flag_accessor = parent.get_const_flag_accessor(primitive_type);
    auto child_flag_accessor = child.get_const_flag_accessor(primitive_type);

    for (int64_t index = 0; index < size; ++index) {
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

std::string MultiMeshManager::parent_to_child_map_attribute_name(int64_t index)
{
    return fmt::format("map_to_child_{}", index);
}
std::array<wmtk::attribute::Accessor<int64_t>, 2> MultiMeshManager::get_map_accessors(
    Mesh& my_mesh,
    ChildData& c)
{
    Mesh& child_mesh = *c.mesh;
    const auto& child_to_parent_handle = child_mesh.m_multi_mesh_manager.map_to_parent_handle;
    const auto& parent_to_child_handle = c.map_handle;


    return std::array<wmtk::attribute::Accessor<int64_t>, 2>{
        {my_mesh.create_accessor(parent_to_child_handle),
         child_mesh.create_accessor(child_to_parent_handle)}};
}
std::array<const wmtk::attribute::Accessor<int64_t>, 2> MultiMeshManager::get_map_const_accessors(
    const Mesh& my_mesh,
    const ChildData& c) const
{
    const Mesh& child_mesh = *c.mesh;
    const auto& child_to_parent_handle = child_mesh.m_multi_mesh_manager.map_to_parent_handle;
    const auto& parent_to_child_handle = c.map_handle;


    return std::array<const wmtk::attribute::Accessor<int64_t>, 2>{
        {my_mesh.create_const_accessor(parent_to_child_handle),
         child_mesh.create_const_accessor(child_to_parent_handle)}};
}
std::string MultiMeshManager::child_to_parent_map_attribute_name()
{
    return "map_to_parent";
}


// remove after bug fix
void MultiMeshManager::check_map_valid(const Mesh& my_mesh) const
{
    for (int64_t index = 0; index < int64_t(children().size()); ++index) {
        const auto& child_data = children()[index];
        assert(bool(child_data.mesh));
        assert(child_data.mesh->absolute_multi_mesh_id().front() == index);
        check_child_map_valid(my_mesh, child_data);
    }
}

void MultiMeshManager::check_child_map_valid(const Mesh& my_mesh, const ChildData& child_data) const
{
    const Mesh& child_mesh = *child_data.mesh;
    const auto parent_to_child_handle = child_data.map_handle;
    PrimitiveType map_type = child_mesh.top_simplex_type();

    const std::string c_to_p_name = child_to_parent_map_attribute_name();

    assert(child_mesh.has_attribute<int64_t>(c_to_p_name, map_type));
    auto child_to_parent_handle =
        child_mesh.get_attribute_handle<int64_t>(c_to_p_name, map_type).as<int64_t>();
    auto child_cell_flag_accessor = child_mesh.get_flag_accessor(map_type);

    auto all_child_tuples = child_mesh.get_all(map_type);

    for (const Tuple& child_tuple : all_child_tuples) {
        logger().debug(
            "[{} -> {}] Checking child tuple {}",
            fmt::join(absolute_id(), ","),
            fmt::join(child_mesh.absolute_multi_mesh_id(), ","),
            wmtk::utils::TupleInspector::as_string(child_tuple));
        // 1. test if all maps in child_mesh exisits
        auto [child_tuple_from_child, parent_tuple_from_child] =
            multimesh::utils::read_tuple_map_attribute_slow(
                child_mesh,
                child_to_parent_handle,
                child_tuple);

        // 2. test if tuples in maps are valid (and up_to_date)
        {
            logger().debug(
                "[{} -> {}] Checking asserts from child {} {} (input tuple was {})",
                fmt::join(absolute_id(), ","),
                fmt::join(child_mesh.absolute_multi_mesh_id(), ","),
                wmtk::utils::TupleInspector::as_string(child_tuple_from_child),
                wmtk::utils::TupleInspector::as_string(child_tuple_from_child),
                wmtk::utils::TupleInspector::as_string(child_tuple));
            assert(child_mesh.is_valid(child_tuple_from_child));
            assert(child_mesh.is_valid(child_tuple_from_child));
            assert(my_mesh.is_valid(parent_tuple_from_child));
        }

        // 3. test if map is symmetric
        {
            auto [parent_tuple_from_parent, child_tuple_from_parent] =
                multimesh::utils::read_tuple_map_attribute_slow(
                    my_mesh,
                    parent_to_child_handle,
                    parent_tuple_from_child);
            logger().debug(
                "[{} -> {}] Checking asserts from child {} {}",
                fmt::join(absolute_id(), ","),
                fmt::join(child_mesh.absolute_multi_mesh_id(), ","),
                wmtk::utils::TupleInspector::as_string(parent_tuple_from_parent),
                wmtk::utils::TupleInspector::as_string(child_tuple_from_parent));

            assert(
                (child_tuple_from_child == child_tuple_from_parent &&
                 parent_tuple_from_child == parent_tuple_from_parent));
        }

        // 4. test switch_top_simplex operation
        // for 4, current code support only mapping between triangle meshes
        if (map_type == PrimitiveType::Triangle &&
            my_mesh.top_simplex_type() == PrimitiveType::Triangle) {
            Tuple cur_child_tuple = child_tuple_from_child;
            Tuple cur_parent_tuple = parent_tuple_from_child;

            auto child_to_parent_accessor =
                child_mesh.create_const_accessor(child_to_parent_handle);
            for (int i = 0; i < 3; i++) {
                if (!child_mesh.is_boundary(PrimitiveType::Edge, cur_child_tuple)) {
                    assert(!my_mesh.is_boundary(PrimitiveType::Edge, cur_parent_tuple));

#ifndef NDEBUG
                    Tuple child_tuple_opp =
                        child_mesh.switch_tuple(cur_child_tuple, PrimitiveType::Triangle);
                    Tuple parent_tuple_opp =
                        my_mesh.switch_tuple(cur_parent_tuple, PrimitiveType::Triangle);
                    assert(
                        parent_tuple_opp == map_tuple_between_meshes(
                                                child_mesh,
                                                my_mesh,
                                                child_to_parent_accessor,
                                                child_tuple_opp));
#endif
                }
                cur_child_tuple = child_mesh.switch_tuples(
                    cur_child_tuple,
                    {PrimitiveType::Vertex, PrimitiveType::Edge});
                cur_parent_tuple = my_mesh.switch_tuples(
                    cur_parent_tuple,
                    {PrimitiveType::Vertex, PrimitiveType::Edge});
            }
        } else if (
            map_type == PrimitiveType::Edge &&
            my_mesh.top_simplex_type() == PrimitiveType::Triangle) {
            if (!my_mesh.is_boundary(PrimitiveType::Edge, parent_tuple_from_child)) {
                auto parent_to_child_accessor =
                    my_mesh.create_const_accessor(parent_to_child_handle);
#ifndef NDEBUG
                const Tuple parent_tuple_opp =
                    my_mesh.switch_tuple(parent_tuple_from_child, PrimitiveType::Triangle);
                assert(
                    child_tuple_from_child == map_tuple_between_meshes(
                                                  my_mesh,
                                                  child_mesh,
                                                  parent_to_child_accessor,
                                                  parent_tuple_opp));
#endif
            }
        } else {
            // TODO: implement other cases
            continue;
        }
    }
}


std::vector<int64_t> MultiMeshManager::least_upper_bound_id(
    const std::vector<int64_t>& a,
    const std::vector<int64_t>& b)
{
    std::vector<int64_t> ret;
    size_t size = std::min(a.size(), b.size());
    for (size_t j = 0; j < size; ++j) {
        if (a[j] == b[j]) {
            ret.emplace_back(a[j]);
        } else {
            break;
        }
    }
    return ret;
}
std::vector<int64_t> MultiMeshManager::relative_id(
    const std::vector<int64_t>& parent,
    const std::vector<int64_t>& child)
{
    assert(is_child(child, parent));
    std::vector<int64_t> ret;
    std::copy(child.begin() + parent.size(), child.end(), std::back_inserter(ret));
    return ret;
}
bool MultiMeshManager::is_child(
    const std::vector<int64_t>& child,
    const std::vector<int64_t>& parent)
{
    if (parent.size() > child.size()) {
        return false;
    }
    for (size_t j = 0; j < parent.size(); ++j) {
        if (parent[j] != child[j]) {
            return false;
        }
    }
    return true;
}

void MultiMeshManager::serialize(MeshWriter& writer, const Mesh* local_root) const
{
    for (const auto& c : m_children) {
        c.mesh->serialize(writer, local_root);
    }
}

bool MultiMeshManager::has_child_mesh() const
{
    for (const bool c : m_has_child_mesh_in_dimension) {
        if (c) {
            return true;
        }
    }
    return false;
}

bool MultiMeshManager::can_map(
    const Mesh& my_mesh,
    const Mesh& other_mesh,
    const simplex::Simplex& my_simplex) const
{
    auto& root = my_mesh.get_multi_mesh_root();
    const simplex::Simplex root_simplex(
        root,
        my_simplex.primitive_type(),
        map_to_root_tuple(my_mesh, my_simplex));
    return root.m_multi_mesh_manager.can_map_child(root, other_mesh, root_simplex);
}
bool MultiMeshManager::can_map_child(
    const Mesh& my_mesh,
    const Mesh& other_mesh,
    const simplex::Simplex& my_simplex) const
{
    if (my_simplex.primitive_type() > other_mesh.top_simplex_type()) {
        return false;
    }
    const auto my_id = absolute_id();
    const auto other_id = other_mesh.absolute_multi_mesh_id();

    int64_t depth = my_id.size();

    auto [root_ref, tuple] = map_up_to_tuples(my_mesh, my_simplex, depth);
    const simplex::Simplex simplex(root_ref, my_simplex.primitive_type(), tuple);

    return !root_ref.m_multi_mesh_manager.map_down_relative_tuples(root_ref, simplex, other_id)
                .empty();
}
} // namespace wmtk::multimesh
