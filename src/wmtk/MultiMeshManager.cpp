#include "MultiMeshManager.hpp"
#include <wmtk/utils/vector_hash.hpp>
//#include <fmt/ranges.h>
#include <functional>
#include <wmtk/attribute/internal/hash.hpp>
#include <wmtk/io/MeshWriter.hpp>
#include <wmtk/simplex/closed_star.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/simplex/utils/make_unique.hpp>
#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleInspector.hpp>
#include <wmtk/utils/vector_hash.hpp>
#include "Mesh.hpp"
#include "multimesh/utils/local_switch_tuple.hpp"
#include "multimesh/utils/transport_tuple.hpp"
#include "multimesh/utils/tuple_map_attribute_io.hpp"

namespace wmtk {

namespace {} // namespace

Tuple MultiMeshManager::map_tuple_between_meshes(
    const Mesh& source_mesh,
    const Mesh& target_mesh,
    const ConstAccessor<int64_t>& map_accessor,
    const Tuple& source_tuple)
{
    PrimitiveType source_mesh_primitive_type = source_mesh.top_simplex_type();
    PrimitiveType target_mesh_primitive_type = target_mesh.top_simplex_type();
    PrimitiveType min_primitive_type =
        std::min(source_mesh_primitive_type, target_mesh_primitive_type);
    Tuple source_mesh_target_tuple = source_tuple;
    const auto [source_mesh_base_tuple, target_mesh_base_tuple] =
        multimesh::utils::read_tuple_map_attribute(map_accessor, source_tuple);

    if (source_mesh_base_tuple.is_null() || target_mesh_base_tuple.is_null()) {
        return Tuple(); // return null tuple
    }

    if (source_mesh_base_tuple.m_global_cid != source_mesh_target_tuple.m_global_cid) {
        assert(source_mesh_primitive_type > target_mesh_primitive_type);
        const std::vector<Tuple> equivalent_tuples = simplex::top_dimension_cofaces_tuples(
            source_mesh,
            simplex::Simplex(target_mesh_primitive_type, source_tuple));
        for (const Tuple& t : equivalent_tuples) {
            if (t.m_global_cid == source_mesh_base_tuple.m_global_cid) {
                source_mesh_target_tuple = t;
                break;
            }
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


MultiMeshManager::MultiMeshManager() = default;

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


    constexpr static int64_t TWO_TUPLE_SIZE = 10;
    constexpr static int64_t DEFAULT_TUPLES_VALUES = -1;
    auto child_to_parent_handle = child_mesh.register_attribute_builtin<int64_t>(
        child_to_parent_map_attribute_name(),
        child_primitive_type,
        TWO_TUPLE_SIZE,
        false,
        DEFAULT_TUPLES_VALUES);

    // TODO: make sure that this attribute doesnt already exist
    auto parent_to_child_handle = my_mesh.register_attribute_builtin<int64_t>(
        parent_to_child_map_attribute_name(new_child_id),
        child_primitive_type,
        TWO_TUPLE_SIZE,
        false,
        DEFAULT_TUPLES_VALUES);


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
        wmtk::multimesh::utils::symmetric_write_tuple_map_attributes(
            parent_to_child_accessor,
            child_to_parent_accessor,
            my_tuple,
            child_tuple);
    }
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
        ret_tups,
        my_simplex.primitive_type());
}
std::vector<simplex::Simplex> MultiMeshManager::lub_map(
    const Mesh& my_mesh,
    const Mesh& other_mesh,
    const simplex::Simplex& my_simplex) const
{
    const auto ret_tups = lub_map_tuples(my_mesh, other_mesh, my_simplex);
    return simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
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
    assert(cur_mesh->m_multi_mesh_manager
               .is_root()); // cur_mesh == nullptr if we just walked past the root node so we stop

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
                simplex::Simplex(pt, t));
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
    const simplex::Simplex simplex(my_simplex.primitive_type(), tuple);

    return root_ref.m_multi_mesh_manager.map_down_relative_tuples(root_ref, simplex, other_id);
}

std::vector<Tuple> MultiMeshManager::lub_map_tuples(
    const Mesh& my_mesh,
    const Mesh& other_mesh,
    const simplex::Simplex& my_simplex) const
{
    const auto my_id = absolute_id();
    const auto other_id = other_mesh.absolute_multi_mesh_id();
    const auto lub_id = least_upper_bound_id(my_id, other_id);

    int64_t depth = my_id.size() - lub_id.size();

    auto [local_root_ref, tuple] = map_up_to_tuples(my_mesh, my_simplex, depth);

    const simplex::Simplex simplex(my_simplex.primitive_type(), tuple);

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
    return map_tuple_to_root_tuple(my_mesh, my_simplex.tuple());
}
Tuple MultiMeshManager::map_tuple_to_root_tuple(const Mesh& my_mesh, const Tuple& my_tuple) const
{
    if (my_mesh.m_multi_mesh_manager.is_root()) {
        return my_tuple;
    } else {
        return map_tuple_to_root_tuple(*m_parent, map_tuple_to_parent_tuple(my_mesh, my_tuple));
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

    auto map_accessor = my_mesh.create_accessor(map_handle);
    return map_tuple_between_meshes(my_mesh, parent_mesh, map_accessor, my_tuple);
}

// helper function for map tuple
std::vector<Tuple> get_all_candidate_tuples_for_child_mesh(
    PrimitiveType parent_type,
    PrimitiveType child_type,
    PrimitiveType simplex_type,
    const std::vector<Tuple>& parent_tuples)
{
    if (parent_type == child_type || child_type == simplex_type) {
        return parent_tuples;
    }

    std::vector<Tuple> ret_tuples = parent_tuples;
    switch (simplex_type) {
    case PrimitiveType::Vertex:
        if (parent_type == PrimitiveType::Face) {
            for (const Tuple& t : parent_tuples) {
                ret_tuples.emplace_back(wmtk::multimesh::utils::local_switch_tuple(
                    parent_type,
                    t,
                    PrimitiveType::Edge));
            }
        }
        if (parent_type == PrimitiveType::Tetrahedron) {
            for (const Tuple& t : parent_tuples) {
                ret_tuples.emplace_back(wmtk::multimesh::utils::local_switch_tuple(
                    parent_type,
                    t,
                    PrimitiveType::Edge));
                ret_tuples.emplace_back(wmtk::multimesh::utils::local_switch_tuples(
                    parent_type,
                    t,
                    {PrimitiveType::Face, PrimitiveType::Edge}));
            }
        }

        break;
    case PrimitiveType::Edge:
        if (parent_type == PrimitiveType::Tetrahedron) {
            for (const Tuple& t : parent_tuples) {
                ret_tuples.emplace_back(wmtk::multimesh::utils::local_switch_tuple(
                    parent_type,
                    t,
                    PrimitiveType::Face));
            }
        }
        break;
    case PrimitiveType::Face:
    case PrimitiveType::Tetrahedron:
    case PrimitiveType::HalfEdge:
    default: break;
    }

    return ret_tuples;
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
    // we will overwrite these tuples inline with the mapped ones while running down the map
    // functionalities
    std::vector<Tuple> tuples = simplex::top_dimension_cofaces_tuples(my_mesh, my_simplex);
    /*
        get all tuples of child mesh top simplex type that contain my_simplex
    */
    tuples = get_all_candidate_tuples_for_child_mesh(
        my_mesh.top_simplex_type(),
        child_mesh.top_simplex_type(),
        my_simplex.primitive_type(),
        tuples);

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
    tuples =
        wmtk::simplex::utils::make_unique_tuples(child_mesh, tuples, my_simplex.primitive_type());

    return tuples;
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
std::array<attribute::MutableAccessor<int64_t>, 2> MultiMeshManager::get_map_accessors(
    Mesh& my_mesh,
    ChildData& c)
{
    Mesh& child_mesh = *c.mesh;
    const auto& child_to_parent_handle = child_mesh.m_multi_mesh_manager.map_to_parent_handle;
    const auto& parent_to_child_handle = c.map_handle;


    return std::array<attribute::MutableAccessor<int64_t>, 2>{
        {my_mesh.create_accessor(parent_to_child_handle),
         child_mesh.create_accessor(child_to_parent_handle)}};
}
std::array<attribute::ConstAccessor<int64_t>, 2> MultiMeshManager::get_map_const_accessors(
    const Mesh& my_mesh,
    const ChildData& c) const
{
    const Mesh& child_mesh = *c.mesh;
    const auto& child_to_parent_handle = child_mesh.m_multi_mesh_manager.map_to_parent_handle;
    const auto& parent_to_child_handle = c.map_handle;


    return std::array<attribute::ConstAccessor<int64_t>, 2>{
        {my_mesh.create_const_accessor(parent_to_child_handle),
         child_mesh.create_const_accessor(child_to_parent_handle)}};
}
std::string MultiMeshManager::child_to_parent_map_attribute_name()
{
    return "map_to_parent";
}

void MultiMeshManager::update_map_tuple_hashes(
    Mesh& my_mesh,
    PrimitiveType primitive_type,
    const std::vector<std::tuple<int64_t, std::vector<Tuple>>>& simplices_to_update,
    const std::vector<std::tuple<int64_t, std::array<int64_t, 2>>>& split_cell_maps)
{
    // logger().trace(
    //     "Update map on [{}] for {} (have {})",
    //     fmt::join(my_mesh.absolute_multi_mesh_id(), ","),
    //     primitive_type_name(primitive_type),
    //     simplices_to_update.size());
    //  for (const auto& [gid, tups] : simplices_to_update) {
    //      logger().trace(
    //          "[{}] Trying to update {}",
    //          fmt::join(my_mesh.absolute_multi_mesh_id(), ","),
    //          gid);
    //  }
    //   parent cells might have been destroyed
    //

    const PrimitiveType parent_primitive_type = my_mesh.top_simplex_type();

    auto parent_hash_accessor = my_mesh.get_const_cell_hash_accessor();
    auto parent_flag_accessor = my_mesh.get_const_flag_accessor(primitive_type);
    // auto& update_tuple = [&](const auto& flag_accessor, Tuple& t) -> bool {
    //     if(acc.index_access().
    // };


    // go over every child mesh and try to update their hashes
    for (auto& child_data : children()) {
        auto& child_mesh = *child_data.mesh;
        // ignore ones whos map are the wrong dimension
        if (child_mesh.top_simplex_type() != primitive_type) {
            continue;
        }
        // logger().trace(
        //     "[{}->{}] Doing a child mesh",
        //     fmt::join(my_mesh.absolute_multi_mesh_id(), ","),
        //     fmt::join(child_mesh.absolute_multi_mesh_id(), ","));
        //  get accessors to the maps
        auto maps = get_map_accessors(my_mesh, child_data);
        auto& [parent_to_child_accessor, child_to_parent_accessor] = maps;

        auto child_flag_accessor = child_mesh.get_const_flag_accessor(primitive_type);
        auto child_hash_accessor = child_mesh.get_const_cell_hash_accessor();


        std::vector<bool> is_gid_visited(my_mesh.capacity(primitive_type), false);
        for (const auto& [original_parent_gid, equivalent_parent_tuples] : simplices_to_update) {
            if (is_gid_visited.at(original_parent_gid)) {
                continue;
            } else {
                is_gid_visited[original_parent_gid] = true;
            }

            // logger.trace()(
            //     "[{}->{}] Trying to update {}",
            //     fmt::join(my_mesh.absolute_multi_mesh_id(), ","),
            //     fmt::join(child_mesh.absolute_multi_mesh_id(), ","),
            //     original_parent_gid);
            //  read off the original map's data
            auto parent_to_child_data = Mesh::get_index_access(parent_to_child_accessor)
                                            .const_vector_attribute(original_parent_gid);

            // read off the data in the Tuple format
            Tuple parent_tuple =
                wmtk::multimesh::utils::vector5_to_tuple(parent_to_child_data.head<5>());
            Tuple child_tuple =
                wmtk::multimesh::utils::vector5_to_tuple(parent_to_child_data.tail<5>());

            // If the parent tuple is valid, it means this parent-child pair has already been
            // handled, so we can skip it
            if (my_mesh.is_valid_slow(parent_tuple)) {
                continue;
            }
            // If the parent tuple is invalid then there was no map so we can try the next cell
            if (parent_tuple.is_null()) {
                continue;
            }

            // navigate parent_original_sharer -> parent_tuple
            // then take parent_new_sharer -> parent_tuple
            parent_tuple = my_mesh.resurrect_tuple(parent_tuple, parent_hash_accessor);
            child_tuple = child_mesh.resurrect_tuple(child_tuple, child_hash_accessor);

            // check if the map is handled in the ear case
            auto child_to_parent_data =
                child_to_parent_accessor.const_vector_attribute(child_tuple);
            Tuple parent_tuple_from_child_map =
                wmtk::multimesh::utils::vector5_to_tuple(child_to_parent_data.tail<5>());
            if (my_mesh.is_valid_slow(parent_tuple_from_child_map)) {
                continue;
            }
            // if the child simplex is deleted then we can skip it
            const char child_flag = child_flag_accessor.const_scalar_attribute(child_tuple);
            bool child_exists = 1 == (child_flag & 1);
            if (!child_exists) {
                logger().debug("child doesnt exist, skip!");
                continue;
            }
            std::vector<Tuple> equivalent_parent_tuples_good_hash = equivalent_parent_tuples;
            for (Tuple& t : equivalent_parent_tuples_good_hash) {
                t = my_mesh.resurrect_tuple(t, parent_hash_accessor);
            }

            // Find a valid representation of this simplex representation of the original tupl
            Tuple old_tuple;
            std::optional<Tuple> old_tuple_opt = find_tuple_from_gid(
                my_mesh,
                my_mesh.top_simplex_type(),
                equivalent_parent_tuples_good_hash,
                parent_tuple.m_global_cid);
            assert(old_tuple_opt.has_value());
            simplex::Simplex old_simplex(primitive_type, old_tuple_opt.value());

            std::optional<Tuple> new_parent_shared_opt = find_valid_tuple(
                my_mesh,
                old_simplex,
                original_parent_gid,
                equivalent_parent_tuples_good_hash,
                split_cell_maps);

            if (!new_parent_shared_opt.has_value()) {
                std::cout << "get skipped, someting is wrong?" << std::endl;
                continue;
            }
            assert(new_parent_shared_opt.has_value());

            Tuple new_parent_tuple_shared = new_parent_shared_opt.value();
            // logger().trace(
            //     "{} => {} ==> {}",
            //     wmtk::utils::TupleInspector::as_string(old_simplex.tuple()),
            //     wmtk::utils::TupleInspector::as_string(parent_tuple),
            //     wmtk::utils::TupleInspector::as_string(child_tuple));

            parent_tuple = wmtk::multimesh::utils::transport_tuple(
                old_simplex.tuple(),
                parent_tuple,
                my_mesh.top_simplex_type(),
                new_parent_tuple_shared,
                my_mesh.top_simplex_type());
            parent_tuple = my_mesh.resurrect_tuple(parent_tuple, parent_hash_accessor);
            assert(my_mesh.is_valid_slow(parent_tuple));
            assert(child_mesh.is_valid_slow(child_tuple));


            wmtk::multimesh::utils::symmetric_write_tuple_map_attributes(
                parent_to_child_accessor,
                child_to_parent_accessor,
                parent_tuple,
                child_tuple);
        }
    }
}
std::optional<Tuple> MultiMeshManager::find_valid_tuple(
    Mesh& my_mesh,
    const simplex::Simplex& old_simplex,
    const int64_t old_gid,
    const std::vector<Tuple>& equivalent_parent_tuples,
    const std::vector<std::tuple<int64_t, std::array<int64_t, 2>>>& split_cell_maps) const
{
    // if old gid was one of the originals then do tuple
    // otherwise just find some random tuple that still exists

    std::optional<Tuple> split_attempt = find_valid_tuple_from_split(
        my_mesh,
        old_simplex,
        old_gid,
        equivalent_parent_tuples,
        split_cell_maps);
    if (!split_attempt.has_value()) {
        split_attempt = find_valid_tuple_from_alternatives(
            my_mesh,
            my_mesh.top_simplex_type(),
            equivalent_parent_tuples);
    }

    return split_attempt;
}


std::optional<Tuple> MultiMeshManager::find_valid_tuple_from_alternatives(
    Mesh& my_mesh,
    PrimitiveType primitive_type,
    const std::vector<Tuple>& tuple_alternatives) const
{
    auto parent_flag_accessor = my_mesh.get_const_flag_accessor(primitive_type);
    // find a new sharer by finding a tuple that exists
    auto it = std::find_if(
        tuple_alternatives.begin(),
        tuple_alternatives.end(),
        [&](const Tuple& t) -> bool {
            return 1 == (Mesh::get_index_access(parent_flag_accessor)
                             .scalar_attribute(wmtk::utils::TupleInspector::global_cid(t)) &
                         1);
        });
    if (it != tuple_alternatives.end()) {
        return *it;
    } else {
        return std::optional<Tuple>{};
    }
}

std::optional<Tuple> MultiMeshManager::find_valid_tuple_from_split(
    Mesh& my_mesh,
    const simplex::Simplex& old_simplex,
    const int64_t old_simplex_gid,
    const std::vector<Tuple>& tuple_alternatives,
    const std::vector<std::tuple<int64_t, std::array<int64_t, 2>>>& split_cell_maps) const
{
    const Tuple& old_tuple = old_simplex.tuple();
    const PrimitiveType primitive_type = old_simplex.primitive_type();

    for (const auto& [old_cid, new_cids] : split_cell_maps) {
        if (old_cid != wmtk::utils::TupleInspector::global_cid(old_tuple)) {
            continue;
        }

        auto old_tuple_opt =
            find_tuple_from_gid(my_mesh, my_mesh.top_simplex_type(), tuple_alternatives, old_cid);
        assert(old_tuple_opt.has_value());

        const Tuple& old_cid_tuple = old_tuple_opt.value();
        for (const int64_t new_cid : new_cids) {
            // try seeing if we get the right gid by shoving in the new face id
            Tuple tuple(
                wmtk::utils::TupleInspector::local_vid(old_cid_tuple),
                wmtk::utils::TupleInspector::local_eid(old_cid_tuple),
                wmtk::utils::TupleInspector::local_fid(old_cid_tuple),
                new_cid,
                my_mesh.get_cell_hash_slow(new_cid));


            if (my_mesh.is_valid_slow(tuple) &&
                old_simplex_gid == my_mesh.id(tuple, primitive_type)) {
                return tuple;
            }
        }
    }
    return std::optional<Tuple>{};
}

std::optional<Tuple> MultiMeshManager::find_tuple_from_gid(
    const Mesh& my_mesh,
    PrimitiveType primitive_type,
    const std::vector<Tuple>& tuples,
    int64_t gid)
{
    // logger().trace("Finding gid {}", gid);

    auto it = std::find_if(tuples.begin(), tuples.end(), [&](const Tuple& t) -> bool {
        return (gid == my_mesh.id(t, primitive_type));
    });
    if (it == tuples.end()) {
        // logger().trace("failed to find tuple");
        return std::optional<Tuple>{};
    } else {
        // logger().trace("got tuple");
        return *it;
    }
}
int64_t MultiMeshManager::child_global_cid(
    const attribute::ConstAccessor<int64_t>& parent_to_child,
    int64_t parent_gid)
{
    // look at src/wmtk/multimesh/utils/tuple_map_attribute_io.cpp to see what index global_cid gets mapped to)
    // 5 is the size of a tuple is 5 longs, global_cid currently gets written to position 3
    return Mesh::get_index_access(parent_to_child).vector_attribute(parent_gid)(5 + 3);
}
int64_t MultiMeshManager::parent_global_cid(
    const attribute::ConstAccessor<int64_t>& child_to_parent,
    int64_t child_gid)
{
    // look at src/wmtk/multimesh/utils/tuple_map_attribute_io.cpp to see what index global_cid gets mapped to)
    // 5 is the size of a tuple is 5 longs, global_cid currently gets written to position 3
    return Mesh::get_index_access(child_to_parent).vector_attribute(child_gid)(5 + 3);
}

void MultiMeshManager::update_vertex_operation_hashes_internal(
    Mesh& m,
    const Tuple& vertex,
    Accessor<int64_t>& hash_accessor)
{
    const PrimitiveType pt = m.top_simplex_type();
    const simplex::SimplexCollection star =
        simplex::closed_star(m, simplex::Simplex::vertex(vertex));
    std::vector<Tuple> tuples_to_update;
    switch (pt) {
    case PrimitiveType::Vertex: {
        const auto star_vertices = star.simplex_vector(PrimitiveType::Vertex);
        tuples_to_update.reserve(star_vertices.size());
        for (const simplex::Simplex& s : star_vertices) {
            tuples_to_update.emplace_back(s.tuple());
        }
        break;
    }
    case PrimitiveType::Edge: {
        const auto star_edges = star.simplex_vector(PrimitiveType::Edge);
        tuples_to_update.reserve(star_edges.size());
        for (const simplex::Simplex& s : star_edges) {
            tuples_to_update.emplace_back(s.tuple());
        }
        break;
    }
    case PrimitiveType::Face: {
        const auto star_faces = star.simplex_vector(PrimitiveType::Face);
        tuples_to_update.reserve(star_faces.size());
        for (const simplex::Simplex& s : star_faces) {
            tuples_to_update.emplace_back(s.tuple());
        }
        break;
    }
    case PrimitiveType::Tetrahedron: {
        const auto star_tets = star.simplex_vector(PrimitiveType::Tetrahedron);
        tuples_to_update.reserve(star_tets.size());
        for (const simplex::Simplex& s : star_tets) {
            tuples_to_update.emplace_back(s.tuple());
        }
        break;
    }
    default:
    case PrimitiveType::HalfEdge: throw std::runtime_error("Invalid case");
    }


    m.update_cell_hashes(tuples_to_update, hash_accessor);

    // need to get a star with new hash, otherwise cannot get_simplices()
    auto vertex_new_hash = m.resurrect_tuple(vertex, hash_accessor);
    const simplex::SimplexCollection star_new_hash =
        simplex::closed_star(m, simplex::Simplex::vertex(vertex_new_hash));
    update_vertex_operation_multimesh_map_hash_internal(m, star_new_hash, hash_accessor);
}

void MultiMeshManager::update_vertex_operation_multimesh_map_hash_internal(
    Mesh& m,
    const simplex::SimplexCollection& vertex_closed_star,
    Accessor<int64_t>& parent_hash_accessor)
{
    auto& mm_manager = m.m_multi_mesh_manager;

    for (auto& child_data : mm_manager.children()) {
        auto& child_mesh = *child_data.mesh;
        auto child_mesh_pt = child_mesh.top_simplex_type();
        auto maps = mm_manager.get_map_accessors(m, child_data);
        auto& [parent_to_child_accessor, child_to_parent_accessor] = maps;

        const auto parent_simplices_to_update = vertex_closed_star.simplex_vector(child_mesh_pt);
        for (auto& s : parent_simplices_to_update) {
            auto s_tuple = s.tuple();
            auto [parent_tuple, child_tuple] =
                wmtk::multimesh::utils::read_tuple_map_attribute(parent_to_child_accessor, s_tuple);

            if (parent_tuple.is_null()) continue;

            // resurrect the parent tuple
            parent_tuple = m.resurrect_tuple(parent_tuple, parent_hash_accessor);

            // write back to map
            wmtk::multimesh::utils::symmetric_write_tuple_map_attributes(
                parent_to_child_accessor,
                child_to_parent_accessor,
                parent_tuple,
                child_tuple);
        }
    }
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
    auto child_to_parent_handle = child_mesh.get_attribute_handle<int64_t>(c_to_p_name, map_type);
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
            assert(child_mesh.is_valid_slow(child_tuple_from_child));
            assert(child_mesh.is_valid_slow(child_tuple_from_child));
            assert(my_mesh.is_valid_slow(parent_tuple_from_child));
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
        if (map_type == PrimitiveType::Face && my_mesh.top_simplex_type() == PrimitiveType::Face) {
            Tuple cur_child_tuple = child_tuple_from_child;
            Tuple cur_parent_tuple = parent_tuple_from_child;

            auto child_to_parent_accessor =
                child_mesh.create_const_accessor(child_to_parent_handle);
            for (int i = 0; i < 3; i++) {
                if (!child_mesh.is_boundary(cur_child_tuple, PrimitiveType::Edge)) {
                    assert(!my_mesh.is_boundary(cur_parent_tuple, PrimitiveType::Edge));

#ifndef NDEBUG
                    Tuple child_tuple_opp = child_mesh.switch_face(cur_child_tuple);
                    Tuple parent_tuple_opp = my_mesh.switch_face(cur_parent_tuple);
                    assert(
                        parent_tuple_opp == map_tuple_between_meshes(
                                                child_mesh,
                                                my_mesh,
                                                child_to_parent_accessor,
                                                child_tuple_opp));
#endif
                }
                cur_child_tuple = child_mesh.switch_edge(child_mesh.switch_vertex(cur_child_tuple));
                cur_parent_tuple = my_mesh.switch_edge(my_mesh.switch_vertex(cur_parent_tuple));
            }
        } else if (
            map_type == PrimitiveType::Edge && my_mesh.top_simplex_type() == PrimitiveType::Face) {
            if (!my_mesh.is_boundary(parent_tuple_from_child, PrimitiveType::Edge)) {
                auto parent_to_child_accessor =
                    my_mesh.create_const_accessor(parent_to_child_handle);
#ifndef NDEBUG
                const Tuple parent_tuple_opp = my_mesh.switch_face(parent_tuple_from_child);
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
    assert(parent.size() <= child.size());
#if !defined(NDEBUG)
    for (size_t j = 0; j < parent.size(); ++j) {
        assert(parent[j] == child[j]);
    }

#endif
    std::vector<int64_t> ret;
    std::copy(child.begin() + parent.size(), child.end(), std::back_inserter(ret));
    return ret;
}

void MultiMeshManager::serialize(MeshWriter& writer)
{
    for (const auto& c : m_children) {
        c.mesh->serialize(writer);
    }
}

} // namespace wmtk
