#include "Mesh.hpp"
#include <numeric>

#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/io/MeshWriter.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/vector_hash.hpp>

#include "Primitive.hpp"

namespace wmtk {


PrimitiveType Mesh::top_simplex_type() const
{
    long dimension = top_cell_dimension();
    assert(dimension >= 0);
    assert(dimension < 4);
    return static_cast<PrimitiveType>(dimension);
}


std::vector<Tuple> Mesh::get_all(PrimitiveType type) const
{
    return get_all(type, false);
}

std::tuple<std::vector<std::vector<long>>, std::vector<std::vector<long>>> Mesh::consolidate()
{
    // Number of dimensions
    long tcp = top_cell_dimension() + 1;

    // Store the map from new indices to old. First index is dimensions, second simplex id
    std::vector<std::vector<long>> new2old(tcp);
    // Store the map from old indices to new. First index is dimensions, second simplex id
    std::vector<std::vector<long>> old2new(tcp);

    // Initialize both maps
    for (long d = 0; d < tcp; d++) {
        Accessor<char> flag_accessor = get_flag_accessor(wmtk::get_primitive_type_from_id(d));
        for (long i = 0; i < capacity(wmtk::get_primitive_type_from_id(d)); ++i) {
            if (flag_accessor.index_access().scalar_attribute(i) & 1) {
                old2new[d].push_back(new2old[d].size());
                new2old[d].push_back(old2new[d].size() - 1); // -1 since we just pushed into it
            } else {
                old2new[d].push_back(-1);
            }
        }
    }

    // Use new2oldmap to compact all attributes
    for (long d = 0; d < tcp; d++) {
        attribute::MeshAttributes<char>& attributesc = m_attribute_manager.m_char_attributes[d];
        for (auto h = attributesc.m_attributes.begin(); h != attributesc.m_attributes.end(); h++)
            h->consolidate(new2old[d]);

        attribute::MeshAttributes<long>& attributesl = m_attribute_manager.m_long_attributes[d];
        for (auto h = attributesl.m_attributes.begin(); h != attributesl.m_attributes.end(); h++)
            h->consolidate(new2old[d]);

        attribute::MeshAttributes<double>& attributesd = m_attribute_manager.m_double_attributes[d];
        for (auto h = attributesd.m_attributes.begin(); h != attributesd.m_attributes.end(); h++)
            h->consolidate(new2old[d]);

        attribute::MeshAttributes<Rational>& attributesr =
            m_attribute_manager.m_rational_attributes[d];
        for (auto h = attributesr.m_attributes.begin(); h != attributesr.m_attributes.end(); h++)
            h->consolidate(new2old[d]);
    }

    // Update the attribute size in the manager
    for (long d = 0; d < tcp; d++) m_attribute_manager.m_capacities[d] = new2old[d].size();

    // Apply old2new to attributes containing indices
    std::vector<std::vector<TypedAttributeHandle<long>>> handle_indices = connectivity_attributes();

    for (long d = 0; d < tcp; d++) {
        for (long i = 0; i < handle_indices[d].size(); ++i) {
            Accessor<long> accessor = create_accessor<long>(handle_indices[d][i]);
            accessor.attribute().index_remap(old2new[d]);
        }
    }
    // Return both maps for custom attribute remapping
    return {new2old, old2new};
}

std::vector<Tuple> Mesh::get_all(PrimitiveType type, const bool include_deleted) const
{
    ConstAccessor<char> flag_accessor = get_flag_accessor(type);
    const attribute::CachingAccessor<char>& flag_accessor_indices = flag_accessor.index_access();
    std::vector<Tuple> ret;
    long cap = capacity(type);
    ret.reserve(cap);
    for (size_t index = 0; index < cap; ++index) {
        if (flag_accessor_indices.const_scalar_attribute(index) & 1)
            ret.emplace_back(tuple_from_id(type, index));
        else if (include_deleted)
            ret.emplace_back();
    }
    return ret;
}

void Mesh::serialize(MeshWriter& writer)
{
    writer.write_top_simplex_type(top_simplex_type());
    m_attribute_manager.serialize(writer);
}


bool Mesh::is_boundary(const Tuple& tuple) const
{
    long my_dim = top_cell_dimension() - 1;
    PrimitiveType pt = static_cast<PrimitiveType>(my_dim);
    return is_boundary(tuple, pt);
}

bool Mesh::is_boundary(const Simplex& s) const
{
    return is_boundary(s.tuple(), s.primitive_type());
}


bool Mesh::is_hash_valid(const Tuple& tuple, const ConstAccessor<long>& hash_accessor) const
{
    const long cid = tuple.m_global_cid;
    return tuple.m_hash == get_cell_hash(cid, hash_accessor);
}

bool Mesh::is_valid_slow(const Tuple& tuple) const
{
    ConstAccessor<long> hash_accessor = get_const_cell_hash_accessor();
    return is_valid(tuple, hash_accessor);
}


ConstAccessor<char> Mesh::get_flag_accessor(PrimitiveType type) const
{
    return get_const_flag_accessor(type);
}
ConstAccessor<char> Mesh::get_const_flag_accessor(PrimitiveType type) const
{
    return create_const_accessor(m_flag_handles.at(get_primitive_type_id(type)));
}
Accessor<char> Mesh::get_flag_accessor(PrimitiveType type)
{
    return create_accessor(m_flag_handles.at(get_primitive_type_id(type)));
}

ConstAccessor<long> Mesh::get_const_cell_hash_accessor() const
{
    return create_const_accessor(m_cell_hash_handle);
}

ConstAccessor<long> Mesh::get_cell_hash_accessor() const
{
    return get_const_cell_hash_accessor();
}
Accessor<long> Mesh::get_cell_hash_accessor()
{
    return create_accessor(m_cell_hash_handle);
}

void Mesh::update_cell_hash(const Tuple& cell, Accessor<long>& hash_accessor)
{
    const long cid = cell.m_global_cid;
    update_cell_hash(cid, hash_accessor);
}
void Mesh::update_cell_hash(const long cid, Accessor<long>& hash_accessor)
{
    ++hash_accessor.index_access().scalar_attribute(cid);
}

void Mesh::update_cell_hashes(const std::vector<Tuple>& cells, Accessor<long>& hash_accessor)
{
    for (const Tuple& t : cells) {
        update_cell_hash(t, hash_accessor);
    }
}
void Mesh::update_cell_hashes(const std::vector<long>& cells, Accessor<long>& hash_accessor)
{
    for (const long t : cells) {
        update_cell_hash(t, hash_accessor);
    }
}

void Mesh::update_cell_hashes_slow(const std::vector<Tuple>& cells)
{
    Accessor<long> hash_accessor = get_cell_hash_accessor();
    update_cell_hashes(cells, hash_accessor);
}


Tuple Mesh::resurrect_tuple(const Tuple& tuple, const ConstAccessor<long>& hash_accessor) const
{
    Tuple t = tuple;
    t.m_hash = get_cell_hash(tuple.m_global_cid, hash_accessor);
    return t;
}

Tuple Mesh::resurrect_tuple_slow(const Tuple& tuple)
{
    Accessor<long> hash_accessor = get_cell_hash_accessor();
    return resurrect_tuple(tuple, hash_accessor);
}

long Mesh::get_cell_hash(long cell_index, const ConstAccessor<long>& hash_accessor) const
{
    return hash_accessor.index_access().const_scalar_attribute(cell_index);
}

long Mesh::get_cell_hash_slow(long cell_index) const
{
    ConstAccessor<long> hash_accessor = get_cell_hash_accessor();
    return get_cell_hash(cell_index, hash_accessor);
}

void Mesh::set_capacities_from_flags()
{
    for (long dim = 0; dim < m_attribute_manager.m_capacities.size(); ++dim) {
        Accessor<char> flag_accessor = create_accessor<char>(m_flag_handles[dim]);
        m_attribute_manager.m_capacities[dim] = flag_accessor.reserved_size();
    }
}

bool Mesh::operator==(const Mesh& other) const
{
    return m_attribute_manager == other.m_attribute_manager;
}


std::vector<std::vector<long>> Mesh::simplices_to_gids(
    const std::vector<std::vector<Simplex>>& simplices) const
{
    std::vector<std::vector<long>> gids;
    gids.resize(simplices.size());
    for (int i = 0; i < simplices.size(); ++i) {
        auto simplices_i = simplices[i];
        for (auto simplex : simplices_i) {
            long d = get_primitive_type_id(simplex.primitive_type());
            assert(d < 3);
            gids[d].emplace_back(id(simplex.tuple(), simplex.primitive_type()));
        }
    }
    return gids;
}

multimesh::attribute::AttributeScopeHandle Mesh::create_scope()
{
    return multimesh::attribute::AttributeScopeHandle(*this);
}

attribute::AttributeScopeHandle Mesh::create_single_mesh_scope()
{
    return m_attribute_manager.create_scope(*this);
}


Tuple Mesh::switch_tuples(
    const Tuple& tuple,
    const std::initializer_list<PrimitiveType>& op_sequence) const
{
    return switch_tuples<std::initializer_list<PrimitiveType>>(tuple, op_sequence);
}
Tuple Mesh::switch_tuples_unsafe(
    const Tuple& tuple,
    const std::initializer_list<PrimitiveType>& op_sequence) const
{
    return switch_tuples_unsafe<std::initializer_list<PrimitiveType>>(tuple, op_sequence);
}


std::vector<long> Mesh::absolute_multi_mesh_id() const
{
    return m_multi_mesh_manager.absolute_id();
}
void Mesh::register_child_mesh(
    const std::shared_ptr<Mesh>& child_mesh_ptr,
    const std::vector<std::array<Tuple, 2>>& map_tuples)
{
    m_multi_mesh_manager.register_child_mesh(*this, child_mesh_ptr, map_tuples);
}


bool Mesh::is_from_same_multi_mesh_structure(const Mesh& other) const
{
    return &get_multi_mesh_root() == &other.get_multi_mesh_root();
}

std::vector<Simplex> Mesh::map(const Mesh& other_mesh, const Simplex& my_simplex) const
{
    if (!is_from_same_multi_mesh_structure(other_mesh)) {
        throw std::runtime_error(
            "Attempted to map between two simplices in different multi-mesh structures");
    }
    return m_multi_mesh_manager.map(*this, other_mesh, my_simplex);
}

std::vector<Simplex> Mesh::map(const Mesh& other_mesh, const std::vector<Simplex>& simplices) const
{
    std::vector<Simplex> ret;
    ret.reserve(simplices.size());
    for (const auto& s : simplices) {
        auto v = map(other_mesh, s);
        ret.insert(ret.end(), v.begin(), v.end());
    }

    return ret;
}

std::vector<Simplex> Mesh::lub_map(const Mesh& other_mesh, const Simplex& my_simplex) const
{
    if (!is_from_same_multi_mesh_structure(other_mesh)) {
        throw std::runtime_error(
            "Attempted to map between two simplices in different multi-mesh structures");
    }
    return m_multi_mesh_manager.lub_map(*this, other_mesh, my_simplex);
}

std::vector<Simplex> Mesh::lub_map(const Mesh& other_mesh, const std::vector<Simplex>& simplices)
    const
{
    std::vector<Simplex> ret;
    ret.reserve(simplices.size());
    for (const auto& s : simplices) {
        auto v = lub_map(other_mesh, s);
        ret.insert(ret.end(), v.begin(), v.end());
    }

    return ret;
}


Simplex Mesh::map_to_parent(const Simplex& my_simplex) const
{
    if (is_multi_mesh_root()) {
        throw std::runtime_error("Attempted to map a simplex to parent despite being a root");
    }
    return m_multi_mesh_manager.map_to_parent(*this, my_simplex);
}
Simplex Mesh::map_to_root(const Simplex& my_simplex) const
{
    return m_multi_mesh_manager.map_to_root(*this, my_simplex);
}

std::vector<Simplex> Mesh::map_to_child(const Mesh& child_mesh, const Simplex& my_simplex) const
{
    if (!is_from_same_multi_mesh_structure(child_mesh)) {
        throw std::runtime_error(
            "Attempted to map between two simplices in different multi-mesh structures");
    }
    return m_multi_mesh_manager.map_to_child(*this, child_mesh, my_simplex);
}

std::vector<Tuple> Mesh::map_tuples(const Mesh& other_mesh, const Simplex& my_simplex) const
{
    if (!is_from_same_multi_mesh_structure(other_mesh)) {
        throw std::runtime_error(
            "Attempted to map between two simplices in different multi-mesh structures");
    }
    return m_multi_mesh_manager.map_tuples(*this, other_mesh, my_simplex);
}

std::vector<Tuple>
Mesh::map_tuples(const Mesh& other_mesh, PrimitiveType pt, const std::vector<Tuple>& tuples) const
{
    std::vector<Tuple> ret;
    ret.reserve(tuples.size());
    for (const auto& t : tuples) {
        auto v = map_tuples(other_mesh, Simplex(pt, t));
        ret.insert(ret.end(), v.begin(), v.end());
    }

    return ret;
}
std::vector<Tuple> Mesh::lub_map_tuples(const Mesh& other_mesh, const Simplex& my_simplex) const
{
    if (!is_from_same_multi_mesh_structure(other_mesh)) {
        throw std::runtime_error(
            "Attempted to map between two simplices in different multi-mesh structures");
    }
    return m_multi_mesh_manager.lub_map_tuples(*this, other_mesh, my_simplex);
}

std::vector<Tuple> Mesh::lub_map_tuples(
    const Mesh& other_mesh,
    PrimitiveType pt,
    const std::vector<Tuple>& tuples) const
{
    std::vector<Tuple> ret;
    ret.reserve(tuples.size());
    for (const auto& t : tuples) {
        auto v = lub_map_tuples(other_mesh, Simplex(pt, t));
        ret.insert(ret.end(), v.begin(), v.end());
    }

    return ret;
}

Tuple Mesh::map_to_parent_tuple(const Simplex& my_simplex) const
{
    if (is_multi_mesh_root()) {
        throw std::runtime_error("Attempted to map a simplex to parent despite being a root");
    }
    return m_multi_mesh_manager.map_to_parent_tuple(*this, my_simplex);
}
Tuple Mesh::map_to_root_tuple(const Simplex& my_simplex) const
{
    return m_multi_mesh_manager.map_to_root_tuple(*this, my_simplex);
}
std::vector<Tuple> Mesh::map_to_child_tuples(const Mesh& child_mesh, const Simplex& my_simplex)
    const
{
    if (!is_from_same_multi_mesh_structure(child_mesh)) {
        throw std::runtime_error(
            "Attempted to map between two simplices in different multi-mesh structures");
    }
    return m_multi_mesh_manager.map_to_child_tuples(*this, child_mesh, my_simplex);
}

bool Mesh::is_multi_mesh_root() const
{
    return m_multi_mesh_manager.is_root();
}
Mesh& Mesh::get_multi_mesh_root()
{
    return m_multi_mesh_manager.get_root_mesh(*this);
}
const Mesh& Mesh::get_multi_mesh_root() const
{
    return m_multi_mesh_manager.get_root_mesh(*this);
}

std::vector<std::shared_ptr<Mesh>> Mesh::get_child_meshes() const
{
    return m_multi_mesh_manager.get_child_meshes();
}

// reserves extra attributes than necessary right now
void Mesh::reserve_more_attributes(PrimitiveType type, long size)
{
    m_attribute_manager.reserve_more_attributes(get_primitive_type_id(type), size);
}
// reserves extra attributes than necessary right now
void Mesh::reserve_more_attributes(const std::vector<long>& sizes)
{
    assert(top_cell_dimension() + 1 == sizes.size());
    for (long j = 0; j < sizes.size(); ++j) {
        m_attribute_manager.reserve_more_attributes(j, sizes[j]);
    }
}

void Mesh::update_vertex_operation_hashes(const Tuple& vertex, Accessor<long>& hash_accessor)
{
    MultiMeshManager::update_vertex_operation_hashes_internal(*this, vertex, hash_accessor);
}


} // namespace wmtk
