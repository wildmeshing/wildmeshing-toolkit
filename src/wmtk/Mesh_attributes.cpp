#include <numeric>
#include "Mesh.hpp"

#include <wmtk/utils/Logger.hpp>

#include "Primitive.hpp"

namespace wmtk {
template <typename T>
attribute::MeshAttributeHandle Mesh::register_attribute(
    const std::string& name,
    PrimitiveType ptype,
    int64_t size,
    bool replace,
    T default_value)
{
    attribute::MeshAttributeHandle attr(
        *this,
        register_attribute_typed<T>(name, ptype, size, replace, default_value));
    reload_accessors();
    return attr;
}

template <typename T>
attribute::TypedAttributeHandle<T> Mesh::register_attribute_typed(
    const std::string& name,
    PrimitiveType ptype,
    int64_t size,
    bool replace,
    T default_value)
{
    auto attr = m_attribute_manager.register_attribute<T>(name, ptype, size, replace, default_value);
    reload_accessors();
    return attr;
}

std::vector<int64_t> Mesh::request_simplex_indices(PrimitiveType type, int64_t count)
{
    // passses back a set of new consecutive ids. in hte future this could do
    // something smarter for re-use but that's probably too much work
    int64_t current_capacity = capacity(type);

    // enable newly requested simplices
    Accessor<char> flag_accessor = get_flag_accessor(type);
    int64_t max_size = flag_accessor.reserved_size();

    if (current_capacity + count > max_size) {
        logger().warn(
            "Requested more {} simplices than available (have {}, wanted {}, can only have at most "
            "{}",
            primitive_type_name(type),
            current_capacity,
            count,
            max_size);
        return {};
    }

    std::vector<int64_t> ret(count);
    std::iota(ret.begin(), ret.end(), current_capacity);


    int64_t new_capacity = current_capacity + ret.size();
    assert(ret.back() + 1 == current_capacity + ret.size());
    size_t primitive_id = get_primitive_type_id(type);

    m_attribute_manager.m_capacities[primitive_id] = new_capacity;

    attribute::CachingAccessor<char>& flag_accessor_indices = flag_accessor.index_access();

    for (const int64_t simplex_index : ret) {
        flag_accessor_indices.scalar_attribute(simplex_index) |= 0x1;
    }

    return ret;
}

int64_t Mesh::capacity(PrimitiveType type) const
{
    return m_attribute_manager.m_capacities.at(get_primitive_type_id(type));
}

void Mesh::reserve_attributes_to_fit()
{
    m_attribute_manager.reserve_to_fit();
}
void Mesh::reserve_attributes(PrimitiveType type, int64_t size)
{
    m_attribute_manager.reserve_attributes(get_primitive_type_id(type), size);
}
void Mesh::set_capacities(std::vector<int64_t> capacities)
{
    m_attribute_manager.set_capacities(std::move(capacities));
}

// reserves extra attributes than necessary right now
void Mesh::reserve_more_attributes(PrimitiveType type, int64_t size)
{
    m_attribute_manager.reserve_more_attributes(get_primitive_type_id(type), size);
}
// reserves extra attributes than necessary right now
void Mesh::reserve_more_attributes(const std::vector<int64_t>& sizes)
{
    assert(top_cell_dimension() + 1 == sizes.size());
    for (int64_t j = 0; j < sizes.size(); ++j) {
        m_attribute_manager.reserve_more_attributes(j, sizes[j]);
    }
}
void Mesh::guarantee_at_least_attributes(PrimitiveType type, int64_t size)
{
    m_attribute_manager.guarantee_at_least_attributes(get_primitive_type_id(type), size);
}
void Mesh::guarantee_at_least_attributes(const std::vector<int64_t>& sizes)
{
    assert(top_cell_dimension() + 1 == sizes.size());
    for (int64_t j = 0; j < sizes.size(); ++j) {
        m_attribute_manager.guarantee_at_least_attributes(j, sizes[j]);
    }
}
void Mesh::guarantee_more_attributes(PrimitiveType type, int64_t size)
{
    m_attribute_manager.guarantee_more_attributes(get_primitive_type_id(type), size);
}
void Mesh::guarantee_more_attributes(const std::vector<int64_t>& sizes)
{
    assert(top_cell_dimension() + 1 == sizes.size());
    for (int64_t j = 0; j < sizes.size(); ++j) {
        m_attribute_manager.guarantee_more_attributes(j, sizes[j]);
    }
}

namespace {
std::vector<attribute::TypedAttributeHandleVariant> variant_diff(
    std::vector<attribute::TypedAttributeHandleVariant>& a,
    std::vector<attribute::TypedAttributeHandleVariant>& b)
{
    std::vector<attribute::TypedAttributeHandleVariant> ret;
    std::sort(a.begin(), a.end());

    std::sort(b.begin(), b.end());

    std::set_difference(a.begin(), a.end(), b.begin(), b.end(), std::inserter(ret, ret.begin()));
    return ret;
}
} // namespace
std::vector<attribute::TypedAttributeHandleVariant> Mesh::custom_attributes() const
{
    auto all = m_attribute_manager.get_all_attributes();
    auto builtins = builtin_attributes();

    return variant_diff(all, builtins);
}

std::string Mesh::get_attribute_name(const attribute::TypedAttributeHandleVariant& handle) const
{
    return m_attribute_manager.get_name(handle);
}
void Mesh::clear_attributes()
{
    m_attribute_manager.clear_attributes(custom_attributes());
}

void Mesh::clear_attributes(
    const std::vector<attribute::TypedAttributeHandleVariant>& keep_attributes)
{
    auto a = this->custom_attributes();
    auto b = keep_attributes;
    m_attribute_manager.clear_attributes(variant_diff(a, b));
}
void Mesh::clear_attributes(const std::vector<attribute::MeshAttributeHandle>& keep_attributes)
{
    std::map<Mesh*, std::vector<attribute::TypedAttributeHandleVariant>> keeps_t;
    for (const auto& attr : keep_attributes) {
        keeps_t[const_cast<Mesh*>(&attr.mesh())].emplace_back(attr.handle());
    }
    for (auto& [mptr, handles] : keeps_t) {
        mptr->clear_attributes(handles);
    }
}

multimesh::attribute::AttributeScopeHandle Mesh::create_scope()
{
    return multimesh::attribute::AttributeScopeHandle(*this);
}

attribute::AttributeScopeHandle Mesh::create_single_mesh_scope()
{
    return m_attribute_manager.create_scope(*this);
}

std::tuple<std::vector<std::vector<int64_t>>, std::vector<std::vector<int64_t>>> Mesh::consolidate()
{
    // Number of dimensions
    int64_t tcp = top_cell_dimension() + 1;

    // Store the map from new indices to old. First index is dimensions, second simplex id
    std::vector<std::vector<int64_t>> new2old(tcp);
    // Store the map from old indices to new. First index is dimensions, second simplex id
    std::vector<std::vector<int64_t>> old2new(tcp);

    // Initialize both maps
    for (int64_t d = 0; d < tcp; d++) {
        Accessor<char> flag_accessor = get_flag_accessor(wmtk::get_primitive_type_from_id(d));
        for (int64_t i = 0; i < capacity(wmtk::get_primitive_type_from_id(d)); ++i) {
            if (flag_accessor.index_access().scalar_attribute(i) & 1) {
                old2new[d].push_back(new2old[d].size());
                new2old[d].push_back(old2new[d].size() - 1); // -1 since we just pushed into it
            } else {
                old2new[d].push_back(-1);
            }
        }
    }

    // Use new2oldmap to compact all attributes
    for (int64_t d = 0; d < tcp; d++) {
        attribute::MeshAttributes<char>& attributesc = m_attribute_manager.m_char_attributes[d];
        for (auto h = attributesc.m_attributes.begin(); h != attributesc.m_attributes.end(); h++)
            h->consolidate(new2old[d]);

        attribute::MeshAttributes<int64_t>& attributesl = m_attribute_manager.m_long_attributes[d];
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
    for (int64_t d = 0; d < tcp; d++) m_attribute_manager.m_capacities[d] = new2old[d].size();

    // Apply old2new to attributes containing indices
    std::vector<std::vector<TypedAttributeHandle<int64_t>>> handle_indices =
        connectivity_attributes();

    for (int64_t d = 0; d < tcp; d++) {
        for (int64_t i = 0; i < handle_indices[d].size(); ++i) {
            Accessor<int64_t> accessor = create_accessor<int64_t>(handle_indices[d][i]);
            accessor.attribute().index_remap(old2new[d]);
        }
    }
    // Return both maps for custom attribute remapping
    return {new2old, old2new};
}
std::vector<attribute::TypedAttributeHandleVariant> Mesh::builtin_attributes() const
{
    std::vector<attribute::TypedAttributeHandleVariant> data;
    for (const auto& vec : connectivity_attributes()) {
        std::copy(vec.begin(), vec.end(), std::back_inserter(data));
    }

    data.emplace_back(m_cell_hash_handle);
    std::copy(m_flag_handles.begin(), m_flag_handles.end(), std::back_inserter(data));

    auto mm_handles = m_multi_mesh_manager.map_handles();
    std::copy(mm_handles.begin(), mm_handles.end(), std::back_inserter(data));
    return data;
}


template wmtk::attribute::MeshAttributeHandle
Mesh::register_attribute<char>(const std::string&, PrimitiveType, int64_t, bool, char);
template wmtk::attribute::MeshAttributeHandle
Mesh::register_attribute<int64_t>(const std::string&, PrimitiveType, int64_t, bool, int64_t);
template wmtk::attribute::MeshAttributeHandle
Mesh::register_attribute<double>(const std::string&, PrimitiveType, int64_t, bool, double);
template wmtk::attribute::MeshAttributeHandle
Mesh::register_attribute<Rational>(const std::string&, PrimitiveType, int64_t, bool, Rational);


template TypedAttributeHandle<char>
Mesh::register_attribute_typed(const std::string&, PrimitiveType, int64_t, bool, char);
template TypedAttributeHandle<int64_t>
Mesh::register_attribute_typed(const std::string&, PrimitiveType, int64_t, bool, int64_t);
template TypedAttributeHandle<double>
Mesh::register_attribute_typed(const std::string&, PrimitiveType, int64_t, bool, double);
template TypedAttributeHandle<Rational>
Mesh::register_attribute_typed(const std::string&, PrimitiveType, int64_t, bool, Rational);
} // namespace wmtk
