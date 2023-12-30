#include <numeric>
#include "Mesh.hpp"
#include "TriMesh.hpp"

#include <wmtk/operations/CollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/SplitNewAttributeStrategy.hpp>

#include <wmtk/operations/tri_mesh/BasicCollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/tri_mesh/BasicSplitNewAttributeStrategy.hpp>
#include <wmtk/operations/tri_mesh/PredicateAwareCollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/tri_mesh/PredicateAwareSplitNewAttributeStrategy.hpp>
#include <wmtk/utils/Logger.hpp>

#include "Primitive.hpp"

namespace wmtk {
template <typename T>
attribute::AttributeInitializationHandle<T> Mesh::register_attribute(
    const std::string& name,
    PrimitiveType ptype,
    int64_t size,
    bool replace,
    T default_value)
{
    MeshAttributeHandle<T> attr(
        *this,
        register_attribute_nomesh(name, ptype, size, replace, default_value, true));

    return add_new_attribute_strategy(attr);
}

template <typename T>
[[nodiscard]] attribute::AttributeInitializationHandle<T> Mesh::add_new_attribute_strategy(
    const MeshAttributeHandle<T>& attr)
{
    return attribute::AttributeInitializationHandle<T>(attr);
}
void Mesh::clear_new_attribute_strategies()
{
    m_transfer_strategies.clear();
}

template <typename T>
TypedAttributeHandle<T> Mesh::register_attribute_nomesh(
    const std::string& name,
    PrimitiveType ptype,
    int64_t size,
    bool replace,
    T default_value,
    bool is_custom)
{
    return m_attribute_manager
        .register_attribute<T>(*this, name, ptype, size, replace, default_value, is_custom);
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

template wmtk::attribute::AttributeInitializationHandle<char>
Mesh::register_attribute(const std::string&, PrimitiveType, int64_t, bool, char);
template wmtk::attribute::AttributeInitializationHandle<int64_t>
Mesh::register_attribute(const std::string&, PrimitiveType, int64_t, bool, int64_t);
template wmtk::attribute::AttributeInitializationHandle<double>
Mesh::register_attribute(const std::string&, PrimitiveType, int64_t, bool, double);
template wmtk::attribute::AttributeInitializationHandle<Rational>
Mesh::register_attribute(const std::string&, PrimitiveType, int64_t, bool, Rational);

template TypedAttributeHandle<char>
Mesh::register_attribute_nomesh(const std::string&, PrimitiveType, int64_t, bool, char, bool);
template TypedAttributeHandle<int64_t>
Mesh::register_attribute_nomesh(const std::string&, PrimitiveType, int64_t, bool, long, bool);
template TypedAttributeHandle<double>
Mesh::register_attribute_nomesh(const std::string&, PrimitiveType, int64_t, bool, double, bool);
template TypedAttributeHandle<Rational>
Mesh::register_attribute_nomesh(const std::string&, PrimitiveType, int64_t, bool, Rational, bool);
} // namespace wmtk
