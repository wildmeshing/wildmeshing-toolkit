#include <numeric>
#include "Mesh.hpp"
#include "TriMesh.hpp"

#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/operations/CollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/SplitNewAttributeStrategy.hpp>

#include <wmtk/operations/tri_mesh/BasicCollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/tri_mesh/BasicSplitNewAttributeStrategy.hpp>
#include <wmtk/utils/Logger.hpp>

#include "Primitive.hpp"

namespace wmtk {
template <typename T>
attribute::AttributeInitializationHandle<T> Mesh::register_attribute(
    const std::string& name,
    PrimitiveType ptype,
    long size,
    bool replace,
    T default_value)
{
    MeshAttributeHandle<T> attr(
        *this,
        register_attribute_nomesh(name, ptype, size, replace, default_value));

    std::shared_ptr<operations::SplitNewAttributeStrategy> split_ptr;
    std::shared_ptr<operations::CollapseNewAttributeStrategy> collapse_ptr;
    if (top_cell_dimension() == 2) {
        split_ptr = std::make_shared<operations::tri_mesh::BasicSplitNewAttributeStrategy<T>>(attr);
        collapse_ptr =
            std::make_shared<operations::tri_mesh::BasicCollapseNewAttributeStrategy<T>>(attr);
        m_split_strategies.emplace_back(split_ptr);
        m_collapse_strategies.emplace_back(collapse_ptr);
    }


    return attribute::AttributeInitializationHandle<T>(attr, split_ptr, collapse_ptr);
}

template <typename T>
TypedAttributeHandle<T> Mesh::register_attribute_nomesh(
    const std::string& name,
    PrimitiveType ptype,
    long size,
    bool replace,
    T default_value)
{
    return m_attribute_manager.register_attribute<T>(name, ptype, size, replace, default_value);
}

std::vector<long> Mesh::request_simplex_indices(PrimitiveType type, long count)
{
    // passses back a set of new consecutive ids. in hte future this could do
    // something smarter for re-use but that's probably too much work
    long current_capacity = capacity(type);

    // enable newly requested simplices
    Accessor<char> flag_accessor = get_flag_accessor(type);
    long max_size = flag_accessor.reserved_size();

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

    std::vector<long> ret(count);
    std::iota(ret.begin(), ret.end(), current_capacity);


    long new_capacity = ret.back() + 1;
    size_t primitive_id = get_primitive_type_id(type);

    m_attribute_manager.m_capacities[primitive_id] = new_capacity;

    attribute::CachingAccessor<char>& flag_accessor_indices = flag_accessor.index_access();

    for (const long simplex_index : ret) {
        flag_accessor_indices.scalar_attribute(simplex_index) |= 0x1;
    }

    return ret;
}

long Mesh::capacity(PrimitiveType type) const
{
    return m_attribute_manager.m_capacities.at(get_primitive_type_id(type));
}

void Mesh::reserve_attributes_to_fit()
{
    m_attribute_manager.reserve_to_fit();
}
void Mesh::reserve_attributes(PrimitiveType type, long size)
{
    m_attribute_manager.reserve_attributes(get_primitive_type_id(type), size);
}
void Mesh::set_capacities(std::vector<long> capacities)
{
    m_attribute_manager.set_capacities(std::move(capacities));
}

template wmtk::attribute::AttributeInitializationHandle<char>
Mesh::register_attribute(const std::string&, PrimitiveType, long, bool, char);
template wmtk::attribute::AttributeInitializationHandle<long>
Mesh::register_attribute(const std::string&, PrimitiveType, long, bool, long);
template wmtk::attribute::AttributeInitializationHandle<double>
Mesh::register_attribute(const std::string&, PrimitiveType, long, bool, double);
template wmtk::attribute::AttributeInitializationHandle<Rational>
Mesh::register_attribute(const std::string&, PrimitiveType, long, bool, Rational);

template TypedAttributeHandle<char>
Mesh::register_attribute_nomesh(const std::string&, PrimitiveType, long, bool, char);
template TypedAttributeHandle<long>
Mesh::register_attribute_nomesh(const std::string&, PrimitiveType, long, bool, long);
template TypedAttributeHandle<double>
Mesh::register_attribute_nomesh(const std::string&, PrimitiveType, long, bool, double);
template TypedAttributeHandle<Rational>
Mesh::register_attribute_nomesh(const std::string&, PrimitiveType, long, bool, Rational);
} // namespace wmtk
