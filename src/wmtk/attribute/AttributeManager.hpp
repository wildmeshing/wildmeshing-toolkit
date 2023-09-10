#pragma once

#include <vector>
#include <wmtk/utils/Rational.hpp>
#include "AttributeScopeHandle.hpp"
#include "MeshAttributes.hpp"

namespace wmtk {
class Mesh;
class MeshWriter;

namespace attribute {
template <typename T>
class MeshAttributes;
struct AttributeManager
{
    AttributeManager(long size);
    ~AttributeManager();
    AttributeManager(const AttributeManager& o);
    AttributeManager(AttributeManager&& o);
    AttributeManager& operator=(const AttributeManager& o);
    AttributeManager& operator=(AttributeManager&& o);

    //=========================================================
    // Storage of Mesh Attributes
    //=========================================================
    std::vector<MeshAttributes<char>> m_char_attributes;
    std::vector<MeshAttributes<long>> m_long_attributes;
    std::vector<MeshAttributes<double>> m_double_attributes;
    std::vector<MeshAttributes<Rational>> m_rational_attributes;


    // max index used for each type of simplex
    std::vector<long> m_capacities;

    // the number of types of attributes (types of simplex)
    long size() const;

    AttributeScopeHandle create_scope(Mesh& m);
    void serialize(MeshWriter& writer);
    void reserve_to_fit();
    void reserve_attributes_to_fit();
    void reserve_attributes(long dimension, long size);
    // specifies the number of simplices of each type and resizes attributes appropritely
    void set_capacities(std::vector<long> capacities);
    bool operator==(const AttributeManager& other) const;
    template <typename T>
    MeshAttributeHandle<T> register_attribute(
        const std::string& name,
        PrimitiveType type,
        long size,
        bool replace = false);
    template <typename T>
    MeshAttributes<T>& get(PrimitiveType ptype);

    template <typename T>
    MeshAttributes<T>& get(const MeshAttributeHandle<T>& handle);

    template <typename T>
    const MeshAttributes<T>& get(PrimitiveType ptype) const;

    template <typename T>
    const MeshAttributes<T>& get(const MeshAttributeHandle<T>& handle) const;

    void push_scope();
    void pop_scope(bool apply_updates = true);
    void clear_current_scope();
};

template <typename T>
const MeshAttributes<T>& AttributeManager::get(PrimitiveType ptype) const
{
    size_t index = get_simplex_dimension(ptype);
    if constexpr (std::is_same_v<T, char>) {
        return m_char_attributes[index];
    }
    if constexpr (std::is_same_v<T, long>) {
        return m_long_attributes[index];
    }
    if constexpr (std::is_same_v<T, double>) {
        return m_double_attributes[index];
    }
    if constexpr (std::is_same_v<T, Rational>) {
        return m_rational_attributes[index];
    }
}

template <typename T>
MeshAttributes<T>& AttributeManager::get(PrimitiveType ptype)
{
    size_t index = get_simplex_dimension(ptype);
    if constexpr (std::is_same_v<T, char>) {
        return m_char_attributes[index];
    }
    if constexpr (std::is_same_v<T, long>) {
        return m_long_attributes[index];
    }
    if constexpr (std::is_same_v<T, double>) {
        return m_double_attributes[index];
    }
    if constexpr (std::is_same_v<T, Rational>) {
        return m_rational_attributes[index];
    }
}

template <typename T>
MeshAttributes<T>& AttributeManager::get(const MeshAttributeHandle<T>& handle)
{
    return get<T>(handle.m_primitive_type);
}
template <typename T>
const MeshAttributes<T>& AttributeManager::get(const MeshAttributeHandle<T>& handle) const
{
    return get<T>(handle.m_primitive_type);
}
template <typename T>
MeshAttributeHandle<T> AttributeManager::register_attribute(
    const std::string& name,
    PrimitiveType ptype,
    long size,
    bool replace)
{
    // return MeshAttributeHandle<T>{
    //    .m_base_handle = get_mesh_attributes<T>(ptype).register_attribute(name, size),
    //    .m_primitive_type = ptype};

    MeshAttributeHandle<T> r;
    r.m_base_handle = get<T>(ptype).register_attribute(name, size, replace),
    r.m_primitive_type = ptype;
    return r;
}
} // namespace attribute
} // namespace wmtk
