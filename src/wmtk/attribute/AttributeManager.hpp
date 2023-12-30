#pragma once

#include <vector>
#include <wmtk/utils/Rational.hpp>
#include "AttributeScopeHandle.hpp"
#include "MeshAttributes.hpp"
#include "TypedAttributeHandle.hpp"
#include "internal/CheckpointScope.hpp"

namespace wmtk {
class Mesh;
class MeshWriter;

namespace attribute {
template <typename T>
class MeshAttributes;
class AttributeManager : public wmtk::utils::MerkleTreeInteriorNode
{
    friend class internal::CheckpointScope;

public:
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

    // handles to all custom attributes
    std::vector<attribute::MeshAttributeHandleVariant> m_custom_attributes;


    // max index used for each type of simplex
    std::vector<long> m_capacities;

    // the number of types of attributes (types of simplex)
    long size() const;

    // attribute directly hashes its "children" components so it overrides "child_hashes"
    std::map<std::string, const wmtk::utils::Hashable*> child_hashables() const override;
    std::map<std::string, std::size_t> child_hashes() const override;

    AttributeScopeHandle create_scope(Mesh& m);
    void serialize(MeshWriter& writer);
    void reserve_to_fit();
    void reserve_attributes_to_fit();
    void reserve_attributes(long dimension, long size);
    // specifies the number of simplices of each type and resizes attributes appropritely
    void set_capacities(std::vector<long> capacities);
    void reserve_more_attributes(long dimension, long size);
    void reserve_more_attributes(const std::vector<long>& more_capacities);
    bool operator==(const AttributeManager& other) const;
    template <typename T>
    TypedAttributeHandle<T> register_attribute(
        Mesh& m,
        const std::string& name,
        PrimitiveType type,
        long size,
        bool replace = false,
        T default_value = T(0),
        bool is_custom = false);
    template <typename T>
    MeshAttributes<T>& get(PrimitiveType ptype);

    template <typename T>
    MeshAttributes<T>& get(const TypedAttributeHandle<T>& handle);

    template <typename T>
    std::string get_name(const TypedAttributeHandle<T>& attr) const;

    std::string get_name(const attribute::MeshAttributeHandleVariant& attr) const;

    template <typename T>
    const MeshAttributes<T>& get(PrimitiveType ptype) const;

    template <typename T>
    const MeshAttributes<T>& get(const TypedAttributeHandle<T>& handle) const;

    void push_scope();
    void pop_scope(bool apply_updates = true);
    void clear_current_scope();

    void change_to_parent_scope() const;
    void change_to_leaf_scope() const;
    template <typename Functor, typename... Args>
    decltype(auto) parent_scope(Functor&& f, Args&&... args) const;

    template <typename T>
    long get_attribute_dimension(const TypedAttributeHandle<T>& handle) const;

    template <typename T>
    void clear_attributes(PrimitiveType ptype, const std::vector<AttributeHandle>& keep_attributes);
};

template <typename T>
const MeshAttributes<T>& AttributeManager::get(PrimitiveType ptype) const
{
    size_t index = get_primitive_type_id(ptype);
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
    size_t index = get_primitive_type_id(ptype);
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
MeshAttributes<T>& AttributeManager::get(const TypedAttributeHandle<T>& handle)
{
    return get<T>(handle.m_primitive_type);
}
template <typename T>
const MeshAttributes<T>& AttributeManager::get(const TypedAttributeHandle<T>& handle) const
{
    return get<T>(handle.m_primitive_type);
}
template <typename T>
TypedAttributeHandle<T> AttributeManager::register_attribute(
    Mesh& m,
    const std::string& name,
    PrimitiveType ptype,
    long size,
    bool replace,
    T default_value,
    bool is_custom)
{
    MeshAttributes<T>& ma = get<T>(ptype);
    const bool exists_already = ma.has_attribute(name);

    TypedAttributeHandle<T> r;
    r.m_base_handle = ma.register_attribute(name, size, replace, default_value),
    r.m_primitive_type = ptype;

    // hacky way to make sure that attributes are not added multiple times to the vector
    if (is_custom && !exists_already) {
        const TypedAttributeHandle<T> rc = r;
        MeshAttributeHandle<T> attr(m, rc);
        m_custom_attributes.emplace_back(attr);
    }

    return r;
}

template <typename Functor, typename... Args>
decltype(auto) AttributeManager::parent_scope(Functor&& f, Args&&... args) const
{
    // we const-cast here because the scope object resets its state  at the end
    // of this scope and we want to use parent-scope for read-only applications
    // anyway ( so it's all read-only-like )
    internal::CheckpointScope scope(const_cast<AttributeManager&>(*this));
    return std::invoke(std::forward<Functor>(f), std::forward<Args>(args)...);
}
template <typename T>
long AttributeManager::get_attribute_dimension(const TypedAttributeHandle<T>& handle) const
{
    assert(handle.is_valid());
    return get(handle).dimension(handle.m_base_handle);
}

template <typename T>
inline void AttributeManager::clear_attributes(
    PrimitiveType ptype,
    const std::vector<AttributeHandle>& keep_attributes)
{
    get<T>(ptype).clear_attributes(keep_attributes);
}

template <typename T>
std::string AttributeManager::get_name(const TypedAttributeHandle<T>& handle) const
{
    return get(handle).get_name(handle.m_base_handle);
}
} // namespace attribute
} // namespace wmtk
