#pragma once

#include <vector>
#include <wmtk/attribute/utils/variant_comparison.hpp>
#include <wmtk/utils/Rational.hpp>
#include "AttributeScopeHandle.hpp"
#include "MeshAttributes.hpp"
#include "TypedAttributeHandle.hpp"
#include "internal/CheckpointScope.hpp"

namespace wmtk {
class Mesh;
class MeshWriter;

namespace attribute {
class AttributeManager : public wmtk::utils::MerkleTreeInteriorNode
{
    friend class internal::CheckpointScope;

public:
    AttributeManager(int64_t size);
    ~AttributeManager();
    AttributeManager(const AttributeManager& o) = delete;
    AttributeManager(AttributeManager&& o) = default;
    AttributeManager& operator=(const AttributeManager& o) = delete;
    AttributeManager& operator=(AttributeManager&& o) = default;

    //=========================================================
    // Storage of Mesh Attributes
    //=========================================================
    std::vector<MeshAttributes<char>> m_char_attributes;
    std::vector<MeshAttributes<int64_t>> m_long_attributes;
    std::vector<MeshAttributes<double>> m_double_attributes;
    std::vector<MeshAttributes<Rational>> m_rational_attributes;


    // max index used for each type of simplex
    std::vector<int64_t> m_capacities;

    // the number of types of attributes (types of simplex)
    int64_t size() const;

    // attribute directly hashes its "children" components so it overrides "child_hashes"
    std::map<std::string, const wmtk::utils::Hashable*> child_hashables() const override;
    std::map<std::string, std::size_t> child_hashes() const override;

    AttributeScopeHandle create_scope(Mesh& m);
    void serialize(MeshWriter& writer) const;
    void reserve_to_fit();
    void reserve_attributes_to_fit();
    void reserve_attributes(int64_t dimension, int64_t size);
    // specifies the number of simplices of each type and resizes attributes appropritely
    void set_capacities(std::vector<int64_t> capacities);
    void reserve_more_attributes(int64_t dimension, int64_t size);
    void reserve_more_attributes(const std::vector<int64_t>& more_capacities);
    void guarantee_more_attributes(int64_t dimension, int64_t size);
    void guarantee_more_attributes(const std::vector<int64_t>& more_capacities);
    void guarantee_at_least_attributes(int64_t dimension, int64_t size);
    void guarantee_at_least_attributes(const std::vector<int64_t>& at_least_capacities);
    bool operator==(const AttributeManager& other) const;

    void assert_capacity_valid() const;

    template <typename T>
    TypedAttributeHandle<T> register_attribute(
        const std::string& name,
        PrimitiveType type,
        int64_t size,
        bool replace,
        T default_value);

    std::vector<TypedAttributeHandleVariant> get_all_attributes() const;


    template <typename T>
    std::vector<MeshAttributes<T>>& get();

    template <typename T>
    MeshAttributes<T>& get(PrimitiveType ptype);

    template <typename T>
    MeshAttributes<T>& get(const TypedAttributeHandle<T>& handle);

    template <typename T>
    std::string get_name(const TypedAttributeHandle<T>& attr) const;

    std::string get_name(const attribute::TypedAttributeHandleVariant& attr) const;

    template <typename T>
    const std::vector<MeshAttributes<T>>& get() const;

    template <typename T>
    const MeshAttributes<T>& get(PrimitiveType ptype) const;

    template <typename T>
    const MeshAttributes<T>& get(const TypedAttributeHandle<T>& handle) const;

    void push_scope();
    void pop_scope(bool apply_updates = true);
    void rollback_current_scope();
    void flush_all_scopes();

    void change_to_parent_scope() const;
    void change_to_child_scope() const;
    template <typename Functor, typename... Args>
    decltype(auto) parent_scope(Functor&& f, Args&&... args) const;

    template <typename T>
    int64_t get_attribute_dimension(const TypedAttributeHandle<T>& handle) const;

    /**
     * @brief Remove all custom attributes besides the one passed in.
     *
     * @param keep_attributes Vector of attributes that should not be removed.
     */
    void clear_attributes(
        const std::vector<attribute::TypedAttributeHandleVariant>& custom_attributes);
};

template <typename T>
const std::vector<MeshAttributes<T>>& AttributeManager::get() const
{
    if constexpr (std::is_same_v<T, char>) {
        return m_char_attributes;
    }
    if constexpr (std::is_same_v<T, int64_t>) {
        return m_long_attributes;
    }
    if constexpr (std::is_same_v<T, double>) {
        return m_double_attributes;
    }
    if constexpr (std::is_same_v<T, Rational>) {
        return m_rational_attributes;
    }
}
template <typename T>
std::vector<MeshAttributes<T>>& AttributeManager::get()
{
    if constexpr (std::is_same_v<T, char>) {
        return m_char_attributes;
    }
    if constexpr (std::is_same_v<T, int64_t>) {
        return m_long_attributes;
    }
    if constexpr (std::is_same_v<T, double>) {
        return m_double_attributes;
    }
    if constexpr (std::is_same_v<T, Rational>) {
        return m_rational_attributes;
    }
}

template <typename T>
const MeshAttributes<T>& AttributeManager::get(PrimitiveType ptype) const
{
    const int8_t index = get_primitive_type_id(ptype);
    return get<T>()[index];
}

template <typename T>
MeshAttributes<T>& AttributeManager::get(PrimitiveType ptype)
{
    size_t index = get_primitive_type_id(ptype);
    return get<T>().at(index);
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
    const std::string& name,
    PrimitiveType ptype,
    int64_t size,
    bool replace,
    T default_value)
{
    TypedAttributeHandle<T> r;
    r.m_base_handle = get<T>(ptype).register_attribute(name, size, replace, default_value),
    r.m_primitive_type = ptype;

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
int64_t AttributeManager::get_attribute_dimension(const TypedAttributeHandle<T>& handle) const
{
    assert(handle.is_valid());
    return get(handle).dimension(handle.m_base_handle);
}

template <typename T>
std::string AttributeManager::get_name(const TypedAttributeHandle<T>& handle) const
{
    return get(handle).get_name(handle.m_base_handle);
}
} // namespace attribute
} // namespace wmtk
