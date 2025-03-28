#pragma once

#include <vector>
#include <wmtk/attribute/utils/variant_comparison.hpp>
#include <wmtk/utils/Rational.hpp>
#include "AttributeScopeHandle.hpp"
#include "TypedAttributeManager.hpp"
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
    std::vector<TypedAttributeManager<char>> m_char_attributes;
    std::vector<TypedAttributeManager<int64_t>> m_long_attributes;
    std::vector<TypedAttributeManager<double>> m_double_attributes;
    std::vector<TypedAttributeManager<Rational>> m_rational_attributes;


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

    std::vector<MeshAttributeHandle::HandleVariant> get_all_attributes() const;


    template <typename T>
    std::vector<TypedAttributeManager<T>>& get();

    template <typename T>
    TypedAttributeManager<T>& get(PrimitiveType ptype);

    template <typename T>
    TypedAttributeManager<T>& get(const TypedAttributeHandle<T>& handle);

    template <typename T>
    std::string get_name(const TypedAttributeHandle<T>& attr) const;

    std::string get_name(const attribute::MeshAttributeHandle::HandleVariant& attr) const;

    template <typename T>
    void set_name(const TypedAttributeHandle<T>& attr, const std::string& name);

    template <typename T>
    const std::vector<TypedAttributeManager<T>>& get() const;

    template <typename T>
    const TypedAttributeManager<T>& get(PrimitiveType ptype) const;

    template <typename T>
    const TypedAttributeManager<T>& get(const TypedAttributeHandle<T>& handle) const;

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

    template <typename T>
    const T& get_attribute_default_value(const TypedAttributeHandle<T>& handle) const;


    /**
     * @brief Remove all custom attributes besides the one passed in.
     *
     * @param keep_attributes Vector of attributes that should not be removed.
     */
    void clear_attributes(
        const std::vector<attribute::MeshAttributeHandle::HandleVariant>& custom_attributes);
    void delete_attribute(const attribute::MeshAttributeHandle::HandleVariant& to_delete);

    /**
     * @brief Validate that handles and attributes are in sync.
     */
    bool validate() const;

    template <typename T>
    bool validate_handle(const TypedAttributeHandle<T>& handle) const;
};

template <typename T>
inline const std::vector<TypedAttributeManager<T>>& AttributeManager::get() const
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
inline std::vector<TypedAttributeManager<T>>& AttributeManager::get()
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
inline const TypedAttributeManager<T>& AttributeManager::get(PrimitiveType ptype) const
{
    const int8_t index = get_primitive_type_id(ptype);
    return get<T>()[index];
}

template <typename T>
inline TypedAttributeManager<T>& AttributeManager::get(PrimitiveType ptype)
{
    size_t index = get_primitive_type_id(ptype);
    return get<T>().at(index);
}

template <typename T>
inline TypedAttributeManager<T>& AttributeManager::get(const TypedAttributeHandle<T>& handle)
{
    return get<T>(handle.m_primitive_type);
}
template <typename T>
inline const TypedAttributeManager<T>& AttributeManager::get(const TypedAttributeHandle<T>& handle) const
{
    return get<T>(handle.m_primitive_type);
}
template <typename T>
inline TypedAttributeHandle<T> AttributeManager::register_attribute(
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
inline decltype(auto) AttributeManager::parent_scope(Functor&& f, Args&&... args) const
{
    // we const-cast here because the scope object resets its state  at the end
    // of this scope and we want to use parent-scope for read-only applications
    // anyway ( so it's all read-only-like )
    internal::CheckpointScope scope(const_cast<AttributeManager&>(*this));
    return std::invoke(std::forward<Functor>(f), std::forward<Args>(args)...);
}
template <typename T>
inline int64_t AttributeManager::get_attribute_dimension(
    const TypedAttributeHandle<T>& handle) const
{
    assert(handle.is_valid());
    return get(handle).dimension(handle.m_base_handle);
}

template <typename T>
inline const T& AttributeManager::get_attribute_default_value(
    const TypedAttributeHandle<T>& handle) const
{
    assert(handle.is_valid());
    return get(handle).default_value(handle.m_base_handle);
}

template <typename T>
inline bool AttributeManager::validate_handle(const TypedAttributeHandle<T>& handle) const
{
    return get(handle).validate_handle(handle.m_base_handle);
}

template <typename T>
inline std::string AttributeManager::get_name(const TypedAttributeHandle<T>& handle) const
{
    return get(handle).get_name(handle.m_base_handle);
}

template <typename T>
inline void AttributeManager::set_name(
    const TypedAttributeHandle<T>& handle,
    const std::string& name)
{
    return get(handle).set_name(handle.m_base_handle, name);
}

} // namespace attribute
} // namespace wmtk
