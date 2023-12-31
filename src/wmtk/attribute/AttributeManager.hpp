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
template <typename T>
class MeshAttributes;
class AttributeManager : public wmtk::utils::MerkleTreeInteriorNode
{
    friend class internal::CheckpointScope;

public:
    AttributeManager(int64_t size);
    ~AttributeManager();
    AttributeManager(const AttributeManager& o);
    AttributeManager(AttributeManager&& o);
    AttributeManager& operator=(const AttributeManager& o);
    AttributeManager& operator=(AttributeManager&& o);

    //=========================================================
    // Storage of Mesh Attributes
    //=========================================================
    std::vector<MeshAttributes<char>> m_char_attributes;
    std::vector<MeshAttributes<int64_t>> m_long_attributes;
    std::vector<MeshAttributes<double>> m_double_attributes;
    std::vector<MeshAttributes<Rational>> m_rational_attributes;

    // handles to all custom attributes
    std::vector<attribute::TypedAttributeHandleVariant> m_custom_attributes;


    // max index used for each type of simplex
    std::vector<int64_t> m_capacities;

    // the number of types of attributes (types of simplex)
    int64_t size() const;

    // attribute directly hashes its "children" components so it overrides "child_hashes"
    std::map<std::string, const wmtk::utils::Hashable*> child_hashables() const override;
    std::map<std::string, std::size_t> child_hashes() const override;

    AttributeScopeHandle create_scope(Mesh& m);
    void serialize(MeshWriter& writer);
    void reserve_to_fit();
    void reserve_attributes_to_fit();
    void reserve_attributes(int64_t dimension, int64_t size);
    // specifies the number of simplices of each type and resizes attributes appropritely
    void set_capacities(std::vector<int64_t> capacities);
    void reserve_more_attributes(int64_t dimension, int64_t size);
    void reserve_more_attributes(const std::vector<int64_t>& more_capacities);
    bool operator==(const AttributeManager& other) const;

    template <typename T>
    TypedAttributeHandle<T> register_attribute_custom(
        const std::string& name,
        PrimitiveType type,
        int64_t size,
        bool replace,
        T default_value);

    template <typename T>

    TypedAttributeHandle<T> register_attribute_builtin(
        const std::string& name,
        PrimitiveType type,
        int64_t size,
        bool replace,
        T default_value);

    template <typename T>
    MeshAttributes<T>& get(PrimitiveType ptype);

    template <typename T>
    MeshAttributes<T>& get(const TypedAttributeHandle<T>& handle);

    template <typename T>
    std::string get_name(const TypedAttributeHandle<T>& attr) const;

    std::string get_name(const attribute::TypedAttributeHandleVariant& attr) const;

    template <typename T>
    const MeshAttributes<T>& get(PrimitiveType ptype) const;

    template <typename T>
    const MeshAttributes<T>& get(const TypedAttributeHandle<T>& handle) const;

    void push_scope();
    void pop_scope(bool apply_updates = true);
    void clear_current_scope();
    void flush_all_scopes();

    void change_to_parent_scope() const;
    void change_to_leaf_scope() const;
    template <typename Functor, typename... Args>
    decltype(auto) parent_scope(Functor&& f, Args&&... args) const;

    template <typename T>
    int64_t get_attribute_dimension(const TypedAttributeHandle<T>& handle) const;

    void remove_attributes(std::vector<attribute::TypedAttributeHandleVariant> keep_attributes);
};

template <typename T>
const MeshAttributes<T>& AttributeManager::get(PrimitiveType ptype) const
{
    size_t index = get_primitive_type_id(ptype);
    if constexpr (std::is_same_v<T, char>) {
        return m_char_attributes[index];
    }
    if constexpr (std::is_same_v<T, int64_t>) {
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
    if constexpr (std::is_same_v<T, int64_t>) {
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
TypedAttributeHandle<T> AttributeManager::register_attribute_custom(
    const std::string& name,
    PrimitiveType ptype,
    int64_t size,
    bool replace,
    T default_value)
{
    // the difference between registering a custom and builtin attribute is that the custom one gets
    // added to the custom list. We can tehrefoer just use the existing builtin attribute
    auto attr = register_attribute_builtin(name, ptype, size, replace, default_value);

    for (const auto& attr_var : m_custom_attributes) {
        if (utils::variant_comparison(attr, attr_var)) {
            return attr;
        }
    }
    m_custom_attributes.emplace_back(attr);
    return attr;
}
template <typename T>
TypedAttributeHandle<T> AttributeManager::register_attribute_builtin(
    const std::string& name,
    PrimitiveType ptype,
    int64_t size,
    bool replace,
    T default_value)
{
    MeshAttributes<T>& ma = get<T>(ptype);
    const bool exists_already = ma.has_attribute(name);

    TypedAttributeHandle<T> r;
    r.m_base_handle = ma.register_attribute(name, size, replace, default_value),
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
inline void AttributeManager::remove_attributes(
    std::vector<attribute::TypedAttributeHandleVariant> keep_attributes)
{
    std::array<std::array<std::vector<AttributeHandle>, 5>, 4>
        keeps; // [char/long/...][ptype][attribute]

    for (const attribute::TypedAttributeHandleVariant& attr : keep_attributes) {
        std::visit(
            [&](auto&& val) {
                using T = typename std::decay_t<decltype(val)>::Type;

                int64_t type_id = -1;

                if constexpr (std::is_same_v<T, char>) {
                    type_id = 0;
                }
                if constexpr (std::is_same_v<T, long>) {
                    type_id = 1;
                }
                if constexpr (std::is_same_v<T, double>) {
                    type_id = 2;
                }
                if constexpr (std::is_same_v<T, Rational>) {
                    type_id = 3;
                }

                assert(type_id != -1);

                keeps[type_id][get_primitive_type_id(val.primitive_type())].emplace_back(
                    val.base_handle());
            },
            attr);
    }

    std::array<std::array<std::vector<AttributeHandle>, 5>, 4>
        customs; // [char/long/...][ptype][attribute]

    for (const attribute::TypedAttributeHandleVariant& attr : m_custom_attributes) {
        std::visit(
            [&](auto&& val) {
                using T = typename std::decay_t<decltype(val)>::Type;

                int64_t type_id = -1;

                if constexpr (std::is_same_v<T, char>) {
                    type_id = 0;
                }
                if constexpr (std::is_same_v<T, long>) {
                    type_id = 1;
                }
                if constexpr (std::is_same_v<T, double>) {
                    type_id = 2;
                }
                if constexpr (std::is_same_v<T, Rational>) {
                    type_id = 3;
                }

                assert(type_id != -1);

                customs[type_id][get_primitive_type_id(val.primitive_type())].emplace_back(
                    val.base_handle());
            },
            attr);
    }

    for (size_t type_id = 0; type_id < keeps.size(); ++type_id) {
        for (size_t ptype_id = 0; ptype_id < keeps[0].size(); ++ptype_id) {
            std::vector<AttributeHandle> diff_char;
            std::vector<AttributeHandle>& keeps_char = keeps[0][ptype_id];
            std::vector<AttributeHandle>& customs_char = customs[0][ptype_id];

            std::sort(
                keeps_char.begin(),
                keeps_char.end()
                );

            std::sort(
                customs_char.begin(),
                customs_char.end()
                );

            std::set_difference(
                customs_char.begin(),
                customs_char.end(),
                keeps_char.begin(),
                keeps_char.end(),
                std::inserter(diff_char, diff_char.begin())
                );

            switch (type_id) {
            case 0:
                get<char>(get_primitive_type_from_id(ptype_id)).remove_attributes(diff_char);
                break;
            case 1:
                get<long>(get_primitive_type_from_id(ptype_id)).remove_attributes(diff_char);
                break;
            case 2:
                get<double>(get_primitive_type_from_id(ptype_id)).remove_attributes(diff_char);
                break;
            case 3:
                get<Rational>(get_primitive_type_from_id(ptype_id)).remove_attributes(diff_char);
                break;
            default: throw std::runtime_error("unknown type"); break;
            }
        }
    }

    // clean up m_custom_attributes

    // std::vector<attribute::TypedAttributeHandleVariant> custom_remain;
    // for (const attribute::TypedAttributeHandleVariant& attr : keep_attributes) {
    //     std::visit(
    //         [&](auto&& val) {
    //             using T = typename std::decay_t<decltype(val)>::Type;
    //
    //             const MeshAttributeHandle<T>& a = std::get<T>(attr);
    //            // get<T>(a.primitive_type()).has_attribute
    //        },
    //        attr);
    //}
}

template <typename T>
std::string AttributeManager::get_name(const TypedAttributeHandle<T>& handle) const
{
    return get(handle).get_name(handle.m_base_handle);
}
} // namespace attribute
} // namespace wmtk
