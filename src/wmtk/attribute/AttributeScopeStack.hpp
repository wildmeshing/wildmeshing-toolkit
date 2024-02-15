#pragma once
#include <iostream>
#include <memory>
#include <vector>
#include "AttributeCache.hpp"
#include "AttributeHandle.hpp"
#include "internal/MapTypes.hpp"

namespace wmtk {
class Mesh;
namespace attribute {
template <typename T>
class Attribute;
template <typename T>
class AccessorBase;
template <typename T>
class AttributeScope;
class AttributeManager;

/**
 * A stack of changes applied to an Attribute.
 * The stack consists of AttributeScopes which hold all changes applied inside one scope. Whenever a
 * new scope is created, it is pushed to the stack. As soon as an AttributeScopeHandle is
 * destructed, `push_scope` of all Attributes is triggered.
 */
template <typename T>
class AttributeScopeStack
{
public:
    template <int D = Eigen::Dynamic>
    using MapResult = internal::MapResult<T,D>;
    template <int D = Eigen::Dynamic>
    using ConstMapResult = internal::ConstMapResult<T,D>;
    // stack is implemented by a parent pointing graph, so we track a pointer
    // to the leaf
    AttributeScopeStack();
    ~AttributeScopeStack();
    AttributeScopeStack(const AttributeScopeStack&) = delete;
    AttributeScopeStack& operator=(const AttributeScopeStack&) = delete;
    AttributeScopeStack(AttributeScopeStack&&) = default;
    AttributeScopeStack& operator=(AttributeScopeStack&&) = default;

    bool empty() const;
    int64_t size() const;


    bool writing_enabled() const;


    /// default mutable vector access
    template <int D = Eigen::Dynamic>
    MapResult<D> vector_attribute(AccessorBase<T>& accessor, int64_t index);
    /// default immutable vector access

    template <int D = Eigen::Dynamic>
    ConstMapResult<D> const_vector_attribute(const AccessorBase<T>& accessor, int64_t index) const;
    /// default mutable scalar access
    T& scalar_attribute(AccessorBase<T>& accessor, int64_t index);

    /// default immutable scalar access
    T const_scalar_attribute(const AccessorBase<T>& accessor, int64_t index) const;

    template <int D = Eigen::Dynamic>
    /// specialized immutable scalar access useful for topological operations
    T const_scalar_attribute(const AccessorBase<T>& accessor, int64_t index, int8_t offset) const;

    void emplace();
    void pop(Attribute<T>& attribute, bool preserve_changes);
    // go to the next most historic scope
    void change_to_next_scope();
    void change_to_previous_scope();
    // go to the scope with active data
    void change_to_current_scope();
    void rollback_current_scope(Attribute<T>& attr);

    /// checks that we are viewing the active state of the attribute
    bool at_current_scope() const;
    /// applies the diffs from the last scope to the current attribute
private:
    void apply_last_scope(Attribute<T>& attr);
    /// apply a particular scope to the current attribute
    void apply_scope(const AttributeScope<T>& scope, Attribute<T>& attr);
    /// apply a particular scope to a piece of data
    void apply_scope(const AttributeScope<T>& scope, const Attribute<T>& attr, std::vector<T>& data)
        const;


    // current scope is

protected:
    std::vector<AttributeScope<T>> m_scopes;
    typename std::vector<AttributeScope<T>>::const_iterator m_active;
    // Mesh& m_mesh;
    // AttributeManager& m_attribute_manager;
    // MeshAttributeHandle<T> m_handle;
};
template <typename T>
int64_t AttributeScopeStack<T>::size() const
{
    return m_scopes.size();
}

template <typename T>
void AttributeScopeStack<T>::rollback_current_scope(Attribute<T>& attr)
{
    assert(!empty());
    assert(at_current_scope());
    apply_last_scope(attr);
}

template <typename T>
template <int D>
inline auto AttributeScopeStack<T>::vector_attribute(AccessorBase<T>& accessor, int64_t index)
    -> MapResult<D>
{
    assert(writing_enabled());

    auto data = accessor.template vector_attribute<D>(index);
    if (!empty()) {
        m_scopes.back().try_caching(index, data);
    }
    return data;
}

template <typename T>
template <int D>
inline auto AttributeScopeStack<T>::const_vector_attribute(
    const AccessorBase<T>& accessor,
    int64_t index) const -> ConstMapResult<D>
{
    if (!at_current_scope()) {
        assert(m_active >= m_scopes.begin());
        assert(m_active < m_scopes.end());
        for (auto it = m_active; it < m_scopes.end(); ++it) {
        //for (auto it = m_active; it < m_scopes.rend(); ++it) {
            if (auto mapit = it->find_value(index); it->is_value(mapit)) {
                const auto& d = mapit->second;
                auto dat = d.template data_as_const_map<D>();
                return dat;
            }
        }
    }
    return accessor.template const_vector_attribute<D>(index);
}

template <typename T>
inline auto AttributeScopeStack<T>::scalar_attribute(AccessorBase<T>& accessor, int64_t index) -> T&
{
    assert(writing_enabled());
    T& value =  accessor.scalar_attribute(index);
    if (!empty()) {
        m_scopes.back().try_caching(index, value);
    }
    return value;
}

template <typename T>
inline auto AttributeScopeStack<T>::const_scalar_attribute(
    const AccessorBase<T>& accessor,
    int64_t index) const -> T
{
    return const_vector_attribute<1>(accessor, index)(0);
}
template <typename T>
template <int D>
inline auto AttributeScopeStack<T>::const_scalar_attribute(
    const AccessorBase<T>& accessor,
    int64_t index,
    int8_t offset) const -> T
{
    if (!at_current_scope()) {
        return const_vector_attribute<D>(accessor, index)(offset);
    } else {
        return accessor.const_scalar_attribute(index, offset);
    }
}
} // namespace attribute
} // namespace wmtk
#include "AttributeScopeStack.hxx"
