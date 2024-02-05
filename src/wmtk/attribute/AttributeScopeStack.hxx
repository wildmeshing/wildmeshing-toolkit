#include <wmtk/utils/Rational.hpp>
#include "Attribute.hpp"
#include "AttributeScope.hpp"
#include "AttributeScopeStack.hpp"

namespace wmtk::attribute {

template <typename T>
AttributeScopeStack<T>::AttributeScopeStack()
{
    m_scopes.reserve(5);
    m_active = m_scopes.rend();
}
template <typename T>
AttributeScopeStack<T>::~AttributeScopeStack() = default;
template <typename T>
void AttributeScopeStack<T>::emplace()
{
    assert(at_current_scope()); // must only be called on leaf node

    // create a new leaf that points to the active stack and
    //
    if (m_scopes.size() + 1 >= m_scopes.capacity()) {
        m_scopes.reserve(m_scopes.capacity() + 2);
        change_to_current_scope();
    }
    m_scopes.emplace_back();
    change_to_current_scope();
}
template <typename T>
void AttributeScopeStack<T>::pop(Attribute<T>& attribute, bool preserve_changes)
{
    assert(at_current_scope()); // must only be called on leaf node
    if (!preserve_changes) {
        rollback_current_scope(attribute);
    }

    change_to_current_scope();
}


template <typename T>
bool AttributeScopeStack<T>::empty() const
{
    return m_scopes.empty();
}


template <typename T>
void AttributeScopeStack<T>::apply_last_scope(Attribute<T>& attr)
{
    assert(at_current_scope());
    assert(!empty());
    apply_scope(m_scopes.back(), attr);
}
template <typename T>
void AttributeScopeStack<T>::apply_scope(const AttributeScope<T>& scope, Attribute<T>& attr)
{
    scope.apply(attr);
}
template <typename T>
void AttributeScopeStack<T>::apply_scope(
    const AttributeScope<T>& scope,
    const Attribute<T>& attr,
    std::vector<T>& data) const
{
    assert(false);
    // scope.apply(attr, data);
}


template <typename T>
void AttributeScopeStack<T>::change_to_previous_scope()
{
    // if the previous is a nullptr it's fine
    assert(!at_current_scope());
    m_active--;
}

template <typename T>
void AttributeScopeStack<T>::change_to_next_scope()
{
    if (at_current_scope()) {
        assert(!empty());
        m_active = m_scopes.rbegin();
        return;
    }
    m_active++;
}
template <typename T>
void AttributeScopeStack<T>::change_to_current_scope()
{
    m_active = m_scopes.rend();
}
template <typename T>
bool AttributeScopeStack<T>::at_current_scope() const
{
    return m_active == m_scopes.rend();
}
template <typename T>
bool AttributeScopeStack<T>::writing_enabled() const
{
    return at_current_scope();
}


// template <typename T>
// auto AttributeScope<T>::load_const_cached_scalar_value(
//     const AccessorBase<T>& accessor,
//     int64_t index) const -> T
//{
//     if (auto it = m_data.find(index); it != m_data.end()) {
//         const auto& dat = it->second.data;
//         assert(dat.size() == 1);
//         return dat(0);
//     } else if (m_previous) {
//         return m_previous->load_const_cached_scalar_value(accessor, index);
//     } else {
//         return accessor.const_scalar_attribute(index);
//     }
// }
//
//
// template <typename T>
// auto AttributeScope<T>::load_const_cached_vector_value(
//     const AccessorBase<T>& accessor,
//     int64_t index) const -> ConstMapResult
//{
//     if (auto it = m_data.find(index); it != m_data.end()) {
//         auto& dat = it->second.data;
//         auto v = ConstMapResult(dat.data(), dat.size());
//         return v;
//     } else if (m_previous) {
//         auto v = m_previous->load_const_cached_vector_value(accessor, index);
//         return v;
//     } else {
//         auto v = accessor.const_vector_attribute(index);
//         return v;
//     }
// }


// template class AttributeScopeStack<int64_t>;
// template class AttributeScopeStack<double>;
// template class AttributeScopeStack<char>;
// template class AttributeScopeStack<Rational>;
} // namespace wmtk::attribute
