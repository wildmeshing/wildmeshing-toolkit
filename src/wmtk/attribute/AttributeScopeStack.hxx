#include <wmtk/utils/Rational.hpp>
#include "Attribute.hpp"
#include "AttributeScope.hpp"
#include "AttributeScopeStack.hpp"

namespace wmtk::attribute {

template <typename T>
inline AttributeScopeStack<T>::AttributeScopeStack()
{
    // a random value that's more than 2ish
    m_scopes.reserve(5);
    m_active = m_scopes.end();
}
template <typename T>
inline AttributeScopeStack<T>::~AttributeScopeStack() = default;
template <typename T>
inline void AttributeScopeStack<T>::emplace()
{
    assert(at_current_scope()); // must only be called on leaf node

    // create a new leaf that points to the active stack and
    //
    if (m_scopes.size() + 1 >= m_scopes.capacity()) {
        m_scopes.reserve(m_scopes.capacity() + 3);
        change_to_current_scope();
    }
    m_scopes.emplace_back();
    change_to_current_scope();
}
template <typename T>
inline void AttributeScopeStack<T>::pop(Attribute<T>& attribute, bool preserve_changes)
{
    assert(at_current_scope()); // must only be called on leaf node
    if (!preserve_changes) {
        // rollback_current_scope(attribute);
    }


    if (m_scopes.size() >= 2) {
        auto it = m_scopes.rbegin();
        auto it2 = it + 1;
        it->apply_to(*it2);
    }
    m_scopes.pop_back();

    change_to_current_scope();
}


template <typename T>
inline bool AttributeScopeStack<T>::empty() const
{
    return m_scopes.empty();
}


template <typename T>
inline void AttributeScopeStack<T>::apply_last_scope(Attribute<T>& attr)
{
    assert(at_current_scope());
    assert(!empty());
    apply_scope(m_scopes.back(), attr);
}
template <typename T>
inline void AttributeScopeStack<T>::apply_scope(const AttributeScope<T>& scope, Attribute<T>& attr)
{
    scope.apply(attr);
}
template <typename T>
inline void AttributeScopeStack<T>::apply_scope(
    const AttributeScope<T>& scope,
    const Attribute<T>& attr,
    std::vector<T>& data) const
{
    assert(false);
    // scope.apply(attr, data);
}


template <typename T>
inline void AttributeScopeStack<T>::change_to_previous_scope()
{
    // if the previous is a nullptr it's fine
    assert(!at_current_scope());
    if (m_active == m_scopes.end()) {
        change_to_current_scope();
    } else {
        m_active++;
    }
    m_at_current_scope = (m_active == m_scopes.end());
}

template <typename T>
inline void AttributeScopeStack<T>::change_to_next_scope()
{
    if (at_current_scope()) {
        assert(!empty());
        assert(m_active == m_scopes.end()); // just making sure the definition doesn't change as
                                            // this should be m_scopes.end()-1
        m_at_current_scope = false;
        m_active--;
    } else {
        m_active--;
    }
}
template <typename T>
inline void AttributeScopeStack<T>::change_to_current_scope()
{
    m_at_current_scope = true;
    // m_back = m_scopes.end()-1;
    m_active = m_scopes.end();
}
template <typename T>
inline bool AttributeScopeStack<T>::at_current_scope() const
{
    assert(m_at_current_scope == (m_active == m_scopes.end()));
    return m_at_current_scope;
}
template <typename T>
inline bool AttributeScopeStack<T>::writing_enabled() const
{
    return at_current_scope();
}

} // namespace wmtk::attribute
