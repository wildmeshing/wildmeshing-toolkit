#include <stdexcept>
#include <wmtk/utils/Rational.hpp>
#include "Attribute.hpp"
#include "AttributeScope.hpp"
#include "AttributeScopeStack.hpp"

namespace wmtk::attribute {

template <typename T>
AttributeScopeStack<T>::AttributeScopeStack()
    : m_scopes()
    , m_active(m_scopes.end())
{
    m_scopes.reserve(4);
    change_to_current_scope();
}
template <typename T>
AttributeScopeStack<T>::~AttributeScopeStack() = default;
template <typename T>
void AttributeScopeStack<T>::emplace()
{
    if (m_scopes.size() >= 10) {
        throw std::runtime_error("CAnnot have more than 4 scopes!");
    }
    assert(at_current_scope()); // must only be called on leaf node

    // make sure there's room for reservation
    if (m_scopes.size() + 1 >= m_scopes.capacity()) {
        reserve(m_scopes.size() + 4);
    }
    bool was_empty = empty();
    m_scopes.emplace_back();
    if (!was_empty) {
        auto rb = m_scopes.rbegin();
        auto& cur = *(rb);
        auto& old = *(rb + 1);
        old.m_next = &cur;
        cur.m_previous = &old;
    }
    change_to_current_scope();
}
template <typename T>
void AttributeScopeStack<T>::pop(Attribute<T>& attribute, bool apply_updates)
{
    // delete myself by setting my parent to be the leaf
    assert(!empty());

    //assert(m_active == nullptr); // must only be called on leaf node
    assert(m_active == m_scopes.end());
    // equivalent to the above but to express a semantic
    assert(writing_enabled());
#if defined(WMTK_ONLY_CACHE_WRITES)
    if (!apply_updates) {
#else
    if (apply_updates) {
#endif
        m_scopes.back().flush(attribute);
    }
#if defined(WMTK_ENABLE_GENERIC_CHECKPOINTS)
    if (!m_checkpoints.empty()) {
        while (m_checkpoints.back() == m_start.get()) {
            m_checkpoints.pop_back();
        }
    }
#endif

    // std::vector shouldn't re-allocate if resizing to a smaller size
    m_scopes.pop_back();
    change_to_current_scope();
}

template <typename T>
void AttributeScopeStack<T>::flush_changes_to_vector(const Attribute<T>& attr, std::vector<T>& data)
    const
{
    assert(!empty());
    assert(m_active == m_scopes.end() - 1);
    m_active->flush_changes_to_vector(attr, data);
}


template <typename T>
bool AttributeScopeStack<T>::empty() const
{
    return m_scopes.empty();
}

template <typename T>
int64_t AttributeScopeStack<T>::depth() const
{
    return m_scopes.size();
}

template <typename T>
AttributeScope<T>* AttributeScopeStack<T>::active_scope_ptr()
{
    return &*m_active;
}

template <typename T>
const AttributeScope<T>* AttributeScopeStack<T>::active_scope_ptr() const
{
    return &*m_active;
}

template <typename T>
void AttributeScopeStack<T>::clear_current_scope(Attribute<T>& attr)
{
    assert(writing_enabled());

    assert(!empty());
    m_scopes.back().flush(attr);
}

#if defined(WMTK_ENABLE_GENERIC_CHECKPOINTS)
template <typename T>
int64_t AttributeScopeStack<T>::add_checkpoint()
{
    int64_t r = m_checkpoints.size();
    m_checkpoints.push_back(m_start.get());
    return r;
}
template <typename T>
AttributeScope<T> const* AttributeScopeStack<T>::get_checkpoint(int64_t index) const
{
    if (m_checkpoints.empty()) {
        return nullptr;
    } else {
        return m_checkpoints.at(index);
    }
}
#endif
template <typename T>
void AttributeScopeStack<T>::change_to_previous_scope() const
{
    // if the previous is a nullptr it's fine
    assert(m_active != m_scopes.end());
    // this should be equivalent - but is left here to express a semantic
    assert(!at_current_scope());
    m_active++;
}

template <typename T>
void AttributeScopeStack<T>::change_to_next_scope() const
{
    assert(m_active != m_scopes.begin());
    m_active--;
}
template <typename T>
void AttributeScopeStack<T>::change_to_current_scope() const
{
    m_active = m_scopes.end();
}
template <typename T>
bool AttributeScopeStack<T>::at_current_scope() const
{
    return m_active == m_scopes.end();
}
template <typename T>
bool AttributeScopeStack<T>::writing_enabled() const
{
    return at_current_scope();
}

template <typename T>
void AttributeScopeStack<T>::reserve(size_t size)
{
    if (size > m_scopes.capacity()) {
        m_scopes.reserve(size);
    }
    for (size_t j = 0; j < m_scopes.size(); ++j) {
        AttributeScope<T>& scope = m_scopes[j];
        if (scope.m_previous != nullptr) {
            assert(j != 0);
            scope.m_previous = &m_scopes[j - 1];
        }
        if (scope.m_next != nullptr) {
            assert(j != m_scopes.size() - 1);
            scope.m_next = &m_scopes[j + 1];
        }
    }
}

// template class AttributeScopeStack<int64_t>;
// template class AttributeScopeStack<double>;
// template class AttributeScopeStack<char>;
// template class AttributeScopeStack<Rational>;
} // namespace wmtk::attribute
