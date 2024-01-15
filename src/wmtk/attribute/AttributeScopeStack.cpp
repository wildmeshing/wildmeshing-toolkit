#include "AttributeScopeStack.hpp"
#include <wmtk/utils/Rational.hpp>
#include "Attribute.hpp"
#include "AttributeScope.hpp"

namespace wmtk::attribute {

template <typename T>
AttributeScopeStack<T>::AttributeScopeStack() = default;
template <typename T>
AttributeScopeStack<T>::~AttributeScopeStack() = default;
template <typename T>
void AttributeScopeStack<T>::emplace()
{
    assert(m_current == m_leaf.get()); // must only be called on leaf node

    // create a new leaf that points to the current stack and
    //
    std::unique_ptr<AttributeScope<T>> new_leaf(new AttributeScope<T>(std::move(m_leaf)));
    m_leaf = std::move(new_leaf);
    change_to_leaf_scope();
}
template <typename T>
void AttributeScopeStack<T>::pop(Attribute<T>& attribute, bool apply_updates)
{
    // delete myself by setting my parent to be the leaf
    assert(bool(m_leaf));
    assert(m_current == m_leaf.get()); // must only be called on leaf node
    if (apply_updates) {
        m_leaf->flush(attribute);
    }
#if defined(WMTK_ENABLE_GENERIC_CHECKPOINTS)
    if (!m_checkpoints.empty()) {
        while (m_checkpoints.back() == m_leaf.get()) {
            m_checkpoints.pop_back();
        }
    }
#endif
    m_leaf = std::move(m_leaf->pop_parent());
    change_to_leaf_scope();
}

template <typename T>
void AttributeScopeStack<T>::flush_changes_to_vector(const Attribute<T>& attr, std::vector<T>& data)
    const
{
    if (m_leaf) {
        m_leaf->flush_changes_to_vector(attr, data);
    }
}


template <typename T>
bool AttributeScopeStack<T>::empty() const
{
    return !bool(m_leaf);
}

template <typename T>
int64_t AttributeScopeStack<T>::depth() const
{
    if (bool(m_leaf)) {
        return m_leaf->depth();
    } else {
        return 0;
    }
}

#if defined(!WMTK_FLUSH_ON_FAIL)
template <typename T>
AttributeScope<T>* AttributeScopeStack<T>::current_scope_ptr()
{
    return m_current;
}

template <typename T>
const AttributeScope<T>* AttributeScopeStack<T>::current_scope_ptr() const
{
    return m_current;
}
#endif

template <typename T>
void AttributeScopeStack<T>::clear_current_scope()
{
    assert(m_current == m_leaf.get()); // must only be called on leaf node
    if (bool(m_leaf)) {
        m_leaf->clear();
    }
}

#if defined(WMTK_ENABLE_GENERIC_CHECKPOINTS)
template <typename T>
int64_t AttributeScopeStack<T>::add_checkpoint()
{
    int64_t r = m_checkpoints.size();
    m_checkpoints.push_back(m_leaf.get());
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
void AttributeScopeStack<T>::change_to_parent_scope() const
{
    assert(!empty());
    m_current = m_current->parent();
}

template <typename T>
void AttributeScopeStack<T>::change_to_leaf_scope() const
{
    m_current = m_leaf.get();
}
template <typename T>
bool AttributeScopeStack<T>::at_leaf_scope() const
{
    return !(bool(m_leaf)) || m_current == m_leaf.get();
}
template <typename T>
bool AttributeScopeStack<T>::writing_enabled() const
{
    return at_leaf_scope();
}

template class AttributeScopeStack<int64_t>;
template class AttributeScopeStack<double>;
template class AttributeScopeStack<char>;
template class AttributeScopeStack<Rational>;
} // namespace wmtk::attribute
