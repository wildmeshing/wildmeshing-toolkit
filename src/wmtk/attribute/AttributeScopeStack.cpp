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
    // create a new leaf that points to the current stack and
    //
    std::unique_ptr<AttributeScope<T>> new_leaf(new AttributeScope<T>(std::move(m_leaf)));
    m_leaf = std::move(new_leaf);
}
template <typename T>
void AttributeScopeStack<T>::pop(Attribute<T>& attribute, bool apply_updates)
{
    // delete myself by setting my parent to be the leaf
    assert(bool(m_leaf));
    if (apply_updates) {
        m_leaf->flush(attribute);
    }
    if (!m_checkpoints.empty()) {
        while (m_checkpoints.back() == m_leaf.get()) {
            m_checkpoints.pop_back();
        }
    }
    m_leaf = std::move(m_leaf->pop_parent());
}

template <typename T>
bool AttributeScopeStack<T>::empty() const
{
    return !bool(m_leaf);
}

template <typename T>
long AttributeScopeStack<T>::depth() const
{
    if (bool(m_leaf)) {
        return m_leaf->depth();
    } else {
        return 0;
    }
}

template <typename T>
AttributeScope<T>* AttributeScopeStack<T>::current_scope_ptr()
{
    if (bool(m_leaf)) {
        return m_leaf.get();
    } else {
        return nullptr;
    }
}

template <typename T>
const AttributeScope<T>* AttributeScopeStack<T>::current_scope_ptr() const
{
    if (bool(m_leaf)) {
        return m_leaf.get();
    } else {
        return nullptr;
    }
}

template <typename T>
void AttributeScopeStack<T>::clear_current_scope()
{
    if (bool(m_leaf)) {
        m_leaf->clear();
    }
}

template <typename T>
long AttributeScopeStack<T>::add_checkpoint()
{
    long r = m_checkpoints.size();
    m_checkpoints.push_back(m_leaf.get());
    return r;
}
template <typename T>
AttributeScope<T> const* AttributeScopeStack<T>::get_checkpoint(long index) const
{
    if (m_checkpoints.empty()) {
        return nullptr;
    } else {
        return m_checkpoints.at(index);
    }
}
template class AttributeScopeStack<long>;
template class AttributeScopeStack<double>;
template class AttributeScopeStack<char>;
template class AttributeScopeStack<Rational>;
} // namespace wmtk::attribute
