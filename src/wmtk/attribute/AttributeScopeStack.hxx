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
    assert(at_current_scope()); // must only be called on leaf node

    // create a new leaf that points to the active stack and
    //
    if(bool(m_start)) {
    std::unique_ptr<AttributeScope<T>> new_start = std::make_unique<AttributeScope<T>>(std::move(m_start));
    m_start = std::move(new_start);
    } else {
        m_start = std::make_unique<AttributeScope<T>>();
    }
    change_to_current_scope();
}
template <typename T>
void AttributeScopeStack<T>::pop(Attribute<T>& attribute, bool apply_updates)
{
    // delete myself by setting my parent to be the leaf
    assert(bool(m_start));

#if defined(WMTK_FLUSH_ON_FAIL)
    assert(m_active == nullptr); // must only be called on leaf node
#else
    assert(m_active == m_start.get()); // must only be called on leaf node
#endif
#if defined(WMTK_ONLY_CACHE_WRITES)
    if (!apply_updates) {
#else
    if (apply_updates) {
#endif
        m_start->flush(attribute);
    }
#if defined(WMTK_ENABLE_GENERIC_CHECKPOINTS)
    if (!m_checkpoints.empty()) {
        while (m_checkpoints.back() == m_start.get()) {
            m_checkpoints.pop_back();
        }
    }
#endif
    m_start = m_start->pop_to_next();
    change_to_current_scope();
}

template <typename T>
void AttributeScopeStack<T>::flush_changes_to_vector(const Attribute<T>& attr, std::vector<T>& data)
    const
{
    if (m_start) {
        m_start->flush_changes_to_vector(attr, data);
    }
}


template <typename T>
bool AttributeScopeStack<T>::empty() const
{
    return !bool(m_start);
}

template <typename T>
int64_t AttributeScopeStack<T>::depth() const
{
    if (bool(m_start)) {
        return m_start->depth();
    } else {
        return 0;
    }
}

template <typename T>
AttributeScope<T>* AttributeScopeStack<T>::active_scope_ptr()
{
    return m_active;
}

template <typename T>
const AttributeScope<T>* AttributeScopeStack<T>::active_scope_ptr() const
{
    return m_active;
}

template <typename T>
void AttributeScopeStack<T>::clear_current_scope(Attribute<T>& attr)
{
    assert(writing_enabled());

    if (bool(m_start)) {
#if defined(WMTK_FLUSH_ON_FAIL)


        m_start->flush(attr);
#else
        m_start->clear();
#endif
    }
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
    assert(m_active != nullptr);
    m_active = m_active->previous();
}

template <typename T>
void AttributeScopeStack<T>::change_to_next_scope() const
{
#if defined(WMTK_FLUSH_ON_FAIL)
    if (m_active == nullptr) {
        assert(!empty());
        m_active = m_start.get();
        return;
    }
#endif
    m_active = m_active->next();
}
template <typename T>
void AttributeScopeStack<T>::change_to_current_scope() const
{
#if defined(WMTK_FLUSH_ON_FAIL)
    m_active = nullptr;
#else
    m_active = m_start.get();
#endif
}
template <typename T>
bool AttributeScopeStack<T>::at_current_scope() const
{
#if defined(WMTK_FLUSH_ON_FAIL)
    return m_active == nullptr;
#else
    // either we do not have a stack or active is the start
    return !(bool(m_start)) || m_active == m_start.get();
#endif
}
template <typename T>
bool AttributeScopeStack<T>::writing_enabled() const
{
    return at_current_scope();
}


//template class AttributeScopeStack<int64_t>;
//template class AttributeScopeStack<double>;
//template class AttributeScopeStack<char>;
//template class AttributeScopeStack<Rational>;
} // namespace wmtk::attribute
