#pragma once
#include <tbb/enumerable_thread_specific.h>
#include "AttributeScopeStack.hpp"


namespace wmtk::attribute {

template <typename T>
class PerThreadAttributeScopeStacks
{
    public:
    void emplace();
    void pop(Attribute<T>& attribute, bool apply_updates);
    AttributeScope<T>* current_scope_ptr();
    const AttributeScope<T>* current_scope_ptr() const;

    bool empty() const;
    void clear_current_scope();

    long depth() const;
    long add_checkpoint();
    AttributeScope<T> const* get_checkpoint(long index) const;

    AttributeScopeStack<T>& local() { return m_stacks.local(); }
    const AttributeScopeStack<T>& local() const { return m_stacks.local(); }
    private:
    mutable tbb::enumerable_thread_specific<AttributeScopeStack<T>> m_stacks;
};
} // namespace wmtk::attribute
