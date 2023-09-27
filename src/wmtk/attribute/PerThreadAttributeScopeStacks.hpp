#pragma once
#include <tbb/enumerable_thread_specific.h>
#include "AttributeScopeStack.hpp"


namespace wmtk::attribute {

template <typename T>
struct PerThreadAttributeScopeStacks
{
    AttributeScopeStack<T>& local() { return m_stacks.local(); }
    const AttributeScopeStack<T>& local() const { return m_stacks.local(); }
    mutable tbb::enumerable_thread_specific<AttributeScopeStack<T>> m_stacks;
};
} // namespace wmtk::attribute
