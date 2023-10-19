#pragma once

#include "AttributeScopeStack.hpp"


namespace wmtk::attribute {

template <typename T>
class PerThreadAttributeScopeStacks
{
public:
    AttributeScopeStack<T>& local() { return m_stacks; }
    const AttributeScopeStack<T>& local() const { return m_stacks; }
    mutable AttributeScopeStack<T> m_stacks;
};
} // namespace wmtk::attribute
