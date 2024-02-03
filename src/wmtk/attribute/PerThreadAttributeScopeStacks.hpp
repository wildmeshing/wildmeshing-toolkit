#pragma once

#include "AttributeScopeStack.hpp"


namespace wmtk::attribute {

template <typename T>
class PerThreadAttributeScopeStacks
{
public:
    PerThreadAttributeScopeStacks() = default;
    PerThreadAttributeScopeStacks(PerThreadAttributeScopeStacks&&) = default;
    PerThreadAttributeScopeStacks& operator=(PerThreadAttributeScopeStacks&&) = default;
    AttributeScopeStack<T>& local();
    const AttributeScopeStack<T>& local() const;

private:
    mutable AttributeScopeStack<T> m_stacks;
};


template <typename T>
AttributeScopeStack<T>& PerThreadAttributeScopeStacks<T>::local()
{
    return m_stacks;
}
template <typename T>
const AttributeScopeStack<T>& PerThreadAttributeScopeStacks<T>::local() const
{
    return m_stacks;
}
} // namespace wmtk::attribute
