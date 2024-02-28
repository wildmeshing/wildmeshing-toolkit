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
    // single stack so far
    mutable AttributeScopeStack<T> m_stack;
};


template <typename T>
inline AttributeScopeStack<T>& PerThreadAttributeScopeStacks<T>::local()
{
    return m_stack;
}
template <typename T>
inline const AttributeScopeStack<T>& PerThreadAttributeScopeStacks<T>::local() const
{
    return m_stack;
}
} // namespace wmtk::attribute
