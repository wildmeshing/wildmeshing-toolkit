#pragma once


#include "internal/AttributeTransactionStack.hpp"
namespace wmtk::attribute {
template <typename T>
class PerThreadAttributeScopeStacks
{
public:
    PerThreadAttributeScopeStacks() = default;
    PerThreadAttributeScopeStacks(PerThreadAttributeScopeStacks&&) = default;
    PerThreadAttributeScopeStacks& operator=(PerThreadAttributeScopeStacks&&) = default;
    internal::AttributeTransactionStack<T>& local();
    const internal::AttributeTransactionStack<T>& local() const;

private:
    // single stack so far
    mutable internal::AttributeTransactionStack<T> m_stack;
};


template <typename T>
inline internal::AttributeTransactionStack<T>& PerThreadAttributeScopeStacks<T>::local()
{
    return m_stack;
}
template <typename T>
inline const internal::AttributeTransactionStack<T>& PerThreadAttributeScopeStacks<T>::local() const
{
    return m_stack;
}
} // namespace wmtk::attribute
