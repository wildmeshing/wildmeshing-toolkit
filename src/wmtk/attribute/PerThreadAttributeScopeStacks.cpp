#include "PerThreadAttributeScopeStacks.hpp"
#include <wmtk/utils/Rational.hpp>


namespace wmtk::attribute {
template class PerThreadAttributeScopeStacks<long>;
template class PerThreadAttributeScopeStacks<double>;
template class PerThreadAttributeScopeStacks<char>;
template class PerThreadAttributeScopeStacks<Rational>;

template <typename T>
void PerThreadAttributeScopeStacks<T>::emplace()
{
    local().emplace();
}
template <typename T>
void PerThreadAttributeScopeStacks<T>::pop(Attribute<T>& attribute, bool apply_updates)
{
    local().pop(attribute, apply_updates);
}

template <typename T>
bool PerThreadAttributeScopeStacks<T>::empty() const
{
    return local().empty();
}

template <typename T>
long PerThreadAttributeScopeStacks<T>::depth() const
{
    return local().depth();
}

template <typename T>
AttributeScope<T>* PerThreadAttributeScopeStacks<T>::current_scope_ptr()
{
    return local().current_scope_ptr();
}

template <typename T>
const AttributeScope<T>* PerThreadAttributeScopeStacks<T>::current_scope_ptr() const
{
    return local().current_scope_ptr();
}

template <typename T>
void PerThreadAttributeScopeStacks<T>::clear_current_scope()
{
    local().clear_current_scope();
    ;
}

template <typename T>
long PerThreadAttributeScopeStacks<T>::add_checkpoint()
{
    return local().add_checkpoint();
}
template <typename T>
AttributeScope<T> const* PerThreadAttributeScopeStacks<T>::get_checkpoint(long index) const
{
    return local().get_checkpoint(index);
}
} // namespace wmtk::attribute
