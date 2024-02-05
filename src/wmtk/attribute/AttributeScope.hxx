#include <wmtk/utils/Rational.hpp>
#include "AttributeScope.hpp"
namespace wmtk::attribute {

template <typename T>
AttributeScope<T>::AttributeScope()
{}
template <typename T>
AttributeScope<T>::~AttributeScope()
{}
template <typename T>
AttributeScope<T>::AttributeScope(std::unique_ptr<AttributeScope>&& next)
{}


template <typename T>
void AttributeScope<T>::apply(Attribute<T>& attr) const
{
    AttributeCache<T>::apply_to(attr);
}
template <typename T>
void AttributeScope<T>::apply(const Attribute<T>& attr, std::vector<T>& data)
{
    AttributeCache<T>::apply_to(attr, data);
}


// template class AttributeScope<int64_t>;
// template class AttributeScope<double>;
// template class AttributeScope<char>;
// template class AttributeScope<Rational>;
} // namespace wmtk::attribute
