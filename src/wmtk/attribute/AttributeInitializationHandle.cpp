#include "AttributeInitializationHandle.hpp"


namespace wmtk::attribute {
AttributeInitializationHandleBase::AttributeInitializationHandleBase() {}
AttributeInitializationHandleBase::~AttributeInitializationHandleBase() = default;

template class AttributeInitializationHandle<double>;
template class AttributeInitializationHandle<int64_t>;
template class AttributeInitializationHandle<char>;
template class AttributeInitializationHandle<Rational>;
} // namespace wmtk::attribute
