
#include <wmtk/Mesh.hpp>
//
#include <wmtk/attribute/utils/variant_comparison.hpp>
#include "AttributeTransferStrategy.hpp"


namespace wmtk::operations {

template <typename T>
AttributeTransferStrategy<T>::AttributeTransferStrategy(
    const attribute::MeshAttributeHandle& handle)
    : AttributeTransferStrategyBase(handle)
{}

template <typename T>
PrimitiveType AttributeTransferStrategy<T>::primitive_type() const
{
    return handle().primitive_type();
}
template <typename T>
Mesh& AttributeTransferStrategy<T>::mesh()
{
    return handle().mesh();
}


template class AttributeTransferStrategy<double>;
template class AttributeTransferStrategy<int64_t>;
template class AttributeTransferStrategy<char>;
template class AttributeTransferStrategy<Rational>;
} // namespace wmtk::operations
