#include "AttributeTransferStrategy.hpp"


namespace wmtk::operations {

template <typename T>
AttributeTransferStrategy<T>::AttributeTransferStrategy(const attribute::MeshAttributeHandle<T>& handle): m_handle(handle)
{
}

template <typename T>
PrimitiveType AttributeTransferStrategy<T>::primitive_type() const
{
    return m_handle.primitive_type();
}
template <typename T>
Mesh& AttributeTransferStrategy<T>::mesh()
{
    return m_handle.mesh();
}

template class AttributeTransferStrategy<double>;
template class AttributeTransferStrategy<long>;
template class AttributeTransferStrategy<char>;
template class AttributeTransferStrategy<Rational>;
} // namespace wmtk::operations
