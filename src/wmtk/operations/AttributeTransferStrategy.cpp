#include "AttributeTransferStrategy.hpp"


namespace wmtk::operations {

    PrimitiveType AttributeTransferStrategy::primitive_type() const {
        return m_handle.primitive_type();
    }
    Mesh& AttributeTransferStrategy::mesh() {
        return m_handle.mesh();
    }

template class AttributeTransferStrategy<double>;
template class AttributeTransferStrategy<long>;
template class AttributeTransferStrategy<char>;
template class AttributeTransferStrategy<Rational>;
} // namespace wmtk::operations
