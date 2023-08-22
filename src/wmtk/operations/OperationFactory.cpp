#include "OperationFactory.hpp"


namespace wmtk::operations {
OperationFactoryBase::OperationFactoryBase(PrimitiveType t)
    : m_primitive(t)
{}
OperationFactoryBase::~OperationFactoryBase() = default;
} // namespace wmtk::operations
