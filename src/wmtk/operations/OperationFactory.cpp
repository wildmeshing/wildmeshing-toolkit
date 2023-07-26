#include "OperationFactory.hpp"


namespace wmtk {
OperationFactoryBase::OperationFactoryBase(PrimitiveType t)
    : m_primitive(t)
{}
OperationFactoryBase::~OperationFactoryBase() = default;
} // namespace wmtk
