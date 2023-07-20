#pragma once
#include "Operation.hpp"
#include "wmtk/Primitive.hpp"
#include "wmtk/Tuple.hpp"

namespace wmtk::operations {
class OperationFactoryBase
{
    // OperationFactoryBase(PrimitiveType p): m_primitive(p) {}
    virtual std::unique_ptr<Operation> create(const Tuple& t) const = 0;
    // PrimitiveType primitive() const { return m_primitive; }
    // private:
    // PrimitiveType m_primitive;
    Mesh& m_mesh;
};

template <typename OperationType>
class OperationFactory : public OperationFactoryBase
{
    std::unique_ptr<Operation> create(const Tuple& t) const
    {
        return std::make_unique<OperationType>(m_mesh, t);
    }
};


} // namespace wmtk::operations
