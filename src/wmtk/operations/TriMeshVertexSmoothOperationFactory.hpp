#pragma once

#include "OperationFactory.hpp"
#include "TriMeshVertexSmoothOperation.hpp"

namespace wmtk {
template <>
class OperationFactory<TriMeshVertexSmoothOperation> : public OperationFactoryBase
{
public:
    OperationFactory()
        : OperationFactoryBase(TriMeshVertexSmoothOperation::primitive_type())
    {}
    std::unique_ptr<Operation> create(Mesh& m, const Tuple& t) const override
    {
        if (!m_position_handle) {
            m_position_handle = std::make_unique<MeshAttributeHandle<double>>(
                m.get_attribute_handle<double>("position", PrimitiveType::Vertex));
        }
        return std::make_unique<TriMeshVertexSmoothOperation>(m, t, *m_position_handle);
    }

private:
    mutable std::unique_ptr<MeshAttributeHandle<double>> m_position_handle;
};
} // namespace wmtk