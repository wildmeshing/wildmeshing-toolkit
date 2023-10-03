#pragma once
#include <wmtk/Primitive.hpp>
#include <wmtk/operations/tri_mesh/VertexAttributesUpdateBase.hpp>
#include <wmtk/operations/tri_mesh/VertexSmoothNewtonMethod.hpp>
#include <wmtk/operations/tri_mesh/VertexSmoothNewtonMethodWithLineSearch.hpp>
#include <wmtk/operations/tri_mesh/VertexSmoothUsingDifferentiableEnergy.hpp>
#include "Operation.hpp"
#include "OperationFactory.hpp"

namespace wmtk {
namespace operations {

class OperationDifferentiableSmoothFactory : public OperationFactoryBase
{
public:
    OperationDifferentiableSmoothFactory(
        const OperationSettings<tri_mesh::VertexSmoothUsingDifferentiableEnergy>& settings)
        : OperationFactoryBase(tri_mesh::VertexSmoothUsingDifferentiableEnergy::primitive_type())
        , m_settings(settings)
    {}

    std::unique_ptr<Operation> create(wmtk::Mesh& m, const Tuple& t) const override
    {
        if (m_settings.second_order) {
            if (!m_settings.line_search) {
                return std::make_unique<tri_mesh::VertexSmoothNewtonMethod>(m, t, m_settings);
            }
            if (m_settings.line_search) {
                return std::make_unique<tri_mesh::VertexSmoothNewtonMethodWithLineSearch>(
                    m,
                    t,
                    m_settings);
            }
        }
        return std::make_unique<tri_mesh::VertexSmoothUsingDifferentiableEnergy>(m, t, m_settings);
    }

protected:
    const OperationSettings<tri_mesh::VertexSmoothUsingDifferentiableEnergy>& m_settings;
};
} // namespace operations
} // namespace wmtk