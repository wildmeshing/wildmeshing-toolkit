#include "VertexSmoothUsingDifferentiableEnergyFactory.hpp"
#include <wmtk/operations/tri_mesh/VertexAttributesUpdateBase.hpp>
#include <wmtk/operations/tri_mesh/internal/VertexSmoothGradientDescent.hpp>
#include <wmtk/operations/tri_mesh/internal/VertexSmoothNewtonMethod.hpp>
#include <wmtk/operations/tri_mesh/internal/VertexSmoothNewtonMethodWithLineSearch.hpp>


namespace wmtk::operations {

template <>

std::unique_ptr<Operation> OperationFactory<
    tri_mesh::VertexSmoothUsingDifferentiableEnergy>::create(wmtk::Mesh& m, const Tuple& t) const
{
    if (m_settings.second_order) {
        if (m_settings.line_search) {
            return std::make_unique<tri_mesh::internal::VertexSmoothNewtonMethodWithLineSearch>(
                m,
                t,
                m_settings);
        } else {
            return std::make_unique<tri_mesh::internal::VertexSmoothNewtonMethod>(m, t, m_settings);
        }
    }
    return std::make_unique<tri_mesh::internal::VertexSmoothGradientDescent>(m, t, m_settings);
}
} // namespace wmtk::operations
