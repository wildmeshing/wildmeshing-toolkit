#include "VertexSmoothUsingDifferentiableEnergyFactory.hpp"
#include <wmtk/operations/tri_mesh/VertexAttributesUpdateBase.hpp>
#include <wmtk/operations/tri_mesh/internal/SeamlessGradientDescent.hpp>
#include <wmtk/operations/tri_mesh/internal/VertexSmoothGradientDescent.hpp>
#include <wmtk/operations/tri_mesh/internal/VertexSmoothNewtonMethod.hpp>
#include <wmtk/operations/tri_mesh/internal/VertexSmoothNewtonMethodWithLineSearch.hpp>

namespace wmtk::operations {

template <>

std::unique_ptr<Operation> OperationFactory<
    tri_mesh::VertexSmoothUsingDifferentiableEnergy>::create(wmtk::Mesh& m, const Tuple& t) const
{
    if (m_settings.do_seamless_optimization) {
        if (!m_settings.uv_handle.has_value()) {
            throw std::runtime_error("uv_handle is not set");
        }
        return std::make_unique<tri_mesh::internal::SeamlessGradientDescent>(
            static_cast<TriMesh&>(m),
            m_settings.uv_mesh_ptr,
            m_settings.uv_handle.value(),
            Simplex(tri_mesh::VertexSmoothUsingDifferentiableEnergy::primitive_type(), t),
            m_settings);
    }
    if (m_settings.second_order) {
        if (m_settings.line_search) {
            return std::make_unique<tri_mesh::internal::VertexSmoothNewtonMethodWithLineSearch>(
                m,
                Simplex(tri_mesh::VertexSmoothUsingDifferentiableEnergy::primitive_type(), t),
                m_settings);
        } else {
            return std::make_unique<tri_mesh::internal::VertexSmoothNewtonMethod>(
                m,
                Simplex(tri_mesh::VertexSmoothUsingDifferentiableEnergy::primitive_type(), t),
                m_settings);
        }
    }
    return std::make_unique<tri_mesh::internal::VertexSmoothGradientDescent>(
        m,
        Simplex(tri_mesh::VertexSmoothUsingDifferentiableEnergy::primitive_type(), t),
        m_settings);
}
} // namespace wmtk::operations
