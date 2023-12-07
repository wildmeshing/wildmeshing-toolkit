#include "VertexSmoothUsingDifferentiableEnergy.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/TriangleInversionInvariant.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk::operations {
void OperationSettings<tri_mesh::VertexSmoothUsingDifferentiableEnergy>::initialize_invariants(
    const TriMesh& m)
{
    base_settings.initialize_invariants(m);
    base_settings.invariants.add(
        std::make_shared<TriangleInversionInvariant>(m, coordinate_handle));
    if (!smooth_boundary) {
        base_settings.invariants.add(std::make_unique<InteriorVertexInvariant>(m));
    }
}
} // namespace wmtk::operations

namespace wmtk::operations::tri_mesh {
VertexSmoothUsingDifferentiableEnergy::VertexSmoothUsingDifferentiableEnergy(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<VertexSmoothUsingDifferentiableEnergy>& settings)
    : VertexAttributesUpdateBase(m, t, settings.base_settings)
    , m_settings{settings}
{}

std::string VertexSmoothUsingDifferentiableEnergy::name() const
{
    return "tri_mesh_vertex_smooth_using_differentiable_energy";
}

function::utils::DifferentiableFunctionEvaluator
VertexSmoothUsingDifferentiableEnergy::get_function_evaluator(Accessor<double>& accessor) const
{
    assert(accessor.mesh().is_valid_slow(input_tuple()));
    auto evaluator = function::utils::DifferentiableFunctionEvaluator(
        *m_settings.energy,
        accessor,
        simplex::Simplex(PrimitiveType::Vertex, input_tuple()));
    const Simplex& simplex = evaluator.simplex();
    Tuple tuple = evaluator.simplex().tuple();
    assert(accessor.mesh().is_valid_slow(evaluator.simplex().tuple()));
    spdlog::info("evaluator {}", (long)&evaluator);
    return evaluator;
}


Accessor<double> VertexSmoothUsingDifferentiableEnergy::coordinate_accessor()
{
    return mesh().create_accessor(m_settings.coordinate_handle);
}
ConstAccessor<double> VertexSmoothUsingDifferentiableEnergy::const_coordinate_accessor() const
{
    return mesh().create_const_accessor(m_settings.coordinate_handle);
}
} // namespace wmtk::operations::tri_mesh