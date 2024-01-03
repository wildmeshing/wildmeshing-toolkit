#include "VertexSmoothUsingDifferentiableEnergy.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/TriangleInversionInvariant.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk::operations {
void OperationSettings<tri_mesh::VertexSmoothUsingDifferentiableEnergy>::create_invariants()
{
    OperationSettings<tri_mesh::VertexAttributesUpdateBase>::create_invariants();
    if (!smooth_boundary) {
        invariants->add(std::make_unique<InteriorVertexInvariant>(m_mesh));
    }
    // NOTE!!! deleted by leyi, we need to add this invariant by ourselves
    // invariants->add(std::make_shared<TriangleInversionInvariant>(m_mesh, coordinate_handle));
}

namespace tri_mesh {
VertexSmoothUsingDifferentiableEnergy::VertexSmoothUsingDifferentiableEnergy(
    Mesh& m,
    const Simplex& t,
    const OperationSettings<VertexSmoothUsingDifferentiableEnergy>& settings)
    : VertexAttributesUpdateBase(m, t, settings)
    , m_settings{settings}
{
    assert(t.primitive_type() == PrimitiveType::Vertex);
}

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
        input_simplex());
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

} // namespace tri_mesh
} // namespace wmtk::operations
