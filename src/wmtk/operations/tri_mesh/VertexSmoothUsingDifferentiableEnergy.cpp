#include "VertexSmoothUsingDifferentiableEnergy.hpp"
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/TriangleInversionInvariant.hpp>

namespace wmtk::operations {
void OperationSettings<tri_mesh::VertexSmoothUsingDifferentiableEnergy>::initialize_invariants(
    const TriMesh& m)
{
    base_settings.initialize_invariants(m);
    base_settings.invariants.add(
        std::make_shared<TriangleInversionInvariant>(m, coordinate_handle));
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

template <int Dim>
optimization::FunctionInterface<Dim> VertexSmoothUsingDifferentiableEnergy::get_function_interface(
    Accessor<double>& accessor) const
{
    return optimization::FunctionInterface(input_tuple(), accessor, *m_settings.energy);
}
template <>
optimization::FunctionInterface<2> VertexSmoothUsingDifferentiableEnergy::get_function_interface<2>(
    Accessor<double>& accessor) const;

template <>
optimization::FunctionInterface<3> VertexSmoothUsingDifferentiableEnergy::get_function_interface<3>(
    Accessor<double>& accessor) const;


Accessor<double> VertexSmoothUsingDifferentiableEnergy::coordinate_accessor()
{
    return mesh().create_accessor(m_settings.coordinate_handle);
}
ConstAccessor<double> VertexSmoothUsingDifferentiableEnergy::const_coordinate_accessor() const
{
    return mesh().create_const_accessor(m_settings.coordinate_handle);
}
} // namespace wmtk::operations::tri_mesh
