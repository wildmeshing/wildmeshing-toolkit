#include "VertexSmoothing.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/TriangleInversionInvariant.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk::operations {
void OperationSettings<tri_mesh::VertexSmoothing>::initialize_invariants(const TriMesh& m)
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
VertexSmoothing::VertexSmoothing(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<VertexSmoothing>& settings)
    : VertexAttributesUpdateBase(m, t, settings.base_settings)
    , m_settings{settings}
{}

std::string VertexSmoothing::name() const
{
    return "tri_mesh_vertex_smoothing";
}

Accessor<double> VertexSmoothing::coordinate_accessor()
{
    return mesh().create_accessor(m_settings.coordinate_handle);
}
ConstAccessor<double> VertexSmoothing::const_coordinate_accessor() const
{
    return mesh().create_const_accessor(m_settings.coordinate_handle);
}
} // namespace wmtk::operations::tri_mesh
