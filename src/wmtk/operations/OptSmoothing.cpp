#include "OptSmoothing.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/Mesh.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/TriangleInversionInvariant.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk::operations {
void OperationSettings<OptSmoothing>::create_invariants()
{
    OperationSettings<AttributesUpdateBase>::create_invariants();
    // if (!smooth_boundary) {
    //     invariants->add(std::make_unique<InteriorVertexInvariant>(m_mesh));
    // }

    invariants->add(std::make_shared<TriangleInversionInvariant>(m_mesh, coordinate_handle));
}

OptSmoothing::OptSmoothing(
    Mesh& m,
    const Simplex& t,
    const OperationSettings<OptSmoothing>& settings)
    : AttributesUpdateBase(m, t, settings)
    , m_settings{settings}
{}

std::string OptSmoothing::name() const
{
    return "opt_smoothing";
}

Accessor<double> OptSmoothing::coordinate_accessor()
{
    return mesh().create_accessor(m_settings.coordinate_handle);
}
ConstAccessor<double> OptSmoothing::const_coordinate_accessor() const
{
    return mesh().create_const_accessor(m_settings.coordinate_handle);
}

bool OptSmoothing::execute()
{
    if (!AttributesUpdateBase::execute()) return false;
    return true;
}

} // namespace wmtk::operations
