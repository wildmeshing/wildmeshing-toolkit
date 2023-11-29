#pragma once

#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::components::internal {

class IsotropicRemeshing
{
    TriMesh& m_mesh;
    double m_length_min = std::numeric_limits<double>::max();
    double m_length_max = std::numeric_limits<double>::lowest();
    bool m_lock_boundary = true;
    bool m_preserve_childmesh_topology = false;
    bool m_preserve_childmesh_geometry = false;

    MeshAttributeHandle<double> m_position_handle;
    Scheduler m_scheduler;

public:
    IsotropicRemeshing(
        TriMesh& mesh,
        const double length,
        const bool lock_boundary,
        const bool preserve_childmesh_topology,
        const bool preserve_childmesh_geometry);

    void remeshing(const long iterations);
};

} // namespace wmtk::components::internal
