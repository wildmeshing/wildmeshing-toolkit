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
    bool m_do_split = true;
    bool m_do_collapse = true;
    bool m_do_swap = true;
    bool m_do_smooth = true;
    bool m_debug_output = false;

    MeshAttributeHandle<double> m_position_handle;
    Scheduler m_scheduler;

public:
    IsotropicRemeshing(
        TriMesh& mesh,
        const double length,
        const bool lock_boundary,
        const bool preserve_childmesh_topology,
        const bool preserve_childmesh_geometry,
        const bool do_split,
        const bool do_collapse,
        const bool do_swap,
        const bool do_smooth,
        const bool debug_output);

    void remeshing(const int64_t iterations);
};

} // namespace wmtk::components::internal
