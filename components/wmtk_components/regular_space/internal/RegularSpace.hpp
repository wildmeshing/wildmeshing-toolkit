#pragma once

#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::components::internal {

class RegularSpace
{
    TriMesh& m_mesh;
    bool m_lock_boundary = true;

    MeshAttributeHandle<double> m_position_handle;

    Scheduler m_scheduler;

public:
    RegularSpace(TriMesh& mesh, const bool lock_boundary);

    void process(const long iterations);
};

} // namespace wmtk::components::internal
