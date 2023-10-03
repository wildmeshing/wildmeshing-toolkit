#pragma once

#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::components::internal {

class IsosurfaceExtraction
{
    TriMesh& m_mesh;
    double m_length_min = std::numeric_limits<double>::max();
    double m_length_max = std::numeric_limits<double>::lowest();
    bool m_lock_boundary = true;

    MeshAttributeHandle<double> m_position_handle;
    MeshAttributeHandle<long> m_tag_handle;
    Scheduler m_scheduler;
    long input_tag_value;
    long embedding_tag_value;
    long offset_tag_value;

public:
    IsosurfaceExtraction(
        TriMesh& mesh,
        const double length,
        const bool lock_boundary,
        long input_tag_value,
        long embedding_tag_value,
        long offset_tag_value);

    void process(const long iteration_times);
};

} // namespace wmtk::components::internal