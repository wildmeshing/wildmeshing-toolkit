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
    MeshAttributeHandle<long> m_vertex_tag_handle;
    MeshAttributeHandle<long> m_edge_tag_handle;
    MeshAttributeHandle<long> m_split_todo_handle;
    Scheduler m_scheduler;
    long input_tag_value;
    long embedding_tag_value;
    long offset_tag_value;
    double offset_distance;

public:
    IsosurfaceExtraction(
        TriMesh& mesh,
        const double length,
        const bool lock_boundary,
        long input_tag_value,
        long embedding_tag_value,
        long offset_tag_value,
        double offset_distance);

    void generate_offset_todo_tags(bool isDifferent);

    void process(const long iteration_times);
};

} // namespace wmtk::components::internal