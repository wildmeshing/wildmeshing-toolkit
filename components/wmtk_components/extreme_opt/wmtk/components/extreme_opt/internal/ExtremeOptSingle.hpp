#pragma once

#include <string>
#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>

namespace wmtk::components::internal {

class ExtremeOptSingle
{
    std::string m_mesh_name = "";
    TriMesh& m_mesh;
    double m_length_min = std::numeric_limits<double>::max();
    double m_length_max = std::numeric_limits<double>::lowest();
    bool m_do_split;
    bool m_do_collapse;
    bool m_do_swap;
    bool m_do_smooth;
    bool m_debug_output;

    attribute::MeshAttributeHandle m_position_handle;
    attribute::MeshAttributeHandle m_uv_handle;


    Scheduler m_scheduler;

public:
    ExtremeOptSingle(
        std::string mesh_name,
        TriMesh& mesh,
        const double length,
        const bool do_split,
        const bool do_collapse,
        const bool do_swap,
        const bool do_smooth,
        const bool debug_output);

    void get_boundary_mesh();
    void remeshing(const long iterations);
    void write_debug_mesh(const long test_id);
};

} // namespace wmtk::components::internal
