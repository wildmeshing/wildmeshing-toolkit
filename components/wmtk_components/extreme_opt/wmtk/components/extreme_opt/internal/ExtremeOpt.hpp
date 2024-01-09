#pragma once

#include <string>
#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>

namespace wmtk::components::internal {

class ExtremeOpt
{
    std::string m_mesh_name = "";
    TriMesh& m_mesh;
    std::shared_ptr<TriMesh> m_uv_mesh_ptr;
    double m_length_min = std::numeric_limits<double>::max();
    double m_length_max = std::numeric_limits<double>::lowest();
    bool m_lock_boundary;
    bool m_do_split;
    bool m_do_collapse;
    bool m_collapse_optimize_E_max;
    bool m_do_swap;
    bool m_swap_optimize_E_max;
    bool m_do_smooth;
    bool m_debug_output;

    attribute::MeshAttributeHandle m_position_handle;
    attribute::MeshAttributeHandle m_uv_handle;


    Scheduler m_scheduler;

public:
    ExtremeOpt(
        std::string mesh_name,
        TriMesh& mesh,
        const double length,
        const bool lock_boundary,
        const bool do_split,
        const bool do_collapse,
        const bool collapse_optimize_E_max,
        const bool do_swap,
        const bool swap_optimize_E_max,
        const bool do_smooth,
        const bool debug_output);
    int vertex_importance(const simplex::Simplex& s_child);
    bool need_to_keep_in_child_mesh(const simplex::Simplex& s_child);
    void remeshing(const long iterations);
    void write_debug_mesh(const long test_id);
};

} // namespace wmtk::components::internal
