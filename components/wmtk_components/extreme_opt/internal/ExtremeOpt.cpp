#include "ExtremeOpt.hpp"
#include <predicates.h>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/operations/tri_mesh/EdgeCollapseToMidpoint.hpp>
#include <wmtk/operations/tri_mesh/EdgeSwapValence.hpp>
#include <wmtk/operations/tri_mesh/ExtremeOptSplit.hpp>
#include <wmtk/operations/tri_mesh/VertexTangentialLaplacianSmooth.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::internal {

ExtremeOpt::ExtremeOpt(
    std::string mesh_name,
    TriMesh& mesh,
    const double length,
    const bool lock_boundary,
    const bool preserve_childmesh_topology,
    const bool preserve_childmesh_geometry,
    const bool do_split,
    const bool do_collapse,
    const bool do_swap,
    const bool do_smooth,
    const bool debug_output)
    : m_mesh_name{mesh_name}
    , m_mesh{mesh}
    , m_length_min{(4. / 5.) * length}
    , m_length_max{(4. / 3.) * length}
    , m_lock_boundary{lock_boundary}
    , m_preserve_childmesh_topology{preserve_childmesh_topology}
    , m_preserve_childmesh_geometry{preserve_childmesh_geometry}
    , m_do_split{do_split}
    , m_do_collapse{do_collapse}
    , m_do_swap{do_swap}
    , m_do_smooth{do_smooth}
    , m_debug_output{debug_output}
    , m_position_handle{m_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex)}
    // TODO: should pass in a handle
    , m_scheduler(m_mesh)
{
    using namespace operations;

    auto child_meshes = m_mesh.get_child_meshes();
    m_uv_mesh_ptr = std::static_pointer_cast<TriMesh>(child_meshes[0]);
    m_uv_handle = m_uv_mesh_ptr->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    // split
    {
        OperationSettings<tri_mesh::ExtremeOptSplit> op_settings;
        op_settings.position = m_position_handle;
        // thresholds are inverted because we continue splitting because we
        // always split until we're under this length, which is the max
        // required length for the op to happen
        op_settings.min_squared_length = m_length_max * m_length_max;
        op_settings.split_settings.split_boundary_edges = !m_lock_boundary;
        op_settings.uv_mesh_ptr = m_uv_mesh_ptr;
        op_settings.uv_handle = m_uv_handle;
        op_settings.initialize_invariants(m_mesh);

        m_scheduler.add_operation_type<tri_mesh::ExtremeOptSplit>("split", op_settings);
    }
    // collapse
    {
        OperationSettings<tri_mesh::EdgeCollapseToMidpoint> op_settings;
        op_settings.position = m_position_handle;
        // thresholds are inverted because we continue collapsing because we
        // always collapse until we're over this length, which is the minimum
        // required length for the op to happen
        op_settings.max_squared_length = m_length_min * m_length_min;
        op_settings.collapse_settings.collapse_boundary_edges = !m_lock_boundary;
        op_settings.collapse_settings.preserve_topology = m_preserve_childmesh_topology;
        op_settings.collapse_settings.preserve_geometry = m_preserve_childmesh_geometry;
        op_settings.collapse_towards_boundary = true;
        op_settings.initialize_invariants(m_mesh);


        m_scheduler.add_operation_type<tri_mesh::EdgeCollapseToMidpoint>("collapse", op_settings);
    }
    // flip
    {
        OperationSettings<tri_mesh::EdgeSwapValence> op_settings;
        op_settings.base_settings.collapse_settings.preserve_topology =
            m_preserve_childmesh_topology;
        op_settings.base_settings.collapse_settings.preserve_geometry =
            m_preserve_childmesh_geometry;
        op_settings.position = m_position_handle;
        op_settings.initialize_invariants(m_mesh);

        m_scheduler.add_operation_type<tri_mesh::EdgeSwapValence>("swap", op_settings);
    }
    // smooth
    {
        OperationSettings<tri_mesh::VertexTangentialLaplacianSmooth> op_settings;
        op_settings.smooth_settings.position = m_position_handle;
        op_settings.smooth_settings.smooth_boundary = false;
        // op_settings.smooth_settings.base_settings.initialize_invariants(m_mesh);

        op_settings.initialize_invariants(m_mesh);

        m_scheduler.add_operation_type<tri_mesh::VertexTangentialLaplacianSmooth>(
            "smooth",
            op_settings);
    }
}

void ExtremeOpt::write_debug_mesh(const long test_id)
{
    ParaviewWriter writer(
        "extreme_opt_" + m_mesh_name + "_seamed_" + std::to_string(test_id),
        "vertices",
        m_mesh,
        true,
        true,
        true,
        false);

    m_mesh.serialize(writer);

    ParaviewWriter writer_uv(
        "extreme_opt_" + m_mesh_name + "_cut_" + std::to_string(test_id),
        "vertices",
        *(m_uv_mesh_ptr),
        true,
        true,
        true,
        false);

    m_uv_mesh_ptr->serialize(writer_uv);
}

void ExtremeOpt::remeshing(const long iterations)
{
    exactinit();
    auto parent_vertex_handle =
        m_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto parent_vertex_accessor = m_mesh.create_accessor(parent_vertex_handle);

    // debug write
    if (m_debug_output) {
        write_debug_mesh(0);
    }

    for (long i = 0; i < iterations; ++i) {
        bool is_conn_valid;
        bool is_map_valid;


        wmtk::logger().info("Iteration {}", i);

        if (m_do_split) {
            m_scheduler.run_operation_on_all(PrimitiveType::Edge, "split");
            wmtk::logger().info("Done split {}\n", i);
        }

        // debug write
        if (m_debug_output) {
            write_debug_mesh(4 * i + 1);
        }

        if (m_do_collapse) {
            m_scheduler.run_operation_on_all(PrimitiveType::Edge, "collapse");
            wmtk::logger().info("Done collapse {}\n", i);
        }

        // debug write
        if (m_debug_output) {
            write_debug_mesh(4 * i + 2);
        }

        if (m_do_swap) {
            m_scheduler.run_operation_on_all(PrimitiveType::Edge, "swap");
            wmtk::logger().info("Done swap {}\n", i);
        }

        // debug write
        if (m_debug_output) {
            write_debug_mesh(4 * i + 3);
        }

        if (m_do_smooth) {
            m_scheduler.run_operation_on_all(PrimitiveType::Vertex, "smooth");
            wmtk::logger().info("Done smooth {}\n", i);
        }

        // debug write
        if (m_debug_output) {
            write_debug_mesh(4 * i + 4);
        }
    } // end for
}

} // namespace wmtk::components::internal
