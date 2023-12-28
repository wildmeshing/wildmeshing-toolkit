#include "ExtremeOpt.hpp"
#include <predicates.h>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/SimplicialComplex.hpp>
// #include <wmtk/operations/tri_mesh/EdgeCollapseToMidpoint.hpp>
#include <wmtk/operations/tri_mesh/ExtremeOptCollapse.hpp>
#include <wmtk/operations/tri_mesh/ExtremeOptSplit.hpp>
#include <wmtk/operations/tri_mesh/ExtremeOptSwap.hpp>

#include <wmtk/function/SYMDIR.hpp>
#include <wmtk/operations/tri_mesh/VertexSmoothUsingDifferentiableEnergy.hpp>
#include <wmtk/operations/tri_mesh/VertexTangentialLaplacianSmooth.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/SeamlessConstraints.hpp>
// TODO: lock boundary don't work for uv mesh now
namespace wmtk::components::internal {

ExtremeOpt::ExtremeOpt(
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
    const bool debug_output)
    : m_mesh_name{mesh_name}
    , m_mesh{mesh}
    , m_length_min{(4. / 5.) * length}
    , m_length_max{(4. / 3.) * length}
    , m_lock_boundary{lock_boundary}
    , m_do_split{do_split}
    , m_do_collapse{do_collapse}
    , m_collapse_optimize_E_max{collapse_optimize_E_max}
    , m_do_swap{do_swap}
    , m_swap_optimize_E_max{swap_optimize_E_max}
    , m_do_smooth{do_smooth}
    , m_debug_output{debug_output}
    , m_position_handle{m_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex)}
    , m_scheduler(m_mesh)
    , m_scheduler_uv(*m_mesh.get_child_meshes()[0])
{
    using namespace operations;

    // get uv mesh and uv coordinates handle
    auto child_meshes = m_mesh.get_child_meshes();
    m_uv_mesh_ptr = std::static_pointer_cast<TriMesh>(child_meshes[0]);
    m_uv_handle = m_uv_mesh_ptr->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    // split
    {
        OperationSettings<tri_mesh::ExtremeOptSplit> split_settings(m_mesh);
        split_settings.position = m_position_handle;
        // thresholds are inverted because we continue splitting because we
        // always split until we're under this length, which is the max
        // required length for the op to happen
        split_settings.min_squared_length = m_length_max * m_length_max;
        split_settings.split_boundary_edges = !m_lock_boundary;
        split_settings.uv_mesh_ptr = m_uv_mesh_ptr;
        split_settings.uv_handle = m_uv_handle;

        m_scheduler.add_operation_type<tri_mesh::ExtremeOptSplit>("split", split_settings);
    }
    // collapse
    {
        OperationSettings<tri_mesh::ExtremeOptCollapse> collapse_settings(m_mesh);
        collapse_settings.position = m_position_handle;
        // thresholds are inverted because we continue collapsing because we
        // always collapse until we're over this length, which is the minimum
        // required length for the op to happen
        collapse_settings.max_squared_length = m_length_min * m_length_min;
        collapse_settings.collapse_boundary_edges = !m_lock_boundary;
        collapse_settings.preserve_topology = true;

        collapse_settings.uv_mesh_ptr = m_uv_mesh_ptr;
        collapse_settings.uv_handle = m_uv_handle;
        collapse_settings.optimize_E_max = m_collapse_optimize_E_max;

        m_scheduler.add_operation_type<tri_mesh::ExtremeOptCollapse>("collapse", collapse_settings);
    }
    // flip
    {
        OperationSettings<tri_mesh::ExtremeOptSwap> swap_settings(m_mesh);
        swap_settings.position = m_position_handle;
        swap_settings.uv_mesh_ptr = m_uv_mesh_ptr;
        swap_settings.uv_handle = m_uv_handle;
        swap_settings.optimize_E_max = m_swap_optimize_E_max;

        m_scheduler.add_operation_type<tri_mesh::ExtremeOptSwap>("swap", swap_settings);
    } // smooth
    {
        OperationSettings<tri_mesh::VertexSmoothUsingDifferentiableEnergy> smooth_settings(
            *m_uv_mesh_ptr);
        smooth_settings.coordinate_handle = m_uv_handle;
        smooth_settings.smooth_boundary = true;
        smooth_settings.second_order = true;
        smooth_settings.line_search = true;
        smooth_settings.step_size = 1;
        std::shared_ptr<wmtk::function::SYMDIR> per_tri_symdir =
            std::make_shared<wmtk::function::SYMDIR>(
                m_mesh,
                *m_uv_mesh_ptr,
                m_position_handle,
                m_uv_handle,
                true);
        smooth_settings.energy =
            std::make_unique<function::LocalDifferentiableFunction>(per_tri_symdir);

        m_scheduler_uv.add_operation_type<tri_mesh::VertexSmoothUsingDifferentiableEnergy>(
            "smooth",
            std::move(smooth_settings));

        // OperationSettings<tri_mesh::VertexTangentialLaplacianSmooth> smooth_settings(m_mesh);
        // smooth_settings.position = m_position_handle;
        // smooth_settings.smooth_boundary = false; // TODO: we don't smooth boundary vertices for now

        // m_scheduler.add_operation_type<tri_mesh::VertexTangentialLaplacianSmooth>(
        //     "smooth",
        //     smooth_settings);
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

    wmtk::function::SYMDIR symdir_max(
        m_mesh,
        *m_uv_mesh_ptr,
        parent_vertex_handle,
        m_uv_handle,
        false);

    wmtk::function::SYMDIR symdir_sum(
        m_mesh,
        *m_uv_mesh_ptr,
        parent_vertex_handle,
        m_uv_handle,
        true);


    auto evaluate_energy_max = [&]() {
        const auto all_face_tuples_uv = m_uv_mesh_ptr->get_all(PrimitiveType::Face);
        std::vector<Simplex> all_faces_uv;
        for (const auto& face_tuple_uv : all_face_tuples_uv) {
            all_faces_uv.push_back(Simplex::face(face_tuple_uv));
        }
        return symdir_max.get_value_max(all_faces_uv);
    };

    auto evaluate_energy_sum = [&]() {
        const auto all_face_tuples_uv = m_uv_mesh_ptr->get_all(PrimitiveType::Face);
        std::vector<Simplex> all_faces_uv;
        for (const auto& face_tuple_uv : all_face_tuples_uv) {
            all_faces_uv.push_back(Simplex::face(face_tuple_uv));
        }
        return symdir_sum.get_value_sum(all_faces_uv);
    };

    wmtk::logger().info("Energy max before: {}", evaluate_energy_max());
    wmtk::logger().info("Energy sum before: {}", evaluate_energy_sum());

    // debug write
    if (m_debug_output) {
        write_debug_mesh(0);
    }
    long cnt = 0;
    for (long i = 0; i < iterations; ++i) {
        bool is_conn_valid;
        bool is_map_valid;


        wmtk::logger().info("Iteration {}", i);

        if (m_do_split) {
            m_scheduler.run_operation_on_all(PrimitiveType::Edge, "split");
            wmtk::logger().info("Done split {}", i);
            wmtk::logger().info("Energy max after split: {}", evaluate_energy_max());
            wmtk::logger().info("Energy sum after split: {}\n", evaluate_energy_sum());
            // debug write
            if (m_debug_output) {
                write_debug_mesh(++cnt);
            }
        }


        if (m_do_collapse) {
            m_scheduler.run_operation_on_all(PrimitiveType::Edge, "collapse");
            wmtk::logger().info("Done collapse {}", i);
            wmtk::logger().info("Energy max after collapse: {}", evaluate_energy_max());
            wmtk::logger().info("Energy sum after collapse: {}\n", evaluate_energy_sum());
            // debug write
            if (m_debug_output) {
                write_debug_mesh(++cnt);
            }
        }


        if (m_do_swap) {
            m_scheduler.run_operation_on_all(PrimitiveType::Edge, "swap");
            wmtk::logger().info("Done swap {}", i);
            wmtk::logger().info("Energy max after swap: {}", evaluate_energy_max());
            wmtk::logger().info("Energy sum after swap: {}\n", evaluate_energy_sum());
            // debug write
            if (m_debug_output) {
                write_debug_mesh(++cnt);
            }
        }


        if (m_do_smooth) {
            m_scheduler_uv.run_operation_on_all(PrimitiveType::Vertex, "smooth");
            wmtk::logger().info("Done smooth {}", i);
            wmtk::logger().info("Energy max after smooth: {}", evaluate_energy_max());
            wmtk::logger().info("Energy sum after smooth: {}\n", evaluate_energy_sum());
            // debug write
            if (m_debug_output) {
                write_debug_mesh(++cnt);
            }
        }


        wmtk::logger().info("Energy max after iter {} : {}", i, evaluate_energy_max());
        wmtk::logger().info("Energy sum after iter {} : {}", i, evaluate_energy_sum());
        wmtk::logger().info("Energy avg after iter{} : {}\n", i, symdir_sum.get_energy_avg());
    } // end for
}

} // namespace wmtk::components::internal
