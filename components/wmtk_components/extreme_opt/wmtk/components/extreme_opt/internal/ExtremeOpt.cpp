#include "ExtremeOpt.hpp"
#include <predicates.h>
#include <wmtk/EdgeMesh.hpp>
// #include <wmtk/operations/tri_mesh/ExtremeOptCollapse.hpp>
// #include <wmtk/operations/tri_mesh/ExtremeOptSplit.hpp>
// #include <wmtk/operations/tri_mesh/ExtremeOptSwap.hpp>
#include <wmtk/function/simplex/SYMDIR.hpp>
#include <wmtk/io/ParaviewWriter.hpp>

#include <wmtk/invariants/EdgeValenceInvariant.hpp>
#include <wmtk/invariants/FunctionInvariant.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorSimplexInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/MaxEdgeLengthInvariant.cpp>
#include <wmtk/invariants/MinEdgeLengthInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/NoBoundaryCollapseToInteriorInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>

#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>
#include <wmtk/utils/Logger.hpp>

// TODO: lock boundary don't work for uv mesh now
namespace wmtk::components::internal {

using namespace operations;
using namespace function;
using namespace invariants;

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
{
    using namespace operations;


    // get uv mesh and uv coordinates handle
    auto child_meshes = m_mesh.get_child_meshes();
    m_uv_mesh_ptr = std::static_pointer_cast<TriMesh>(child_meshes[0]);
    m_uv_handle = m_uv_mesh_ptr->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    /*
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

            m_scheduler.add_operation_type<tri_mesh::ExtremeOptCollapse>("collapse",
       collapse_settings);
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
            // OperationSettings<tri_mesh::VertexSmoothUsingDifferentiableEnergy> smooth_settings(
            //     *m_uv_mesh_ptr);
            OperationSettings<tri_mesh::VertexSmoothUsingDifferentiableEnergy>
       smooth_settings(m_mesh); smooth_settings.coordinate_handle = m_uv_handle;
            smooth_settings.smooth_boundary = true;
            smooth_settings.second_order = true;
            smooth_settings.line_search = true;
            smooth_settings.step_size = 1;
            smooth_settings.do_seamless_optimization = true;
            smooth_settings.uv_mesh_ptr = m_uv_mesh_ptr;
            smooth_settings.uv_handle = m_uv_handle;
            std::shared_ptr<wmtk::function::SYMDIR> per_tri_symdir =
                std::make_shared<wmtk::function::SYMDIR>(
                    m_mesh,
                    *m_uv_mesh_ptr,
                    m_position_handle,
                    m_uv_handle,
                    true);
            smooth_settings.energy =
                std::make_unique<function::LocalDifferentiableFunction>(per_tri_symdir);

            // m_scheduler_uv.add_operation_type<tri_mesh::VertexSmoothUsingDifferentiableEnergy>(
            //     "smooth",
            //     std::move(smooth_settings));
            m_scheduler.add_operation_type<tri_mesh::VertexSmoothUsingDifferentiableEnergy>(
                "smooth",
                std::move(smooth_settings));

            // OperationSettings<tri_mesh::VertexTangentialLaplacianSmooth> smooth_settings(m_mesh);
            // smooth_settings.position = m_position_handle;
            // smooth_settings.smooth_boundary = false; // TODO: we don't smooth boundary vertices for now

            // m_scheduler.add_operation_type<tri_mesh::VertexTangentialLaplacianSmooth>(
            //     "smooth",
            //     smooth_settings);
        }
        */
}

void ExtremeOpt::write_debug_mesh(const long test_id)
{
    wmtk::io::ParaviewWriter writer(
        "extreme_opt_" + m_mesh_name + "_seamed_" + std::to_string(test_id),
        "vertices",
        m_mesh,
        true,
        true,
        true,
        false);

    m_mesh.serialize(writer);

    wmtk::io::ParaviewWriter writer_uv(
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
    // TODO: create lambdas for priority

    // create energy to optimize
    std::shared_ptr<function::PerSimplexFunction> symdir_no_diff =
        std::make_shared<function::SYMDIR>(
            m_mesh,
            *m_uv_mesh_ptr,
            m_position_handle,
            m_uv_handle,
            true);
    std::shared_ptr<function::PerSimplexAutodiffFunction> symdir =
        std::make_shared<function::SYMDIR>(
            m_mesh,
            *m_uv_mesh_ptr,
            m_position_handle,
            m_uv_handle,
            true);

    auto evaluate_energy_sum = [&]() {
        double energy_sum = 0;
        const auto all_face_tuples = m_uv_mesh_ptr->get_all(PrimitiveType::Face);
        for (const auto& t : all_face_tuples) {
            energy_sum += symdir_no_diff->get_value(simplex::Simplex::face(t));
        }
        return energy_sum;
    };

    auto uv_accessor = m_uv_mesh_ptr->create_accessor(m_uv_handle.as<double>());


    // create lambdas for priority
    auto get_length = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        const auto uv0 = uv_accessor.vector_attribute(s.tuple());
        const auto uv1 = uv_accessor.vector_attribute(m_uv_mesh_ptr->switch_vertex(s.tuple()));
        return (uv0 - uv1).norm();
    };

    auto long_edge_first = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return std::vector<double>({-get_length(s)});
    };

    auto short_edges_first = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return std::vector<double>({get_length(s)});
    };


    /////////////////////////////////
    // Creation of the 4ops
    std::vector<std::shared_ptr<Operation>> ops;

    // 1) EdgeSplit
    auto split_op = std::make_shared<EdgeSplit>(m_mesh);
    {
        // MinEdgeLengthInvariant
        split_op->add_invariant(std::make_shared<MinEdgeLengthInvariant>(
            *m_uv_mesh_ptr,
            m_uv_handle.as<double>(),
            m_length_max * m_length_max));
        // Position and uv coordinate update
        split_op->set_new_attribute_strategy(
            m_position_handle,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::Mean);
        split_op->set_new_attribute_strategy(
            m_uv_handle,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::Mean);
        // long edge first priority
        split_op->set_priority(long_edge_first);
    }

    // 2) EdgeCollapse
    auto collapse_op = std::make_shared<EdgeCollapse>(m_mesh);
    {
        // LinkConditionInvariant
        collapse_op->add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m_mesh));
        // InversionInvariant
        collapse_op->add_invariant(
            std::make_shared<SimplexInversionInvariant>(m_mesh, m_position_handle.as<double>()));
        collapse_op->add_invariant(
            std::make_shared<SimplexInversionInvariant>(*m_uv_mesh_ptr, m_uv_handle.as<double>()));
        // MinEdgeLengthInvariant
        collapse_op->add_invariant(std::make_shared<MinEdgeLengthInvariant>(
            *m_uv_mesh_ptr,
            m_uv_handle.as<double>(),
            m_length_min * m_length_min));
        // Energy Decrease
        collapse_op->add_invariant(
            std::make_shared<FunctionInvariant>(m_uv_mesh_ptr->top_simplex_type(), symdir_no_diff));

        // set strategy for position attribute
        // TODO: is there a better way of doing this?
        // Importance definition: (input is the simplex on the uv_mesh)
        // 1) interior vertex: 0
        // 2) normal boundary vertex: 1
        // 3) branch point: 2
        auto vertex_importance = [&](const simplex::Simplex& s_child) -> int {
            assert(s_child.primitive_type() == primitive_type::Vertex);
            if (!m_uv_mesh_ptr->is_boundary_vertex(s_child.tuple())) return 0;
            const simplex::Simplex s_parent = m_uv_mesh_ptr->map_to_parent(s_child);
            if (m_mesh.map_to_child(*m_uv_mesh_ptr, s_parent).size() == 2) {
                return 1;
            } else {
                return 2;
            }
        };

        auto need_to_keep_in_child_mesh = [&](const simplex::Simplex& s_child) {
            assert(s_child.primitive_type() == primitive_type::Vertex);
            const simplex::Simplex s_child_sv =
                simplex::Simplex::vertex(m_uv_mesh_ptr->switch_vertex(s_child.tuple()));
            return (vertex_importance(s_child) >= vertex_importance(s_child_sv));
        };

        auto need_to_keep_in_parent_mesh = [&](const simplex::Simplex& s_parent) {
            const simplex::Simplex s_child = m_mesh.map_to_child(*m_uv_mesh_ptr, s_parent).front();
            return !need_to_keep_in_child_mesh(s_child);
        };


        // short edge first priority
        collapse_op->set_priority(short_edges_first);

        // TODO: set branch point invariant for collapse
        // TODO: set logic for which vertex to stay(attribute)
    }

    // 3) TriEdgeSwap
    auto swap_op = std::make_shared<composite::TriEdgeSwap>(m_mesh);
    {
        // link condition for collpase op in swap
        swap_op->collapse().add_invariant(
            std::make_shared<MultiMeshLinkConditionInvariant>(m_mesh));
        // Interior Edge in uv_mesh
        swap_op->add_invariant(std::make_shared<InteriorEdgeInvariant>(*m_uv_mesh_ptr));
        // no inversion on uv_mesh
        swap_op->add_invariant(
            std::make_shared<SimplexInversionInvariant>(*m_uv_mesh_ptr, m_uv_handle.as<double>()));
        // Energy Decrease
        swap_op->add_invariant(
            std::make_shared<FunctionInvariant>(m_uv_mesh_ptr->top_simplex_type(), symdir_no_diff));

        // long edge first priority
        swap_op->set_priority(long_edge_first);

        // set default attribute strategy
        swap_op->split().set_new_attribute_strategy(
            m_position_handle,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::Mean);
        swap_op->collapse().set_new_attribute_strategy(
            m_position_handle,
            CollapseBasicStrategy::CopyOther);
        swap_op->split().set_new_attribute_strategy(
            m_uv_handle,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::Mean);
        swap_op->collapse().set_new_attribute_strategy(
            m_uv_handle,
            CollapseBasicStrategy::CopyOther);
    }
    // TODO: ADD SMOOTH!!!


    wmtk::logger().info("Energy sum before: {}", evaluate_energy_sum());
    if (m_debug_output) {
        write_debug_mesh(0);
    }
    long cnt = 0;
    for (long i = 0; i < iterations; ++i) {
        bool is_conn_valid;
        bool is_map_valid;


        wmtk::logger().info("Iteration {}", i);

        if (m_do_split) {
            m_scheduler.run_operation_on_all(*split_op);
            wmtk::logger().info("Done split {}", i);
            // wmtk::logger().info("Energy max after split: {}", evaluate_energy_max());
            wmtk::logger().info("Energy sum after split: {}\n", evaluate_energy_sum());

            // debug write
            if (m_debug_output) {
                write_debug_mesh(++cnt);
            }
        }

        // if (m_do_collapse) {
        //     m_scheduler.run_operation_on_all(*collapse_op);
        //     wmtk::logger().info("Done collapse {}", i);
        //     // wmtk::logger().info("Energy max after collapse: {}", evaluate_energy_max());
        //     // wmtk::logger().info("Energy sum after collapse: {}\n", evaluate_energy_sum());

        //     // debug write
        //     if (m_debug_output) {
        //         write_debug_mesh(++cnt);
        //     }
        // }

        if (m_do_swap) {
            m_scheduler.run_operation_on_all(*swap_op);
            wmtk::logger().info("Done swap {}", i);
            // wmtk::logger().info("Energy max after swap: {}", evaluate_energy_max());
            wmtk::logger().info("Energy sum after swap: {}\n", evaluate_energy_sum());

            // debug write
            if (m_debug_output) {
                write_debug_mesh(++cnt);
            }
        }

        // if (m_do_smooth) {
        //     m_scheduler.run_operation_on_all(*smooth_op);
        //     wmtk::logger().info("Done smooth {}", i);
        //     // wmtk::logger().info("Energy max after smooth: {}", evaluate_energy_max());
        //     // wmtk::logger().info("Energy sum after smooth: {}\n", evaluate_energy_sum());
        // debug write
        // if (m_debug_output) {
        //     write_debug_mesh(++cnt);
        // }
        // }

        // wmtk::logger().info("Energy max after iter {} : {}", i, evaluate_energy_max());
        // wmtk::logger().info("Energy sum after iter {} : {}", i, evaluate_energy_sum());
        // wmtk::logger().info("Energy avg after iter{} : {}\n", i, symdir_sum.get_energy_avg());
    } // end for
}

} // namespace wmtk::components::internal
