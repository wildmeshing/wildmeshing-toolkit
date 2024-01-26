#include "ExtremeOptSingle.hpp"
#include <predicates.h>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/function/simplex/AMIPS.hpp>
#include <wmtk/function/simplex/SYMDIR.hpp>
#include <wmtk/function/simplex/TriangleAMIPS.hpp>
#include <wmtk/function/simplex/TriangleAreaAD.hpp>
#include <wmtk/invariants/EdgeValenceInvariant.hpp>
#include <wmtk/invariants/FunctionInvariant.hpp>
#include <wmtk/invariants/FunctionNumericalInvariant.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorSimplexInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/MaxEdgeLengthInvariant.cpp>
#include <wmtk/invariants/MinEdgeLengthInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/NoBoundaryCollapseToInteriorInvariant.hpp>
#include <wmtk/invariants/SeamlessCollapseInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/OptimizationSmoothing.hpp>
#include <wmtk/operations/SeamlessSmoothing.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::internal {

using namespace operations;
using namespace function;
using namespace invariants;

ExtremeOptSingle::ExtremeOptSingle(
    std::string mesh_name,
    TriMesh& mesh,
    const double length,
    const bool do_split,
    const bool do_collapse,
    const bool do_swap,
    const bool do_smooth,
    const bool debug_output)
    : m_mesh_name{mesh_name}
    , m_mesh{mesh}
    , m_length_min{(4. / 5.) * length}
    , m_length_max{(4. / 3.) * length}
    , m_do_split{do_split}
    , m_do_collapse{do_collapse}
    , m_do_swap{do_swap}
    , m_do_smooth{do_smooth}
    , m_debug_output{debug_output}
    , m_position_handle{m_mesh.get_attribute_handle<double>(
          "ref_coordinates",
          PrimitiveType::Vertex)}
    , m_uv_handle{m_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex)}
{}

void ExtremeOptSingle::write_debug_mesh(const long test_id)
{
    m_mesh.consolidate();

    wmtk::io::ParaviewWriter writer(
        "extreme_opt_" + m_mesh_name + "_3d_" + std::to_string(test_id),
        "ref_coordinates",
        m_mesh,
        true,
        true,
        true,
        false);

    m_mesh.serialize(writer);

    wmtk::io::ParaviewWriter writer_uv(
        "extreme_opt_" + m_mesh_name + "_uv_" + std::to_string(test_id),
        "vertices",
        m_mesh,
        true,
        true,
        true,
        false);

    m_mesh.serialize(writer_uv);
}


void ExtremeOptSingle::remeshing(const long iterations)
{
    // create energy to optimize
    std::shared_ptr<function::PerSimplexFunction> symdir_no_diff =
        std::make_shared<function::SYMDIR>(m_mesh, m_mesh, m_position_handle, m_uv_handle, true);
    std::shared_ptr<function::PerSimplexAutodiffFunction> symdir =
        std::make_shared<function::SYMDIR>(m_mesh, m_mesh, m_position_handle, m_uv_handle, true);
    std::shared_ptr<function::PerSimplexFunction> triangle_area =
        std::make_shared<function::TriangleAreaAD>(m_mesh, m_position_handle);

    auto evaluate_function_sum = [&](std::shared_ptr<function::PerSimplexFunction> f) {
        double function_sum = 0;
        const auto all_face_tuples = m_mesh.get_all(PrimitiveType::Face);
        for (const auto& t : all_face_tuples) {
            function_sum += f->get_value(simplex::Simplex::face(t));
        }
        return function_sum;
    };

    auto uv_accessor = m_mesh.create_accessor(m_uv_handle.as<double>());


    // create lambdas for priority
    auto get_length = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        const auto uv0 = uv_accessor.vector_attribute(s.tuple());
        const auto uv1 = uv_accessor.vector_attribute(m_mesh.switch_vertex(s.tuple()));
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
    /////////////////////////////////
    // 1) EdgeSplit
    auto split_op = std::make_shared<EdgeSplit>(m_mesh);
    {
        // MinEdgeLengthInvariant
        split_op->add_invariant(std::make_shared<MinEdgeLengthInvariant>(
            m_mesh,
            m_uv_handle.as<double>(),
            m_length_min * m_length_min));
        split_op->add_invariant(std::make_shared<FunctionNumericalInvariant>(
            m_mesh.top_simplex_type(),
            symdir_no_diff));
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
            std::make_shared<SimplexInversionInvariant>(m_mesh, m_uv_handle.as<double>()));
        // MinEdgeLengthInvariant
        collapse_op->add_invariant(std::make_shared<MaxEdgeLengthInvariant>(
            m_mesh,
            m_uv_handle.as<double>(),
            m_length_max * m_length_max));
        // Energy Decrease
        collapse_op->add_invariant(
            std::make_shared<FunctionInvariant>(m_mesh.top_simplex_type(), symdir_no_diff));
        {
            auto tmp = std::make_shared<CollapseNewAttributeStrategy<double>>(m_uv_handle);
            tmp->set_strategy(CollapseBasicStrategy::CopyOther);
            tmp->set_simplex_predicate(BasicSimplexPredicate::IsInterior);
            collapse_op->set_new_attribute_strategy(m_uv_handle, tmp);
        }
        {
            auto tmp = std::make_shared<CollapseNewAttributeStrategy<double>>(m_position_handle);
            tmp->set_strategy(CollapseBasicStrategy::CopyOther);
            tmp->set_simplex_predicate(BasicSimplexPredicate::IsInterior);
            collapse_op->set_new_attribute_strategy(m_position_handle, tmp);
        }


        // short edge first priority
        collapse_op->set_priority(short_edges_first);
    }

    // 3) TriEdgeSwap
    auto swap_op = std::make_shared<composite::TriEdgeSwap>(m_mesh);
    {
        // link condition for collpase op in swap
        swap_op->collapse().add_invariant(
            std::make_shared<MultiMeshLinkConditionInvariant>(m_mesh));
        // Interior Edge in uv_mesh
        swap_op->add_invariant(std::make_shared<InteriorEdgeInvariant>(m_mesh));
        // no inversion on uv_mesh
        swap_op->add_invariant(
            std::make_shared<SimplexInversionInvariant>(m_mesh, m_uv_handle.as<double>()));
        // Energy Decrease
        swap_op->add_invariant(
            std::make_shared<FunctionInvariant>(m_mesh.top_simplex_type(), symdir_no_diff));

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

    auto energy =
        std::make_shared<function::LocalNeighborsSumFunction>(m_mesh, m_uv_handle, *symdir);
    auto smooth_op = std::make_shared<OptimizationSmoothing>(m_mesh, energy);
    smooth_op->add_invariant(
        std::make_shared<SimplexInversionInvariant>(m_mesh, m_uv_handle.as<double>()));

    double E_sum = evaluate_function_sum(symdir_no_diff);
    double area_sum = evaluate_function_sum(triangle_area);
    wmtk::logger().info("Energy sum before: {}", E_sum);
    wmtk::logger().info("Energy Avg before: {}", E_sum / area_sum);
    if (m_debug_output) {
        write_debug_mesh(0);
    }
    long cnt = 0;
    for (long i = 0; i < iterations; ++i) {
        wmtk::logger().info("Iteration {}", i);

        if (m_do_split) {
            m_scheduler.run_operation_on_all(*split_op);
            wmtk::logger().info("Done split {}", i);
            // wmtk::logger().info("Energy max after split: {}", evaluate_energy_max());
            E_sum = evaluate_function_sum(symdir_no_diff);
            wmtk::logger().info("Energy sum after split: {}\n", E_sum);
            wmtk::logger().info("Energy avg after split: {}\n", E_sum / area_sum);

            // debug write
            if (m_debug_output) {
                write_debug_mesh(++cnt);
            }
        }

        if (m_do_collapse) {
            m_scheduler.run_operation_on_all(*collapse_op);
            wmtk::logger().info("Done collapse {}", i);
            // wmtk::logger().info("Energy max after collapse: {}", evaluate_energy_max());
            E_sum = evaluate_function_sum(symdir_no_diff);
            area_sum = evaluate_function_sum(triangle_area);
            wmtk::logger().info("Energy sum after collapse: {}\n", E_sum);
            wmtk::logger().info("Energy avg after collapse: {}\n", E_sum / area_sum);

            // debug write
            if (m_debug_output) {
                write_debug_mesh(++cnt);
            }
        }

        if (m_do_swap) {
            m_scheduler.run_operation_on_all(*swap_op);
            wmtk::logger().info("Done swap {}", i);
            // wmtk::logger().info("Energy max after swap: {}", evaluate_energy_max());
            E_sum = evaluate_function_sum(symdir_no_diff);
            area_sum = evaluate_function_sum(triangle_area);
            wmtk::logger().info("Energy sum after swap: {}\n", E_sum);
            wmtk::logger().info("Energy avg after swap: {}\n", E_sum / area_sum);

            // debug write
            if (m_debug_output) {
                write_debug_mesh(++cnt);
            }
        }

        if (m_do_smooth) {
            m_scheduler.run_operation_on_all(*smooth_op);
            wmtk::logger().info("Done smooth {}", i);
            // wmtk::logger().info("Energy max after smooth: {}", evaluate_energy_max());
            E_sum = evaluate_function_sum(symdir_no_diff);
            // area_sum = evaluate_function_sum(triangle_area);
            wmtk::logger().info("Energy sum after smooth: {}\n", E_sum);
            wmtk::logger().info("Energy avg after smooth: {}\n", E_sum / area_sum);
            // debug write
            if (m_debug_output) {
                write_debug_mesh(++cnt);
            }
        }

        // wmtk::logger().info("Energy max after iter {} : {}", i, evaluate_energy_max());
        // wmtk::logger().info("Energy sum after iter {} : {}", i, evaluate_function_sum());
        // wmtk::logger().info("Energy avg after iter{} : {}\n", i, symdir_sum.get_energy_avg());
    } // end for
}

} // namespace wmtk::components::internal
