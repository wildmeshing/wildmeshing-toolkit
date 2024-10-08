#include "cdt_optimization.hpp"

#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/components/utils/get_attributes.hpp>
#include <wmtk/function/PerSimplexFunction.hpp>
#include <wmtk/function/simplex/AMIPS.hpp>
#include <wmtk/invariants/EnvelopeInvariant.hpp>
#include <wmtk/invariants/InteriorSimplexInvariant.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/invariants/MaxEdgeLengthInvariant.hpp>
#include <wmtk/invariants/MaxFunctionInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/MultiMeshMapValidInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/Swap2dEdgeLengthInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/invariants/uvEdgeInvariant.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include <wmtk/multimesh/consolidate.hpp>
#include <wmtk/operations/AttributesUpdate.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/attribute_new/CollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_new/NewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/orient.hpp>
#include "predicates.h"


namespace wmtk::components {
std::shared_ptr<Mesh> cdt_optimization(
    TriMesh& mesh,
    const attribute::MeshAttributeHandle& position_handle,
    std::optional<attribute::MeshAttributeHandle>& inversion_position_handle,
    const int64_t passes,
    bool update_other_position,
    const double length_rel,
    bool lock_boundary,
    double envelope_size,
    const std::vector<attribute::MeshAttributeHandle>& pass_through)
{
    if (mesh.top_simplex_type() != PrimitiveType::Triangle) {
        log_and_throw_error(
            "shortest edge collapse works only for triangle meshes: {}",
            primitive_type_name(mesh.top_simplex_type()));
    }

    // TriMesh& mesh = static_cast<TriMesh&>(position_handle.mesh());

    auto pass_through_attributes = pass_through;

    bool check_inversion = inversion_position_handle.has_value();

    /////////////////////////////////////////////

    auto visited_edge_flag =
        mesh.register_attribute<char>("visited_edge", PrimitiveType::Edge, 1, false, char(1));

    auto update_flag_func = [](Eigen::Ref<const Eigen::MatrixXd> P) -> Eigen::VectorX<char> {
        assert(P.cols() == 2);
        assert(P.rows() == 2 || P.rows() == 3);
        return Eigen::VectorX<char>::Constant(1, char(1));
    };
    auto tag_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<char, double>>(
            visited_edge_flag,
            position_handle,
            update_flag_func);

    //////////////////////////////////
    // Storing edge lengths
    auto edge_length_attribute =
        mesh.register_attribute<double>("edge_length", PrimitiveType::Edge, 1);
    auto edge_length_accessor = mesh.create_accessor(edge_length_attribute.as<double>());
    // Edge length update
    auto compute_edge_length = [](Eigen::Ref<const Eigen::MatrixXd> P) -> Eigen::VectorXd {
        assert(P.cols() == 2);
        assert(P.rows() == 2 || P.rows() == 3);
        return Eigen::VectorXd::Constant(1, (P.col(0) - P.col(1)).norm());
    };
    auto edge_length_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            edge_length_attribute,
            position_handle,
            compute_edge_length);
    edge_length_update->run_on_all();

    //////////////////////////////////
    // computing bbox diagonal
    Eigen::VectorXd bmin(mesh.top_cell_dimension());
    bmin.setConstant(std::numeric_limits<double>::max());
    Eigen::VectorXd bmax(mesh.top_cell_dimension());
    bmax.setConstant(std::numeric_limits<double>::lowest());

    auto pt_accessor = mesh.create_const_accessor<double>(position_handle);

    const auto vertices = mesh.get_all(PrimitiveType::Vertex);
    for (const auto& v : vertices) {
        const auto p = pt_accessor.vector_attribute(v);
        for (int64_t d = 0; d < bmax.size(); ++d) {
            bmin[d] = std::min(bmin[d], p[d]);
            bmax[d] = std::max(bmax[d], p[d]);
        }
    }

    const double bbdiag = (bmax - bmin).norm();

    const double length_abs = bbdiag * length_rel;

    wmtk::logger().info(
        "bbox max {}, bbox min {}, diag {}, target edge length {}",
        bmax,
        bmin,
        bbdiag,
        length_abs);

    auto short_edges_first_priority = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return edge_length_accessor.const_scalar_attribute(s.tuple());
    };
    auto long_edges_first_priority = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return -edge_length_accessor.const_scalar_attribute(s.tuple());
    };
    pass_through_attributes.push_back(edge_length_attribute);
    auto todo_smaller = std::make_shared<TodoSmallerInvariant>(
        mesh,
        edge_length_attribute.as<double>(),
        4. / 5. * length_abs); // MTAO: why is this 4/5?

    auto todo_larger= = std::make_shared<TodoLargerInvariant>(
        mesh,
        edge_length_attribute.as<double>(),
        4.0 / 3.0);

    //////////////////////////invariants

    auto invariant_link_condition = std::make_shared<MultiMeshLinkConditionInvariant>(mesh);

    auto invariant_interior_edge = std::make_shared<invariants::InvariantCollection>(mesh);
    auto invariant_interior_vertex = std::make_shared<invariants::InvariantCollection>(mesh);

    auto set_all_invariants = [&](auto&& m) {
        invariant_interior_edge->add(
            std::make_shared<invariants::InteriorSimplexInvariant>(m, PrimitiveType::Edge));
        invariant_interior_vertex->add(
            std::make_shared<invariants::InteriorSimplexInvariant>(m, PrimitiveType::Vertex));
    };
    multimesh::MultiMeshVisitor visitor(set_all_invariants);
    visitor.execute_from_root(mesh);

    auto invariant_mm_map = std::make_shared<MultiMeshMapValidInvariant>(mesh);

    ////////////// positions
    std::vector<attribute::MeshAttributeHandle> positions;
    positions.push_back(position_handle);
    if (check_inversion) {
        positions.push_back(inversion_position_handle.value());
    }

    //////////////////////////////////////////
    // split
    //////////////////////////////////////////

    auto split = std::make_shared<wmtk::operations::EdgeSplit>(mesh);
    split->add_invariant(todo_larger);
    if (check_inversion) {
        split->add_invariant(std::make_shared<SimplexInversionInvariant<double>>(
            inversion_position_handle.value().mesh(),
            inversion_position_handle.value().as<double>()));
    }
    
    split->set_priority(long_edges_first_priority);

    split->set_new_attribute_strategy(
        visited_edge_flag,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);

    split->add_transfer_strategy(tag_update);
    split->add_transfer_strategy(edge_length_update);

    for (auto& p : positions) {
        split->set_new_attribute_strategy(p);
    }

    for (const auto& attr : pass_through_attributes) {
        split->set_new_attribute_strategy(
            attr,
            wmtk::operations::SplitBasicStrategy::None,
            wmtk::operations::SplitRibBasicStrategy::None);
    }



    //////////////////////////////////////////
    // collapse
    //////////////////////////////////////////
    auto collapse = std::make_shared<wmtk::operations::EdgeCollapse>(mesh);
    collapse->add_invariant(todo_smaller);
    collapse->add_invariant(invariant_link_condition);
    collapse->add_invariant(invariant_mm_map);


    if (envelope_size > 0) {
        collapse->add_invariant(std::make_shared<wmtk::invariants::EnvelopeInvariant>(
            position_handle,
            bbdiag * envelope_size,
            position_handle));
    }


    if (check_inversion) {
        collapse->add_invariant(std::make_shared<SimplexInversionInvariant<double>>(
            inversion_position_handle.value().mesh(),
            inversion_position_handle.value().as<double>()));
    }
    collapse->set_new_attribute_strategy(
        visited_edge_flag,
        wmtk::operations::CollapseBasicStrategy::None);
    collapse->add_transfer_strategy(tag_update);

    collapse->set_priority(short_edges_first_priority);
    collapse->add_transfer_strategy(edge_length_update);

    if (lock_boundary) {
        collapse->add_invariant(invariant_interior_edge);
        // set collapse towards boundary
        for (auto& p : positions) {
            auto tmp = std::make_shared<wmtk::operations::CollapseNewAttributeStrategy<double>>(p);
            tmp->set_strategy(wmtk::operations::CollapseBasicStrategy::CopyOther);
            tmp->set_simplex_predicate(wmtk::operations::BasicSimplexPredicate::IsInterior);
            collapse->set_new_attribute_strategy(p, tmp);
        }
    } else {
        for (auto& p : positions) {
            collapse->set_new_attribute_strategy(
                p,
                wmtk::operations::CollapseBasicStrategy::CopyOther);
        }
    }


    for (const auto& attr : pass_through_attributes) {
        collapse->set_new_attribute_strategy(attr);
    }

    //////////////////////////////////////////
    // swap
    //////////////////////////////////////////

    std::shared_ptr<function::PerSimplexFunction> amips =
        std::make_shared<wmtk::function::AMIPS>(mesh, position_handle);
    auto function_invariant =
        std::make_shared<wmtk::invariants::MaxFunctionInvariant>(mesh.top_simplex_type(), amips);

    auto swap = std::make_shared<wmtk::operations::composite::TriEdgeSwap>(mesh);

    swap->add_invariant(
        std::make_shared<wmtk::Swap2dEdgeLengthInvariant>(mesh, position_handle.as<double>()));

    swap->add_invariant(invariant_mm_map);

    swap->collapse().add_invariant(invariant_link_condition);

    if (envelope_size > 0) {
        swap->add_invariant(std::make_shared<wmtk::invariants::EnvelopeInvariant>(
            position_handle,
            bbdiag * envelope_size,
            position_handle));
    }


    if (check_inversion) {
        swap->collapse().add_invariant(std::make_shared<SimplexInversionInvariant<double>>(
            inversion_position_handle.value().mesh(),
            inversion_position_handle.value().as<double>()));
    }

    // swap->add_invariant(function_invariant);

    swap->split().set_new_attribute_strategy(
        visited_edge_flag,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    swap->collapse().set_new_attribute_strategy(
        visited_edge_flag,
        wmtk::operations::CollapseBasicStrategy::None);

    swap->add_transfer_strategy(tag_update);

    swap->add_transfer_strategy(edge_length_update);

    for (auto& p : positions) {
        swap->split().set_new_attribute_strategy(p);
        swap->collapse().set_new_attribute_strategy(
            p,
            wmtk::operations::CollapseBasicStrategy::CopyOther);
    }

    swap->set_priority(short_edges_first_priority);

    swap->add_transfer_strategy(edge_length_update);

    for (const auto& attr : pass_through_attributes) {
        swap->split().set_new_attribute_strategy(attr);
        swap->collapse().set_new_attribute_strategy(attr);
    }

    //////////////////////////////////////////
    // scheduler
    //////////////////////////////////////////

    Scheduler scheduler;

    for (int64_t i = 0; i < passes; ++i) {
        auto collapse_stats =
            scheduler.run_operation_on_all(*collapse, visited_edge_flag.as<char>());

        logger().info(
            "Executed {}, {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, "
            "executing: {}",
            "collapse",
            collapse_stats.number_of_performed_operations(),
            collapse_stats.number_of_successful_operations(),
            collapse_stats.number_of_failed_operations(),
            collapse_stats.collecting_time,
            collapse_stats.sorting_time,
            collapse_stats.executing_time);

        auto swap_stats = scheduler.run_operation_on_all(*swap, visited_edge_flag.as<char>());

        logger().info(
            "Executed {}, {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, "
            "executing: {}",
            "swap",
            swap_stats.number_of_performed_operations(),
            swap_stats.number_of_successful_operations(),
            swap_stats.number_of_failed_operations(),
            swap_stats.collecting_time,
            swap_stats.sorting_time,
            swap_stats.executing_time);

        multimesh::consolidate(mesh);
    }


    // debug code
    for (const auto& e : mesh.get_all(PrimitiveType::Edge)) {
        if (mesh.is_boundary(PrimitiveType::Edge, e)) {
            logger().error("after sec mesh has nonmanifold edges");
        }
    }

    auto inversion_pos_accessor =
        inversion_position_handle.value().mesh().create_const_accessor<double>(
            inversion_position_handle.value());

    // debug code
    for (const auto& t :
         inversion_position_handle.value().mesh().get_all(PrimitiveType::Tetrahedron)) {
        const auto vertices = inversion_position_handle.value().mesh().orient_vertices(t);
        std::vector<Vector3d> pos;
        for (int i = 0; i < vertices.size(); ++i) {
            pos.push_back(inversion_pos_accessor.const_vector_attribute(vertices[i]));
        }
        if (orient3d(pos[0].data(), pos[1].data(), pos[2].data(), pos[3].data()) <= 0) {
            wmtk::logger().error(
                "Flipped or degenerated tet! orient3d = {}",
                orient3d(pos[0].data(), pos[1].data(), pos[2].data(), pos[3].data()));
        } else if (orient3d(pos[0].data(), pos[1].data(), pos[2].data(), pos[3].data()) <= 1e-5) {
            wmtk::logger().error(
                "Nearly degenerated tet! orient3d = {}",
                orient3d(pos[0].data(), pos[1].data(), pos[2].data(), pos[3].data()));
        }
    }


    return mesh.shared_from_this();
}
} // namespace wmtk::components
