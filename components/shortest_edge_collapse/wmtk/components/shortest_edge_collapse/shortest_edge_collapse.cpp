#include "shortest_edge_collapse.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/components/utils/get_attributes.hpp>
#include <wmtk/invariants/EnvelopeInvariant.hpp>
#include <wmtk/invariants/InteriorSimplexInvariant.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/invariants/MaxEdgeLengthInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/MultiMeshMapValidInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/invariants/uvEdgeInvariant.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include <wmtk/multimesh/consolidate.hpp>
#include <wmtk/operations/AttributesUpdate.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/attribute_new/CollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_new/NewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/utils/Logger.hpp>


namespace wmtk::components::shortest_edge_collapse {

void shortest_edge_collapse(Mesh& mesh_in, const ShortestEdgeCollapseOptions& options)
{
    if (mesh_in.top_simplex_type() != PrimitiveType::Edge &&
        mesh_in.top_simplex_type() != PrimitiveType::Triangle &&
        mesh_in.top_simplex_type() != PrimitiveType::Tetrahedron) {
        log_and_throw_error(
            "shortest edge collapse works only for edge, triangle, or tet meshes: {}",
            primitive_type_name(mesh_in.top_simplex_type()));
    }

    if (!mesh_in.is_multi_mesh_root()) {
        log_and_throw_error("The mesh passed in shortest_edge_collapse must be the root mesh");
    }

    attribute::MeshAttributeHandle position_handle = options.position_handle;
    std::vector<attribute::MeshAttributeHandle> other_position_handles =
        options.other_position_handles;

    Mesh& mesh = position_handle.mesh();

    std::vector<attribute::MeshAttributeHandle> inversion_position_handles;
    if (options.check_inversions) {
        if (position_handle.mesh().top_cell_dimension() == position_handle.dimension()) {
            logger().info("Adding inversion check on collapsing mesh.");
            inversion_position_handles.emplace_back(position_handle);
        }
        for (auto& h : other_position_handles) {
            if (h.mesh().top_cell_dimension() == h.dimension()) {
                logger().info("Adding inversion check on other mesh.");
                inversion_position_handles.emplace_back(h);
            }
        }

        if (inversion_position_handles.empty()) {
            logger().warn("Shortest-edge collapse should check for inversions but there was no "
                          "position handle that is valid for inversion checks.");
        }
    }

    std::vector<attribute::MeshAttributeHandle> pass_through_attributes =
        options.pass_through_attributes;

    for (auto& h : other_position_handles) {
        pass_through_attributes.emplace_back(h);
    }

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
    Eigen::VectorXd bmin(position_handle.dimension());
    bmin.setConstant(std::numeric_limits<double>::max());
    Eigen::VectorXd bmax(position_handle.dimension());
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

    const double length_abs = bbdiag * options.length_rel;

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
    pass_through_attributes.push_back(edge_length_attribute);
    auto todo = std::make_shared<TodoSmallerInvariant>(
        mesh,
        edge_length_attribute.as<double>(),
        4. / 5. * length_abs); // MTAO: why is this 4/5?

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
    wmtk::multimesh::MultiMeshVisitor visitor(set_all_invariants);
    visitor.execute_from_root(mesh);

    auto invariant_mm_map = std::make_shared<MultiMeshMapValidInvariant>(mesh);

    ////////////// positions
    std::vector<attribute::MeshAttributeHandle> position_handles;
    position_handles.emplace_back(position_handle);

    //////////////////////////////////////////
    // collapse
    auto collapse = std::make_shared<wmtk::operations::EdgeCollapse>(mesh);
    collapse->add_invariant(todo);
    collapse->add_invariant(invariant_link_condition);
    collapse->add_invariant(invariant_mm_map);


    if (options.envelope_size) {
        const double env_size = bbdiag * options.envelope_size.value();
        bool envelope_added = false;

        if (position_handle.mesh().top_cell_dimension() < position_handle.dimension()) {
            logger().info("Adding envelope check on collapsing mesh.");
            collapse->add_invariant(std::make_shared<wmtk::invariants::EnvelopeInvariant>(
                position_handle,
                env_size,
                position_handle));
            envelope_added = true;
        }

        for (auto& h : other_position_handles) {
            if (h.mesh().top_cell_dimension() < h.dimension()) {
                logger().info("Adding envelope check on other mesh.");
                collapse->add_invariant(
                    std::make_shared<wmtk::invariants::EnvelopeInvariant>(h, env_size, h));
                envelope_added = true;
            }
        }

        if (!envelope_added) {
            logger().warn("Shortest-edge collapse should check for inversion but there was no "
                          "position handle that is valid for inversion checks.");
        }
    }

    for (auto& h : inversion_position_handles) {
        collapse->add_invariant(
            std::make_shared<SimplexInversionInvariant<double>>(h.mesh(), h.as<double>()));
    }

    collapse->set_new_attribute_strategy(
        visited_edge_flag,
        wmtk::operations::CollapseBasicStrategy::None);
    collapse->add_transfer_strategy(tag_update);

    collapse->set_priority(short_edges_first_priority);
    collapse->add_transfer_strategy(edge_length_update);

    if (options.lock_boundary) {
        if (mesh_in.top_simplex_type() != PrimitiveType::Edge) {
            collapse->add_invariant(invariant_interior_edge);
        }
        // set collapse towards boundary
        for (const auto& pos_handle : position_handles) {
            auto pos_collapse_strategy =
                std::make_shared<wmtk::operations::CollapseNewAttributeStrategy<double>>(
                    pos_handle);
            pos_collapse_strategy->set_strategy(wmtk::operations::CollapseBasicStrategy::Default);
            pos_collapse_strategy->set_simplex_predicate(
                wmtk::operations::BasicSimplexPredicate::IsInterior);
            collapse->set_new_attribute_strategy(pos_handle, pos_collapse_strategy);
        }
    } else {
        for (const auto& pos_handle : position_handles) {
            collapse->set_new_attribute_strategy(
                pos_handle,
                wmtk::operations::CollapseBasicStrategy::CopyOther);
        }
    }


    for (const auto& attr : pass_through_attributes) {
        collapse->set_new_attribute_strategy(attr);
    }

    auto propagate_position = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd { return P; };
    for (auto& h : other_position_handles) {
        auto transfer_position =
            std::make_shared<operations::SingleAttributeTransferStrategy<double, double>>(
                h,
                position_handle,
                propagate_position);
        collapse->add_transfer_strategy(transfer_position);
    }


    //////////////////////////////////////////
    Scheduler scheduler;
    SchedulerStats pass_stats =
        scheduler.run_operation_on_all(*collapse, visited_edge_flag.as<char>());


    // // debug code
    // for (const auto& e : mesh.get_all(PrimitiveType::Edge)) {
    //     if (mesh.is_boundary(PrimitiveType::Edge, e)) {
    //         logger().error("after sec mesh has nonmanifold edges");
    //     }
    // }

    wmtk::multimesh::consolidate(mesh);

    logger().info(
        "Executed {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, executing: {}",
        pass_stats.number_of_performed_operations(),
        pass_stats.number_of_successful_operations(),
        pass_stats.number_of_failed_operations(),
        pass_stats.collecting_time,
        pass_stats.sorting_time,
        pass_stats.executing_time);
}

void shortest_edge_collapse(
    Mesh& mesh,
    const attribute::MeshAttributeHandle& position_handle,
    const double length_rel,
    std::optional<bool> lock_boundary,
    std::optional<double> envelope_size,
    bool check_inversion,
    const std::vector<attribute::MeshAttributeHandle>& pass_through)
{
    ShortestEdgeCollapseOptions options;
    options.position_handle = position_handle;
    options.length_rel = length_rel;
    if (lock_boundary) {
        options.lock_boundary = lock_boundary.value();
    }
    options.envelope_size = envelope_size;

    options.pass_through_attributes = pass_through;
    shortest_edge_collapse(mesh, options);
}
} // namespace wmtk::components::shortest_edge_collapse
