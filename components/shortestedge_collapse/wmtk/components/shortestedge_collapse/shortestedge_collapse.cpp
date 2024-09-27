#include "shortestedge_collapse.hpp"

#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>
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
#include <wmtk/operations/attribute_update/TopologicalTransferStrategy.hpp>
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/Logger.hpp>


namespace wmtk::components {

void shortestedge_collapse(TriMesh& mesh, const ShortestEdgeCollapseOptions& options)
{
    if (mesh.top_simplex_type() != PrimitiveType::Triangle) {
        log_and_throw_error(
            "shortest edge collapse works only for triangle meshes: {}",
            primitive_type_name(mesh.top_simplex_type()));
    }

    const attribute::MeshAttributeHandle& position_handle = options.position_handle;
    const attribute::MeshAttributeHandle& inversion_position_handle =
        options.inversion_position_handle;

    // TriMesh& mesh = static_cast<TriMesh&>(position_handle.mesh());

    std::vector<attribute::MeshAttributeHandle> pass_through_attributes =
        options.pass_through_attributes;

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

    auto pos_acc = mesh.create_accessor(position_handle.as<double>());
    auto visited_edge_flag_acc = mesh.create_accessor(visited_edge_flag.as<char>());

    ////////////////////////////////////////////////
    auto edge_attributes_update = [&mesh, &edge_length_accessor, &pos_acc, &visited_edge_flag_acc](
                                      const simplex::Simplex& s) -> void {
        assert(s.primitive_type() == PrimitiveType::Vertex);

        const auto edges = simplex::cofaces_single_dimension(mesh, s, PrimitiveType::Edge);
        for (const simplex::Simplex& e : edges) {
            visited_edge_flag_acc.scalar_attribute(e) = char(1);
            const auto vertices = simplex::faces_single_dimension(mesh, e, PrimitiveType::Vertex);
            const auto p0 = pos_acc.const_vector_attribute(vertices.simplex_vector()[0]);
            const auto p1 = pos_acc.const_vector_attribute(vertices.simplex_vector()[1]);
            edge_length_accessor.scalar_attribute(e) = (p1 - p0).norm();
        }
    };

    auto vertex_to_edge_transfer = std::make_shared<wmtk::operations::TopologicalTransferStrategy>(
        position_handle,
        edge_attributes_update);

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
    multimesh::MultiMeshVisitor visitor(set_all_invariants);
    visitor.execute_from_root(mesh);

    auto invariant_mm_map = std::make_shared<MultiMeshMapValidInvariant>(mesh);

    ////////////// positions
    std::vector<attribute::MeshAttributeHandle> position_handles;
    position_handles.push_back(position_handle);
    if (inversion_position_handle.is_valid()) {
        position_handles.push_back(inversion_position_handle);
    }

    //////////////////////////////////////////
    // collapse
    auto collapse = std::make_shared<wmtk::operations::EdgeCollapse>(mesh);
    collapse->add_invariant(todo);
    collapse->add_invariant(invariant_link_condition);
    collapse->add_invariant(invariant_mm_map);


    if (options.envelope_size) {
        collapse->add_invariant(std::make_shared<wmtk::invariants::EnvelopeInvariant>(
            position_handle,
            bbdiag * options.envelope_size.value(),
            position_handle));
    }


    if (inversion_position_handle.is_valid()) {
        collapse->add_invariant(std::make_shared<SimplexInversionInvariant<double>>(
            inversion_position_handle.mesh(),
            inversion_position_handle.as<double>()));
    }
    collapse->set_new_attribute_strategy(
        visited_edge_flag,
        wmtk::operations::CollapseBasicStrategy::None);

    collapse->set_priority(short_edges_first_priority);
    // collapse->add_transfer_strategy(tag_update);
    // collapse->add_transfer_strategy(edge_length_update);
    collapse->add_topology_transfer_strategy(vertex_to_edge_transfer);

    if (options.lock_boundary) {
        collapse->add_invariant(invariant_interior_edge);
        // set collapse towards boundary
        for (const auto& pos_handle : position_handles) {
            auto pos_collapse_strategy =
                std::make_shared<wmtk::operations::CollapseNewAttributeStrategy<double>>(
                    pos_handle);
            pos_collapse_strategy->set_strategy(wmtk::operations::CollapseBasicStrategy::CopyOther);
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

    multimesh::consolidate(mesh);

    logger().info(
        "Executed {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, executing: {}",
        pass_stats.number_of_performed_operations(),
        pass_stats.number_of_successful_operations(),
        pass_stats.number_of_failed_operations(),
        pass_stats.collecting_time,
        pass_stats.sorting_time,
        pass_stats.executing_time);
}

void shortestedge_collapse(
    TriMesh& mesh,
    const attribute::MeshAttributeHandle& position_handle,
    const double length_rel,
    std::optional<bool> lock_boundary,
    std::optional<double> envelope_size,
    std::optional<attribute::MeshAttributeHandle> inversion_position_handle,
    const std::vector<attribute::MeshAttributeHandle>& pass_through)
{
    ShortestEdgeCollapseOptions options;
    options.position_handle = position_handle;
    options.length_rel = length_rel;
    if (lock_boundary) {
        options.lock_boundary = lock_boundary.value();
    }
    options.envelope_size = envelope_size;
    if (inversion_position_handle) {
        options.inversion_position_handle = inversion_position_handle.value();
    }
    options.pass_through_attributes = pass_through;
    shortestedge_collapse(mesh, options);
}
} // namespace wmtk::components
