#include "shortestedge_collapse.hpp"

#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/components/base/get_attributes.hpp>
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

#include "internal/SECOptions.hpp"

namespace wmtk::components {
void shortestedge_collapse(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    SECOptions options = j.get<SECOptions>();

    std::shared_ptr<Mesh> mesh_in = cache.read_mesh(options.input);

    auto pos_handles = base::get_attributes(cache, *mesh_in, options.attributes.position);
    assert(pos_handles.size() == 1);
    auto pos_handle = pos_handles.front();

    if (pos_handle.mesh().top_simplex_type() != PrimitiveType::Triangle) {
        log_and_throw_error(
            "isotropic remeshing works only for triangle meshes: {}",
            mesh_in->top_simplex_type());
    }

    auto pass_through_attributes = base::get_attributes(cache, *mesh_in, options.pass_through);
    auto other_positions =
        base::get_attributes(cache, *mesh_in, options.attributes.other_positions);

    std::optional<attribute::MeshAttributeHandle> position_for_inversion;

    if (!options.attributes.inversion_position.empty()) {
        auto tmp = base::get_attributes(cache, *mesh_in, options.attributes.inversion_position);
        assert(tmp.size() == 1);
        position_for_inversion = tmp.front();
    }

    /////////////////////////////////////////////

    // TriMesh& mesh = static_cast<TriMesh&>(*mesh_in);
    TriMesh& mesh = static_cast<TriMesh&>(pos_handle.mesh());


    // // debug code
    // for (const auto& e : mesh.get_all(PrimitiveType::Edge)) {
    //     if (mesh.is_boundary(PrimitiveType::Edge, e)) {
    //         logger().error("before sec mesh has nonmanifold edges");
    //     }
    // }

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
            pos_handle,
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
            pos_handle,
            compute_edge_length);
    edge_length_update->run_on_all();

#if false // old implementation from teseo/sec branch
    double avg_length = 0;
    const auto edges = mesh.get_all(PrimitiveType::Edge);
    for (const auto& e : edges) {
        avg_length += edge_length_accessor.const_scalar_attribute(e);
    }
    avg_length /= edges.size();
    if (options.length_abs < 0) {
        if (options.length_rel < 0) {
            throw std::runtime_error("Either absolute or relative length must be set!");
        }
        options.length_abs = avg_length * options.length_rel;
    }
    logger().info("Average edge length: {}, target {}", avg_length, options.length_abs);
#else
    //////////////////////////////////
    // computing bbox diagonal
    Eigen::VectorXd bmin(mesh.top_cell_dimension());
    bmin.setConstant(std::numeric_limits<double>::max());
    Eigen::VectorXd bmax(mesh.top_cell_dimension());
    bmax.setConstant(std::numeric_limits<double>::lowest());

    auto pt_accessor = mesh.create_const_accessor<double>(pos_handle);

    const auto vertices = mesh.get_all(PrimitiveType::Vertex);
    for (const auto& v : vertices) {
        const auto p = pt_accessor.vector_attribute(v);
        for (int64_t d = 0; d < bmax.size(); ++d) {
            bmin[d] = std::min(bmin[d], p[d]);
            bmax[d] = std::max(bmax[d], p[d]);
        }
    }

    const double bbdiag = (bmax - bmin).norm();

    options.length_abs = bbdiag * options.length_rel;

    wmtk::logger().info(
        "bbox max {}, bbox min {}, diag {}, target edge length {}",
        bmax,
        bmin,
        bbdiag,
        options.length_abs);

#endif
    auto short_edges_first_priority = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return edge_length_accessor.const_scalar_attribute(s.tuple());
    };
    pass_through_attributes.push_back(edge_length_attribute);
    auto todo = std::make_shared<TodoSmallerInvariant>(
        mesh,
        edge_length_attribute.as<double>(),
        4. / 5. * options.length_abs); // MTAO: why is this 4/5?

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
    std::vector<attribute::MeshAttributeHandle> positions = other_positions;
    positions.push_back(pos_handle);


    //////////////////////////////////////////
    // collapse
    auto collapse = std::make_shared<wmtk::operations::EdgeCollapse>(mesh);
    collapse->add_invariant(todo);
    collapse->add_invariant(invariant_link_condition);
    collapse->add_invariant(invariant_mm_map);


    if (options.envelope_size > 0) {
        collapse->add_invariant(std::make_shared<wmtk::invariants::EnvelopeInvariant>(
            pos_handle,
            bbdiag * options.envelope_size,
            pos_handle));
    }


    if (position_for_inversion) {
        collapse->add_invariant(std::make_shared<SimplexInversionInvariant<double>>(
            position_for_inversion.value().mesh(),
            position_for_inversion.value().as<double>()));
    }
    collapse->set_new_attribute_strategy(
        visited_edge_flag,
        wmtk::operations::CollapseBasicStrategy::None);
    collapse->add_transfer_strategy(tag_update);

    collapse->set_priority(short_edges_first_priority);
    collapse->add_transfer_strategy(edge_length_update);


    // hack for uv
    if (options.fix_uv_seam) {
        collapse->add_invariant(
            std::make_shared<invariants::uvEdgeInvariant>(mesh, other_positions.front().mesh()));
    }

    if (options.lock_boundary && !options.use_for_periodic) {
        collapse->add_invariant(invariant_interior_edge);
        // set collapse towards boundary
        for (auto& p : positions) {
            auto tmp = std::make_shared<wmtk::operations::CollapseNewAttributeStrategy<double>>(p);
            tmp->set_strategy(wmtk::operations::CollapseBasicStrategy::CopyOther);
            tmp->set_simplex_predicate(wmtk::operations::BasicSimplexPredicate::IsInterior);
            collapse->set_new_attribute_strategy(p, tmp);
        }
    } else if (options.use_for_periodic) {
        collapse->add_invariant(
            std::make_shared<invariants::FusionEdgeInvariant>(mesh, mesh.get_multi_mesh_root()));
        for (auto& p : positions) {
            collapse->set_new_attribute_strategy(
                p,
                wmtk::operations::CollapseBasicStrategy::CopyOther);
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


    // output
    cache.write_mesh(*mesh_in, options.output);
}
} // namespace wmtk::components
