#include "shortestedge_collapse.hpp"

#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/components/base/get_attributes.hpp>
#include <wmtk/invariants/InteriorSimplexInvariant.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/invariants/MaxEdgeLengthInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/MultiMeshMapValidInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/uvEdgeInvariant.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include <wmtk/multimesh/consolidate.hpp>
#include <wmtk/operations/AttributesUpdate.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/attribute_new/CollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_new/NewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/utils/Logger.hpp>

#include <Eigen/Geometry>
#include "internal/SECOptions.hpp"

namespace wmtk::components {
// compute the length relative to the bounding box diagonal
namespace {
double relative_to_absolute_length(
    const attribute::MeshAttributeHandle& pos_handle,
    const double length_rel)
{
    auto pos = pos_handle.mesh().create_const_accessor<double>(pos_handle);
    const auto vertices = pos_handle.mesh().get_all(PrimitiveType::Vertex);
    Eigen::AlignedBox<double, Eigen::Dynamic> bbox(pos.dimension());


    for (const auto& v : vertices) {
        bbox.extend(pos.const_vector_attribute(v));
    }

    const double diag_length = bbox.sizes().norm();

    return length_rel * diag_length;
}
} // namespace

void shortestedge_collapse(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    SECOptions options = j.get<SECOptions>();

    std::shared_ptr<Mesh> mesh_in = cache.read_mesh(options.input);

    auto pos_handles = base::get_attributes(cache, *mesh_in, options.attributes.position);
    assert(pos_handles.size() == 1);
    auto pos_handle = pos_handles.front();

    if (mesh_in->top_simplex_type() != PrimitiveType::Triangle) {
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

    if (options.length_abs < 0) {
        if (options.length_rel < 0) {
            throw std::runtime_error("Either absolute or relative length must be set!");
        }
        options.length_abs = relative_to_absolute_length(pos_handle, options.length_rel);
    }
    /////////////////////////////////////////////

    TriMesh& mesh = static_cast<TriMesh&>(*mesh_in);

    std::vector<attribute::MeshAttributeHandle> positions = other_positions;
    positions.push_back(pos_handle);

    auto invariant_link_condition = std::make_shared<MultiMeshLinkConditionInvariant>(mesh);

    auto invariant_max_edge_length = std::make_shared<MaxEdgeLengthInvariant>(
        mesh,
        pos_handle.as<double>(),
        options.length_abs * options.length_abs);

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

    auto update_position_func = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        return P.col(0);
    };
    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>
        update_position;

    if (options.attributes.update_other_positions) {
        update_position =
            std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
                other_positions.front(),
                pos_handle,
                update_position_func);
    }


    //////////////////////////////////////////
    // collapse
    auto op_collapse = std::make_shared<wmtk::operations::EdgeCollapse>(mesh);
    op_collapse->add_invariant(invariant_link_condition);
    if (position_for_inversion) {
        op_collapse->add_invariant(std::make_shared<SimplexInversionInvariant<double>>(
            position_for_inversion.value().mesh(),
            position_for_inversion.value().as<double>()));
    }

    op_collapse->add_invariant(invariant_max_edge_length);
    op_collapse->add_invariant(invariant_mm_map);

    // hack for uv
    if (options.fix_uv_seam) {
        op_collapse->add_invariant(
            std::make_shared<invariants::uvEdgeInvariant>(mesh, other_positions.front().mesh()));
    }

    if (options.lock_boundary && !options.use_for_periodic) {
        op_collapse->add_invariant(invariant_interior_edge);
        // set collapse towards boundary
        for (auto& p : positions) {
            auto tmp = std::make_shared<wmtk::operations::CollapseNewAttributeStrategy<double>>(p);
            tmp->set_strategy(wmtk::operations::CollapseBasicStrategy::Mean);
            tmp->set_simplex_predicate(wmtk::operations::BasicSimplexPredicate::IsInterior);
            op_collapse->set_new_attribute_strategy(p, tmp);
        }
    } else if (options.use_for_periodic) {
        op_collapse->add_invariant(
            std::make_shared<invariants::FusionEdgeInvariant>(mesh, mesh.get_multi_mesh_root()));
        for (auto& p : positions) {
            op_collapse->set_new_attribute_strategy(
                p,
                wmtk::operations::CollapseBasicStrategy::Mean);
        }
    } else {
        for (auto& p : positions) {
            op_collapse->set_new_attribute_strategy(
                p,
                wmtk::operations::CollapseBasicStrategy::Mean);
        }
    }


    for (const auto& attr : pass_through_attributes) {
        op_collapse->set_new_attribute_strategy(attr);
    }


    //////////////////////////////////////////
    Scheduler scheduler;
    for (long i = 0; i < options.iterations; ++i) {
        wmtk::logger().info("Iteration {}", i);

        SchedulerStats pass_stats = scheduler.run_operation_on_all(*op_collapse);

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

    // output
    cache.write_mesh(*mesh_in, options.output);
}
} // namespace wmtk::components
