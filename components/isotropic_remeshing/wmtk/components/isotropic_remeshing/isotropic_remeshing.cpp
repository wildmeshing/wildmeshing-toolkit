#include "isotropic_remeshing.hpp"

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/components/multimesh/MeshCollection.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/output/utils/format.hpp>
#include <wmtk/invariants/FusionEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorSimplexInvariant.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/MultiMeshMapValidInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/uvEdgeInvariant.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include <wmtk/multimesh/consolidate.hpp>
#include <wmtk/operations/AttributesUpdate.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/operations/utils/VertexLaplacianSmooth.hpp>
#include <wmtk/operations/utils/VertexTangentialLaplacianSmooth.hpp>
#include <wmtk/utils/Logger.hpp>


#include <Eigen/Geometry>
#include <wmtk/invariants/InvariantCollection.hpp>
#include "IsotropicRemeshingOptions.hpp"
#include "internal/configure_collapse.hpp"
#include "internal/configure_split.hpp"
#include "internal/configure_swap.hpp"

namespace wmtk::components::isotropic_remeshing {


void isotropic_remeshing(const IsotropicRemeshingOptions& options)
{
    using namespace internal;


    auto position = options.position_attribute;

    // if (position.mesh().top_simplex_type() != PrimitiveType::Triangle) {
    //     log_and_throw_error(
    //         "isotropic remeshing works only for triangle meshes: {}",
    //         primitive_type_name(position.mesh().top_simplex_type()));
    // }

    auto pass_through_attributes = options.pass_through_attributes;
    auto other_positions = options.other_position_attributes;


    // clear attributes
    std::vector<attribute::MeshAttributeHandle> keeps = pass_through_attributes;
    keeps.emplace_back(position);
    keeps.insert(keeps.end(), other_positions.begin(), other_positions.end());

    // TODO: brig me back!
    // mesh_in->clear_attributes(keeps);

    // gather handles again as they were invalidated by clear_attributes
    // positions = utils::get_attributes(cache, *mesh_in,
    // options.position_attribute); assert(positions.size() == 1); position =
    // positions.front(); pass_through_attributes = utils::get_attributes(cache,
    // *mesh_in, options.pass_through_attributes);

    std::optional<attribute::MeshAttributeHandle> position_for_inversion =
        options.inversion_position_attribute;


    // assert(dynamic_cast<TriMesh*>(&position.mesh()) != nullptr);

    // TriMesh& mesh = static_cast<TriMesh&>(position.mesh());
    Mesh& mesh = position.mesh();


    std::vector<attribute::MeshAttributeHandle> positions = other_positions;
    positions.push_back(position);

    auto invariant_link_condition =
        std::make_shared<wmtk::invariants::MultiMeshLinkConditionInvariant>(mesh);


    auto invariant_interior_vertex = std::make_shared<invariants::InvariantCollection>(mesh);

    auto set_all_invariants = [&](auto&& m) {
        // TODO: this used to do vertex+edge, but just checkign for vertex should be sufficient?
        for (PrimitiveType pt = PrimitiveType::Vertex; pt < m.top_simplex_type(); pt = pt + 1) {
            invariant_interior_vertex->add(
                std::make_shared<invariants::InteriorSimplexInvariant>(m, pt));
        }
    };
    wmtk::multimesh::MultiMeshVisitor visitor(set_all_invariants);
    visitor.execute_from_root(mesh);


    auto update_position_func = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        return P.col(0);
    };
    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>
        update_position;

    if (!options.other_position_attributes.empty()) {
        update_position =
            std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
                other_positions.front(),
                position,
                update_position_func);
    }

    using namespace operations;

    assert(mesh.is_connectivity_valid());

    std::vector<std::pair<std::string, std::shared_ptr<Operation>>> ops;

    // split
    wmtk::logger().debug("Configure isotropic remeshing split");
    auto op_split = std::make_shared<EdgeSplit>(mesh);
    configure_split(*op_split, mesh, options);
    assert(op_split->attribute_new_all_configured());
    ops.emplace_back("split", op_split);


    //////////////////////////////////////////
    // collapse
    wmtk::logger().debug("Configure isotropic remeshing collapse");
    auto op_collapse = std::make_shared<EdgeCollapse>(mesh);


    configure_collapse(*op_collapse, mesh, options);


    assert(op_collapse->attribute_new_all_configured());
    ops.emplace_back("collapse", op_collapse);


    //////////////////////////////////////////
    // swap
    std::shared_ptr<operations::Operation> op_swap = configure_swap(mesh, options);

    // adds common invariants like inversion check and asserts taht the swap is ready for prime time
    wmtk::logger().debug("Configure isotropic remeshing swap");

    ops.emplace_back("swap", op_swap);


    //////////////////////////////////////////
    // smooth
    auto op_smooth = std::make_shared<AttributesUpdateWithFunction>(mesh);
    if (position.dimension() == 3 && mesh.top_simplex_type() == PrimitiveType::Triangle) {
        op_smooth->set_function(VertexTangentialLaplacianSmooth(position));
    } else {
        op_smooth->set_function(VertexLaplacianSmooth(position));
    }

    if (options.lock_boundary) {
        op_smooth->add_invariant(invariant_interior_vertex);
    }

    // hack for uv
    if (options.fix_uv_seam) {
        op_smooth->add_invariant(
            std::make_shared<invariants::uvEdgeInvariant>(mesh, other_positions.front().mesh()));
    }

    if (position_for_inversion) {
        op_smooth->add_invariant(std::make_shared<SimplexInversionInvariant<double>>(
            position_for_inversion.value().mesh(),
            position_for_inversion.value().as<double>()));
    }

    if (update_position) op_smooth->add_transfer_strategy(update_position);
    ops.emplace_back("smooth", op_smooth);

    auto log_mesh = [&](int64_t index) {
        spdlog::warn(
            "{} {}",
            options.intermediate_output_format.empty(),
            options.mesh_collection == nullptr);
        if (options.intermediate_output_format.empty() && options.mesh_collection == nullptr) {
            wmtk::logger().error("Failed to log using intermediate_output_format because "
                                 "no MeshCollection was attached");
            return;
        }
        for (const auto& [name, opts] : options.intermediate_output_format) {
            auto opt = wmtk::components::output::utils::format(opts, index);
            wmtk::components::output::output(options.mesh_collection->get_mesh(name), opt);
        }
    };

    log_mesh(0);


    //////////////////////////////////////////
    Scheduler scheduler;
    for (long i = 1; i <= options.iterations; ++i) {
        wmtk::logger().info("Iteration {}", i);

        SchedulerStats pass_stats;
        for (const auto& [name, opptr] : ops) {
            if (!bool(opptr)) {
                spdlog::warn("op {} is empty", name);
                continue;
            }
            const auto stats = scheduler.run_operation_on_all(*opptr);
            logger().info(
                "Executed {} {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, executing: {}",
                stats.number_of_performed_operations(),
                name,
                stats.number_of_successful_operations(),
                stats.number_of_failed_operations(),
                stats.collecting_time,
                stats.sorting_time,
                stats.executing_time);
            pass_stats += stats;
        }

        wmtk::multimesh::consolidate(mesh);

        logger().info(
            "Executed {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, executing: {}",
            pass_stats.number_of_performed_operations(),
            pass_stats.number_of_successful_operations(),
            pass_stats.number_of_failed_operations(),
            pass_stats.collecting_time,
            pass_stats.sorting_time,
            pass_stats.executing_time);

        wmtk::multimesh::consolidate(mesh);
        log_mesh(i);
    }
}
} // namespace wmtk::components::isotropic_remeshing
