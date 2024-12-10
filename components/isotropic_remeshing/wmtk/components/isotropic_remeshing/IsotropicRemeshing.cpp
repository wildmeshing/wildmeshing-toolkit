
#include "IsotropicRemeshing.hpp"
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/components/multimesh/MeshCollection.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/output/utils/format.hpp>
#include <wmtk/invariants/EnvelopeInvariant.hpp>
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

#include <wmtk/Mesh.hpp>
#include "internal/configure_collapse.hpp"
#include "internal/configure_split.hpp"
#include "internal/configure_swap.hpp"


namespace wmtk::components::isotropic_remeshing {
IsotropicRemeshing::~IsotropicRemeshing() = default;
IsotropicRemeshing::IsotropicRemeshing(const IsotropicRemeshingOptions& opts)
    : m_options(opts)
{
    if (m_options.envelope_size.has_value()) {
        for (const auto& h : all_envelope_positions()) {
            auto envelope_invariant = std::make_shared<invariants::EnvelopeInvariant>(
                h,
                m_options.envelope_size.value(),
                h);
            m_envelope_invariants.emplace_back(envelope_invariant);
        }
    }

    // split
    m_operations.emplace_back("split", m_split = configure_split());


    //////////////////////////////////////////
    // collapse

    m_operations.emplace_back("collapse", m_collapse = configure_collapse());


    //////////////////////////////////////////
    // swap

    m_operations.emplace_back("swap", m_swap = configure_swap());

    //////////////////////////////////////////
    // smooth
    m_operations.emplace_back("smooth", m_smooth = configure_smooth());
}

std::vector<wmtk::attribute::MeshAttributeHandle> IsotropicRemeshing::all_envelope_positions() const
{
    std::vector<wmtk::attribute::MeshAttributeHandle> handles;
    if (m_options.envelope_size.has_value()) {
        auto try_add = [&](const wmtk::attribute::MeshAttributeHandle& h) {
            if (is_envelope_position(h)) {
                handles.emplace_back(h);
            }
        };

        for (const auto& h : m_options.all_positions()) {
            try_add(h);
        }
    }
    return handles;
}

bool IsotropicRemeshing::is_envelope_position(const wmtk::attribute::MeshAttributeHandle& position)
{
    return position.mesh().top_cell_dimension() < position.dimension();
}


void IsotropicRemeshing::run()
{
    using namespace internal;


    // TODO: brig me back!
    // mesh_in->clear_attributes(keeps);

    // gather handles again as they were invalidated by clear_attributes
    // positions = utils::get_attributes(cache, *mesh_in,
    // m_options.position_attribute); assert(positions.size() == 1); position =
    // positions.front(); pass_through_attributes = utils::get_attributes(cache,
    // *mesh_in, m_options.pass_through_attributes);


    // assert(dynamic_cast<TriMesh*>(&position.mesh()) != nullptr);

    // TriMesh& mesh = static_cast<TriMesh&>(position.mesh());


    auto position = m_options.position_attribute;
    Mesh& mesh = position.mesh();



    auto log_mesh = [&](int64_t index) {
        if (m_options.intermediate_output_format.empty() && m_options.mesh_collection == nullptr) {
            wmtk::logger().error("Failed to log using intermediate_output_format because "
                                 "no MeshCollection was attached");
            return;
        }
        for (const auto& [name, opts] : m_options.intermediate_output_format) {
            auto opt = wmtk::components::output::utils::format(opts, index);
            wmtk::components::output::output(m_options.mesh_collection->get_mesh(name), opt);
        }
    };

    log_mesh(0);


    //////////////////////////////////////////
    Scheduler scheduler;
    for (long i = 1; i <= m_options.iterations; ++i) {
        wmtk::logger().info("Iteration {}", i);

        SchedulerStats pass_stats;
        for (const auto& [name, opptr] : m_operations) {
            if (!bool(opptr)) {
                spdlog::warn("op {} is empty", name);
                continue;
            }
            const auto stats = scheduler.run_operation_on_all(*opptr);
            logger().debug(
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


std::shared_ptr<operations::EdgeSplit> IsotropicRemeshing::configure_split()
{
    wmtk::logger().debug("Configure isotropic remeshing split");
    wmtk::Mesh& mesh = m_options.position_attribute.mesh();
    auto op = std::make_shared<operations::EdgeSplit>(mesh);
    internal::configure_split(*op, mesh, m_options);
    assert(op->attribute_new_all_configured());
    return op;
}
std::shared_ptr<operations::EdgeCollapse> IsotropicRemeshing::configure_collapse()
{
    wmtk::logger().debug("Configure isotropic remeshing collapse");
    wmtk::Mesh& mesh = m_options.position_attribute.mesh();
    auto op = std::make_shared<operations::EdgeCollapse>(mesh);
    internal::configure_collapse(*op, mesh, m_options);
    assert(op->attribute_new_all_configured());
    return op;
}
std::shared_ptr<operations::Operation> IsotropicRemeshing::configure_swap()
{
    // adds common invariants like inversion check and asserts taht the swap is ready for prime time
    wmtk::logger().debug("Configure isotropic remeshing swap");
    wmtk::Mesh& mesh = m_options.position_attribute.mesh();
    auto op = std::make_shared<operations::EdgeSplit>(mesh);
    internal::configure_split(*op, mesh, m_options);
    return op;
}

std::shared_ptr<operations::AttributesUpdateWithFunction> IsotropicRemeshing::configure_smooth()
{
    auto position = m_options.position_attribute;
    Mesh& mesh = position.mesh();
    auto pass_through_attributes = m_options.pass_through_attributes;
    auto other_positions = m_options.other_position_attributes;
    assert(mesh.is_connectivity_valid());

    std::vector<attribute::MeshAttributeHandle> positions = other_positions;
    positions.push_back(position);
    // if (position.mesh().top_simplex_type() != PrimitiveType::Triangle) {
    //     log_and_throw_error(
    //         "isotropic remeshing works only for triangle meshes: {}",
    //         primitive_type_name(position.mesh().top_simplex_type()));
    // }


    // clear attributes
    std::vector<attribute::MeshAttributeHandle> keeps = pass_through_attributes;
    keeps.emplace_back(position);
    keeps.insert(keeps.end(), other_positions.begin(), other_positions.end());

    auto op_smooth = std::make_shared<operations::AttributesUpdateWithFunction>(mesh);
    auto update_position_func = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        return P.col(0);
    };
    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>
        update_position;
    std::optional<attribute::MeshAttributeHandle> position_for_inversion =
        m_options.inversion_position_attribute;

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
    if (!m_options.other_position_attributes.empty()) {
        update_position =
            std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
                other_positions.front(),
                position,
                update_position_func);
    }

    if (position.dimension() == 3 && mesh.top_simplex_type() == PrimitiveType::Triangle) {
        op_smooth->set_function(operations::VertexTangentialLaplacianSmooth(position));
    } else {
        op_smooth->set_function(operations::VertexLaplacianSmooth(position));
    }

    if (m_options.lock_boundary) {
        op_smooth->add_invariant(invariant_interior_vertex);
    }

    // hack for uv
    if (m_options.fix_uv_seam) {
        op_smooth->add_invariant(
            std::make_shared<invariants::uvEdgeInvariant>(mesh, other_positions.front().mesh()));
    }

    if (position_for_inversion) {
        op_smooth->add_invariant(std::make_shared<SimplexInversionInvariant<double>>(
            position_for_inversion.value().mesh(),
            position_for_inversion.value().as<double>()));
    }

    if (update_position) op_smooth->add_transfer_strategy(update_position);
    return op_smooth;
}
} // namespace wmtk::components::isotropic_remeshing
