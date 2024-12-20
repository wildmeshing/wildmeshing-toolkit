
#include "IsotropicRemeshing.hpp"

// main execution tools
#include <wmtk/Scheduler.hpp>
#include <wmtk/multimesh/consolidate.hpp>


//
#include <wmtk/invariants/EnvelopeInvariant.hpp>
#include <wmtk/invariants/InteriorSimplexInvariant.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>

// utils for setting invariants
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/MultiMeshMapValidInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
// meshvisitor requires knowing all the mesh types
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/utils/Logger.hpp>

// for logging meshes
#include <wmtk/components/multimesh/MeshCollection.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/output/utils/format.hpp>


// op types
#include <wmtk/operations/AttributesUpdate.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/composite/EdgeSwap.hpp>
//
#include <Eigen/Geometry>


namespace wmtk::components::isotropic_remeshing {
IsotropicRemeshing::~IsotropicRemeshing() = default;
IsotropicRemeshing::IsotropicRemeshing(const IsotropicRemeshingOptions& opts)
    : m_options(opts)
{
    if (!m_options.position_attribute.is_valid()) {
        throw std::runtime_error("Isotropic remeshing run without a valid position attribute");
    }
    if (m_options.envelope_size.has_value()) {
        make_envelopes();
    }

    make_interior_invariants();


    // split
    if (m_options.use_split) {
        configure_split();
        assert(bool(m_split));
        m_operations.emplace_back("split", m_split);
    } else {
        wmtk::logger().info("Running Isotropic Remeshing without a split configured");
    }


    //////////////////////////////////////////
    // collapse

    if (m_options.use_collapse) {
        configure_collapse();
        assert(bool(m_collapse));
        m_operations.emplace_back("collapse", m_collapse);
    } else {
        wmtk::logger().info("Running Isotropic Remeshing without a collapse configured");
    }


    //////////////////////////////////////////
    // swap

    if (m_options.use_swap) {
        configure_swap();
        assert(bool(m_smooth));
        m_operations.emplace_back("swap", m_swap);
    } else if (m_options.edge_swap_mode != EdgeSwapMode::Skip) {
        wmtk::logger().info("Running Isotropic Remeshing without a swap configured despite being "
                            "supposed to use them");
    }

    //////////////////////////////////////////
    // smooth
    if (m_options.use_smooth) {
        configure_smooth();
        assert(bool(m_smooth));
        m_operations.emplace_back("smooth", m_smooth);
    } else {
        wmtk::logger().info("Running Isotropic Remeshing without a smooth configured");
    }
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


void IsotropicRemeshing::make_interior_invariants()
{
    auto position = m_options.position_attribute;
    Mesh& mesh = position.mesh();
    auto invariant_interior_vertex = std::make_shared<invariants::InvariantCollection>(mesh);
    m_interior_edge_invariants = std::make_shared<invariants::InvariantCollection>(mesh);

    auto set_all_invariants = [&](auto&& m) {
        // TODO: this used to do vertex+edge, but just checkign for vertex should be sufficient?
        for (PrimitiveType pt = PrimitiveType::Vertex; pt < m.top_simplex_type(); pt = pt + 1) {
            invariant_interior_vertex->add(
                std::make_shared<invariants::InteriorSimplexInvariant>(m, pt));
        }

        m_interior_edge_invariants->add(
            std::make_shared<invariants::InteriorSimplexInvariant>(m, PrimitiveType::Edge));
    };
    wmtk::multimesh::MultiMeshVisitor visitor(set_all_invariants);
    visitor.execute_from_root(mesh);
    m_interior_position_invariants = invariant_interior_vertex;
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

void IsotropicRemeshing::make_envelope_invariants()
{
    make_envelopes();
}

void IsotropicRemeshing::make_envelopes()
{
    if (!m_options.envelope_size.has_value()) {
        throw std::runtime_error(
            "Could not create envelopes because options do not have an envelope size");
    }
    auto envelope_positions = all_envelope_positions();

    std::vector<std::shared_ptr<invariants::EnvelopeInvariant>> envelope_invariants;


    std::transform(
        envelope_positions.begin(),
        envelope_positions.end(),
        std::back_inserter(envelope_invariants),
        [&](const wmtk::attribute::MeshAttributeHandle& mah) {
            return std::make_shared<invariants::EnvelopeInvariant>(
                mah,
                std::sqrt(2) * m_options.envelope_size.value(),
                mah);
        });

    m_envelope_invariants = std::make_shared<invariants::InvariantCollection>(
        m_options.position_attribute.mesh().get_multi_mesh_root());

    for (const auto& invar : envelope_invariants) {
        m_envelope_invariants->add(invar);
    }
}
} // namespace wmtk::components::isotropic_remeshing
