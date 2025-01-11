
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/MultiMeshMapValidInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/utils/Logger.hpp>
#include "IsotropicRemeshing.hpp"


#include <Eigen/Geometry>
#include <wmtk/invariants/InvariantCollection.hpp>

#include <wmtk/Mesh.hpp>
#include "internal/configure_collapse.hpp"

namespace wmtk::components::isotropic_remeshing {
void IsotropicRemeshing::configure_collapse()
{
    wmtk::logger().debug("Configure isotropic remeshing collapse");
    wmtk::Mesh& mesh = m_options.position_attribute.mesh();
    auto op = std::make_shared<operations::EdgeCollapse>(mesh);
    internal::configure_collapse(*op, mesh, m_options);

    if (m_envelope_invariants) {
        spdlog::info("Attaching envelope invariants");
        op->add_invariant(m_envelope_invariants);
    }
    assert(op->attribute_new_all_configured());
    m_collapse = op;
}
} // namespace wmtk::components::isotropic_remeshing
