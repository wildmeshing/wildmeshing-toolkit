#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/components/multimesh/MeshCollection.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/utils/Logger.hpp>
#include "IsotropicRemeshing.hpp"


#include <Eigen/Geometry>
#include <wmtk/invariants/InvariantCollection.hpp>

#include <wmtk/Mesh.hpp>
#include "internal/configure_collapse.hpp"
#include "internal/configure_split.hpp"
#include "internal/configure_swap.hpp"

namespace wmtk::components::isotropic_remeshing {
void IsotropicRemeshing::configure_split()
{
    wmtk::logger().debug("Configure isotropic remeshing split");
    wmtk::Mesh& mesh = m_options.position_attribute.mesh();
    auto op = std::make_shared<operations::EdgeSplit>(mesh);
    internal::configure_split(*op, mesh, m_options);

    if (m_options.lock_boundary && !m_options.use_for_periodic) {
        op->add_invariant(m_interior_position_invariants);
    }

    assert(op->attribute_new_all_configured());
    m_split = op;
}
} // namespace wmtk::components::isotropic_remeshing
