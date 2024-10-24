#include "configure_collapse.hpp"
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include "../IsotropicRemeshingOptions.hpp"
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/MultiMeshMapValidInvariant.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
namespace wmtk::components::isotropic_remeshing::internal {


std::shared_ptr<invariants::InvariantCollection> collapse_invariants(
    Mesh& m,
    const IsotropicRemeshingOptions& options)
{
    auto& root = m.get_multi_mesh_root();
    auto ic_root = std::make_shared<invariants::InvariantCollection>(root);

    auto invariant_link_condition =
        std::make_shared<wmtk::invariants::MultiMeshLinkConditionInvariant>(root);

    auto invariant_mm_map = std::make_shared<MultiMeshMapValidInvariant>(root);
    ic_root->add(invariant_link_condition);
    ic_root->add(invariant_mm_map);

    auto ic = std::make_shared<invariants::InvariantCollection>(root);
    const std::optional<attribute::MeshAttributeHandle>& position_for_inversion =
        options.inversion_position_attribute;


    if (position_for_inversion) {
        ic->add(std::make_shared<SimplexInversionInvariant<double>>(
            position_for_inversion.value().mesh(),
            position_for_inversion.value().as<double>()));
        ic->add(ic_root);
    }

    return ic;
}

void configure_collapse(operations::EdgeCollapse& ec, Mesh& m, const IsotropicRemeshingOptions& options)
{
    auto invars = collapse_invariants(m, options);
    ec.add_invariant(invars);
}
} // namespace wmtk::components::isotropic_remeshing::internal
