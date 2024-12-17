#include "configure_split.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/Mesh.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/invariants/MinEdgeLengthInvariant.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include "../IsotropicRemeshingOptions.hpp"
#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>

namespace wmtk::components::isotropic_remeshing::internal {
std::shared_ptr<invariants::InvariantCollection> split_invariants(
    Mesh& m,
    const IsotropicRemeshingOptions& options)
{
    auto ic = std::make_shared<invariants::InvariantCollection>(m);
    const double length_max = (4. / 3.) * options.get_absolute_length();
    auto invariant_min_edge_length = std::make_shared<MinEdgeLengthInvariant>(
        options.position_attribute.mesh(),
        options.position_attribute.as<double>(),
        length_max * length_max);
    ic->add(invariant_min_edge_length);
    if (options.lock_boundary && !options.use_for_periodic &&
        options.edge_swap_mode != EdgeSwapMode::Skip) {
        auto invariant_interior_edge = std::make_shared<invariants::InvariantCollection>(m);
        ic->add(invariant_interior_edge);
    }
    return ic;
}

void configure_split(operations::EdgeSplit& es, Mesh& m, const IsotropicRemeshingOptions& options)
{
    es.attribute_new_all_configured();
    auto invars = split_invariants(m, options);
    es.add_invariant(invars);
    for (auto& p : options.all_positions()) {
        es.set_new_attribute_strategy(p, operations::SplitBasicStrategy::None, operations::SplitRibBasicStrategy::Mean);
    }
    for (const auto& attr : options.pass_through_attributes) {
        es.set_new_attribute_strategy(attr);
    }
    assert(es.attribute_new_all_configured());
}

} // namespace wmtk::components::isotropic_remeshing::internal
