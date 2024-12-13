#include "configure_swap.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/SeparateSubstructuresInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/ValenceImprovementInvariant.hpp>
#include <wmtk/invariants/uvEdgeInvariant.hpp>
#include <wmtk/operations/attribute_new/CollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>
#include "../IsotropicRemeshingOptions.hpp"
#include "configure_collapse.hpp"

namespace wmtk::components::isotropic_remeshing::internal {
std::shared_ptr<wmtk::operations::Operation> tet_swap(
    TetMesh& mesh,
    const IsotropicRemeshingOptions& options);


void finalize_swap(operations::composite::EdgeSwap& op, const IsotropicRemeshingOptions& options)
{
    assert(op.split().attribute_new_all_configured());
    assert(op.collapse().attribute_new_all_configured());
};

namespace {
std::shared_ptr<wmtk::operations::composite::EdgeSwap> tri_swap(
    TriMesh& mesh,
    const IsotropicRemeshingOptions& options)
{
    auto swap = std::make_shared<wmtk::operations::composite::TriEdgeSwap>(mesh);
    // hack for uv
    if (options.fix_uv_seam) {
        swap->add_invariant(std::make_shared<invariants::uvEdgeInvariant>(
            mesh,
            options.other_position_attributes.front().mesh()));
    }

    auto invariant_valence_improve =
        std::make_shared<invariants::ValenceImprovementInvariant>(mesh);

    auto collapse_invars = collapse_invariants(mesh, options);
    swap->add_invariant(invariant_valence_improve);
    swap->add_invariant(collapse_invars);

    for (const auto& p : options.all_positions()) {
        swap->split().set_new_attribute_strategy(
            p,
            wmtk::operations::SplitBasicStrategy::None,
            wmtk::operations::SplitRibBasicStrategy::Mean);
        swap->collapse().set_new_attribute_strategy(
            p,
            wmtk::operations::CollapseBasicStrategy::CopyOther);
    }
    for (const auto& attr : options.pass_through_attributes) {
        swap->split().set_new_attribute_strategy(attr);
        swap->collapse().set_new_attribute_strategy(attr);
    }
    finalize_swap(*swap, options);
    return swap;
}

} // namespace


void configure_swap_transfer(
    operations::composite::EdgeSwap& swap,
    const attribute::MeshAttributeHandle& vertex_handle)
{
    swap.split().set_new_attribute_strategy(vertex_handle);
    swap.collapse().set_new_attribute_strategy(
        vertex_handle,
        wmtk::operations::CollapseBasicStrategy::CopyOther);
}

} // namespace wmtk::components::isotropic_remeshing::internal
