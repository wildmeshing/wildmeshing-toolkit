#include "configure_collapse.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/components/multimesh/MeshCollection.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/invariants/MaxEdgeLengthInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/MultiMeshMapValidInvariant.hpp>
#include <wmtk/invariants/SeparateSubstructuresInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/uvEdgeInvariant.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/attribute_new/CollapseNewAttributeStrategy.hpp>
#include "../IsotropicRemeshingOptions.hpp"
#include <wmtk/invariants/CannotMapSimplexInvariant.hpp>
namespace wmtk::components::isotropic_remeshing::internal {

std::shared_ptr<wmtk::invariants::InvariantCollection> collapse_core_invariants(
    Mesh& m,
    const IsotropicRemeshingOptions& options)
{
    auto& root = m.get_multi_mesh_root();
    auto ic_root = std::make_shared<wmtk::invariants::InvariantCollection>(root);
    auto invariant_link_condition =
        std::make_shared<wmtk::invariants::MultiMeshLinkConditionInvariant>(root);

    auto invariant_mm_map = std::make_shared<MultiMeshMapValidInvariant>(root);
    ic_root->add(invariant_link_condition);
    ic_root->add(invariant_mm_map);
    if (options.separate_substructures) {
        auto invariant_separate_substructures =
            std::make_shared<wmtk::invariants::SeparateSubstructuresInvariant>(root);
        ic_root->add(invariant_separate_substructures);
    }
    return ic_root;
}

std::shared_ptr<wmtk::invariants::InvariantCollection> collapse_invariants(
    Mesh& m,
    const IsotropicRemeshingOptions& options)
{
    auto ic_root = collapse_core_invariants(m, options);

    auto ic = std::make_shared<wmtk::invariants::InvariantCollection>(m);
    const std::optional<attribute::MeshAttributeHandle>& position_for_inversion =
        options.inversion_position_attribute;


    if (position_for_inversion) {
        ic->add(std::make_shared<SimplexInversionInvariant<double>>(
            position_for_inversion.value().mesh(),
            position_for_inversion.value().as<double>()));
    }

    ic->add(ic_root);
    const double length_min = (4. / 5.) * options.get_absolute_length();
    auto invariant_max_edge_length = std::make_shared<MaxEdgeLengthInvariant>(
        options.position_attribute.mesh(),
        options.position_attribute.as<double>(),
        length_min * length_min);
    ic->add(invariant_max_edge_length);

    if (options.mesh_collection != nullptr) {
        for (const auto& mesh_name : options.static_mesh_names) {
            auto& mesh2 = options.mesh_collection->get_mesh(mesh_name);
            ic_root->add(std::make_shared<wmtk::invariants::CannotMapSimplexInvariant>(
                m,
                mesh2,
                PrimitiveType::Vertex));
        }
    }
    /*

    // hack for uv
    if (options.fix_uv_seam) {
        assert(
            options.other_position_attributes.size() ==
            1); // this only works with a 2d pos + uv mesh for now

        ic->add(std::make_shared<invariants::uvEdgeInvariant>(
            m,
            options.other_position_attributes.front().mesh()));
    }


    if (options.lock_boundary && !options.use_for_periodic) {
        auto invariant_interior_edge = std::make_shared<invariants::InvariantCollection>(m);
        ic->add(invariant_interior_edge);
    }
    */
    return ic;
}

void configure_collapse(
    operations::EdgeCollapse& ec,
    Mesh& m,
    const IsotropicRemeshingOptions& options)
{
    auto invars = collapse_invariants(m, options);
    ec.add_invariant(invars);


    const auto positions = options.all_positions();

    if (options.lock_boundary && !options.use_for_periodic) {
        // set collapse towards boundary
        for (auto& p : positions) {
            auto tmp = std::make_shared<operations::CollapseNewAttributeStrategy<double>>(p);
            tmp->set_strategy(operations::CollapseBasicStrategy::Mean);
            tmp->set_simplex_predicate(operations::BasicSimplexPredicate::IsInterior);
            ec.set_new_attribute_strategy(p, tmp);
        }
    } else if (options.use_for_periodic) {
        assert(false); // TODO: make fusion simplex invariant
        ec.add_invariant(
            std::make_shared<wmtk::invariants::FusionEdgeInvariant>(m, m.get_multi_mesh_root()));
        for (auto& p : positions) {
            ec.set_new_attribute_strategy(p, operations::CollapseBasicStrategy::Mean);
        }
    } else {
        for (auto& p : positions) {
            ec.set_new_attribute_strategy(p, operations::CollapseBasicStrategy::Mean);
        }
    }

    for (const auto& attr : options.pass_through_attributes) {
        ec.set_new_attribute_strategy(attr);
    }
}
} // namespace wmtk::components::isotropic_remeshing::internal
