#include "configure_swap.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include "configure_collapse.hpp"
//#include <wmtk/invariants/Swap23EnergyBeforeInvariant.hpp>
//#include <wmtk/invariants/Swap32EnergyBeforeInvariant.hpp>
//#include <wmtk/invariants/Swap44EnergyBeforeInvariant.hpp>
//#include <wmtk/invariants/Swap56EnergyBeforeInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/ValenceImprovementInvariant.hpp>
#include <wmtk/invariants/uvEdgeInvariant.hpp>
#include <wmtk/operations/attribute_new/CollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>
#include "../IsotropicRemeshingOptions.hpp"

namespace wmtk::components::isotropic_remeshing::internal {
namespace {
void finalize_swap(operations::composite::EdgeSwap& op, const IsotropicRemeshingOptions& options)
{
    assert(op.split().attribute_new_all_configured());
    assert(op.collapse().attribute_new_all_configured());
};

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
        swap->collapse().set_new_attribute_strategy(p, wmtk::operations::CollapseBasicStrategy::CopyOther);
    }
    for (const auto& attr : options.pass_through_attributes) {
        swap->split().set_new_attribute_strategy(attr);
        swap->collapse().set_new_attribute_strategy(attr);
    }
    finalize_swap(*swap, options);
    return swap;
}

// std::shared_ptr<wmtk::operations::Operation>
// tet_swap56(TetMesh& mesh, const IsotropicRemeshingOptions& options, int64_t index)
//{
//     TetMesh& tetmesh = static_cast<TetMesh&>(mesh);
//     auto swap56 = std::make_shared<MinOperationSequence>(*mesh);
//     for (int i = 0; i < 5; ++i) {
//         auto swap = std::make_shared<TetEdgeSwap>(*mesh, i);
//         swap->collapse().add_invariant(invariant_separate_substructures);
//         swap->collapse().add_invariant(link_condition);
//         swap->collapse().set_new_attribute_strategy(pt_attribute,
//         CollapseBasicStrategy::CopyOther); swap->collapse().set_new_attribute_strategy(
//             sizing_field_scalar_attribute,
//             CollapseBasicStrategy::CopyOther);
//         swap->split().set_new_attribute_strategy(pt_attribute);
//         swap->split().set_new_attribute_strategy(sizing_field_scalar_attribute);
//        // swap->split().add_transfer_strategy(amips_update);
//        // swap->collapse().add_transfer_strategy(amips_update);
//        swap->split().set_new_attribute_strategy(
//            visited_edge_flag,
//            wmtk::operations::SplitBasicStrategy::None,
//            wmtk::operations::SplitRibBasicStrategy::None);
//        swap->collapse().set_new_attribute_strategy(
//            visited_edge_flag,
//            wmtk::operations::CollapseBasicStrategy::None);
//
//        swap->split().set_new_attribute_strategy(
//            target_edge_length_attribute,
//            wmtk::operations::SplitBasicStrategy::Copy,
//            wmtk::operations::SplitRibBasicStrategy::Mean);
//        swap->collapse().set_new_attribute_strategy(
//            target_edge_length_attribute,
//            wmtk::operations::CollapseBasicStrategy::None);
//
//        swap->add_invariant(
//            std::make_shared<Swap56EnergyBeforeInvariant>(*mesh, pt_attribute.as<Rational>(), i));
//
//        swap->add_transfer_strategy(amips_update);
//
//        // swap->add_invariant(inversion_invariant);
//        swap->collapse().add_invariant(inversion_invariant);
//
//        // swap->collapse().add_invariant(envelope_invariant);
//
//        for (const auto& attr : pass_through_attributes) {
//            swap->split().set_new_attribute_strategy(
//                attr,
//                wmtk::operations::SplitBasicStrategy::None,
//                wmtk::operations::SplitRibBasicStrategy::None);
//            swap->collapse().set_new_attribute_strategy(
//                attr,
//                wmtk::operations::CollapseBasicStrategy::None);
//        }
//
//        finalize_swap(*swap);
//        swap56->add_operation(swap);
//    }
//    op_swap = swap56;
//    return op_swap;
//}
//
//
// std::shared_ptr<wmtk::operations::Operation>
// tet_swap44(TetMesh& mesh, const IsotropicRemeshingOptions& options, int64_t index)
//{
//    // swap44
//
//    auto swap44 = std::make_shared<MinOperationSequence>(*mesh);
//    for (int i = 0; i < 2; ++i) {
//        auto swap = std::make_shared<TetEdgeSwap>(*mesh, i);
//        swap->collapse().add_invariant(invariant_separate_substructures);
//        swap->collapse().add_invariant(link_condition);
//        swap->collapse().set_new_attribute_strategy(pt_attribute,
//        CollapseBasicStrategy::CopyOther); swap->collapse().set_new_attribute_strategy(
//            sizing_field_scalar_attribute,
//            CollapseBasicStrategy::CopyOther);
//        swap->split().set_new_attribute_strategy(pt_attribute);
//        swap->split().set_new_attribute_strategy(sizing_field_scalar_attribute);
//        swap->split().set_new_attribute_strategy(
//            visited_edge_flag,
//            wmtk::operations::SplitBasicStrategy::None,
//            wmtk::operations::SplitRibBasicStrategy::None);
//        swap->collapse().set_new_attribute_strategy(
//            visited_edge_flag,
//            wmtk::operations::CollapseBasicStrategy::None);
//
//        // swap->split().add_transfer_strategy(amips_update);
//        // swap->collapse().add_transfer_strategy(amips_update);
//
//        swap->split().set_new_attribute_strategy(
//            target_edge_length_attribute,
//            wmtk::operations::SplitBasicStrategy::Copy,
//            wmtk::operations::SplitRibBasicStrategy::Mean);
//        swap->collapse().set_new_attribute_strategy(
//            target_edge_length_attribute,
//            wmtk::operations::CollapseBasicStrategy::None);
//
//        swap->add_transfer_strategy(amips_update);
//
//        swap->add_invariant(
//            std::make_shared<Swap44EnergyBeforeInvariant>(*mesh, pt_attribute.as<Rational>(), i));
//
//        // swap->add_invariant(inversion_invariant);
//        swap->collapse().add_invariant(inversion_invariant);
//
//        // swap->collapse().add_invariant(envelope_invariant);
//
//        for (const auto& attr : pass_through_attributes) {
//            swap->split().set_new_attribute_strategy(
//                attr,
//                wmtk::operations::SplitBasicStrategy::None,
//                wmtk::operations::SplitRibBasicStrategy::None);
//            swap->collapse().set_new_attribute_strategy(
//                attr,
//                wmtk::operations::CollapseBasicStrategy::None);
//        }
//
//        swap44->add_operation(swap);
//    }
//
//    auto swap44_energy_check = [&](int64_t idx, const simplex::Simplex& t) -> double {
//        constexpr static PrimitiveType PV = PrimitiveType::Vertex;
//        constexpr static PrimitiveType PE = PrimitiveType::Edge;
//        constexpr static PrimitiveType PF = PrimitiveType::Triangle;
//        constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;
//
//        auto accessor = mesh->create_const_accessor(pt_attribute.as<Rational>());
//
//        // get the coords of the vertices
//        // input edge end points
//        const Tuple e0 = t.tuple();
//        const Tuple e1 = mesh->switch_tuple(e0, PV);
//        // other four vertices
//        std::array<Tuple, 4> v;
//        auto iter_tuple = e0;
//        for (int64_t i = 0; i < 4; ++i) {
//            v[i] = mesh->switch_tuples(iter_tuple, {PE, PV});
//            iter_tuple = mesh->switch_tuples(iter_tuple, {PF, PT});
//        }
//
//        if (iter_tuple != e0) return 0;
//        assert(iter_tuple == e0);
//
//        std::array<Eigen::Vector3<Rational>, 6> positions = {
//            {accessor.const_vector_attribute(v[(idx + 0) % 4]),
//             accessor.const_vector_attribute(v[(idx + 1) % 4]),
//             accessor.const_vector_attribute(v[(idx + 2) % 4]),
//             accessor.const_vector_attribute(v[(idx + 3) % 4]),
//             accessor.const_vector_attribute(e0),
//             accessor.const_vector_attribute(e1)}};
//        std::array<Eigen::Vector3d, 6> positions_double = {
//            {positions[0].cast<double>(),
//             positions[1].cast<double>(),
//             positions[2].cast<double>(),
//             positions[3].cast<double>(),
//             positions[4].cast<double>(),
//             positions[5].cast<double>()}};
//
//        std::array<std::array<int, 4>, 4> new_tets = {
//            {{{0, 1, 2, 4}}, {{0, 2, 3, 4}}, {{0, 1, 2, 5}}, {{0, 2, 3, 5}}}};
//
//        double new_energy_max = std::numeric_limits<double>::lowest();
//
//        for (int i = 0; i < 4; ++i) {
//            if (wmtk::utils::wmtk_orient3d(
//                    positions[new_tets[i][0]],
//                    positions[new_tets[i][1]],
//                    positions[new_tets[i][2]],
//                    positions[new_tets[i][3]]) > 0) {
//                auto energy = wmtk::function::utils::Tet_AMIPS_energy({{
//                    positions_double[new_tets[i][0]][0],
//                    positions_double[new_tets[i][0]][1],
//                    positions_double[new_tets[i][0]][2],
//                    positions_double[new_tets[i][1]][0],
//                    positions_double[new_tets[i][1]][1],
//                    positions_double[new_tets[i][1]][2],
//                    positions_double[new_tets[i][2]][0],
//                    positions_double[new_tets[i][2]][1],
//                    positions_double[new_tets[i][2]][2],
//                    positions_double[new_tets[i][3]][0],
//                    positions_double[new_tets[i][3]][1],
//                    positions_double[new_tets[i][3]][2],
//                }});
//
//                if (energy > new_energy_max) new_energy_max = energy;
//            } else {
//                auto energy = wmtk::function::utils::Tet_AMIPS_energy({{
//                    positions_double[new_tets[i][1]][0],
//                    positions_double[new_tets[i][1]][1],
//                    positions_double[new_tets[i][1]][2],
//                    positions_double[new_tets[i][0]][0],
//                    positions_double[new_tets[i][0]][1],
//                    positions_double[new_tets[i][0]][2],
//                    positions_double[new_tets[i][2]][0],
//                    positions_double[new_tets[i][2]][1],
//                    positions_double[new_tets[i][2]][2],
//                    positions_double[new_tets[i][3]][0],
//                    positions_double[new_tets[i][3]][1],
//                    positions_double[new_tets[i][3]][2],
//                }});
//
//                if (energy > new_energy_max) new_energy_max = energy;
//            }
//        }
//
//        return new_energy_max;
//    };
//
//    swap44->set_value_function(swap44_energy_check);
//    swap44->add_invariant(std::make_shared<EdgeValenceInvariant>(*mesh, 4));
//}
// std::shared_ptr<wmtk::operations::Operation>
// tet_swap44(TetMesh& mesh, const IsotropicRemeshingOptions& options, int64_t index)
//{
//    // swap 32
//    auto swap32 = std::make_shared<TetEdgeSwap>(*mesh, 0);
//    swap32->add_invariant(std::make_shared<EdgeValenceInvariant>(*mesh, 3));
//    swap32->add_invariant(
//        std::make_shared<Swap32EnergyBeforeInvariant>(*mesh, pt_attribute.as<Rational>()));
//
//    swap32->collapse().add_invariant(invariant_separate_substructures);
//    swap32->collapse().add_invariant(link_condition);
//    swap32->collapse().set_new_attribute_strategy(pt_attribute, CollapseBasicStrategy::CopyOther);
//    swap32->collapse().set_new_attribute_strategy(
//        sizing_field_scalar_attribute,
//        CollapseBasicStrategy::CopyOther);
//    // swap32->add_invariant(inversion_invariant);
//    swap32->split().set_new_attribute_strategy(pt_attribute);
//    swap32->split().set_new_attribute_strategy(sizing_field_scalar_attribute);
//    swap32->split().set_new_attribute_strategy(
//        visited_edge_flag,
//        wmtk::operations::SplitBasicStrategy::None,
//        wmtk::operations::SplitRibBasicStrategy::None);
//    swap32->collapse().set_new_attribute_strategy(
//        visited_edge_flag,
//        wmtk::operations::CollapseBasicStrategy::None);
//
//    // swap32->split().add_transfer_strategy(amips_update);
//    // swap32->collapse().add_transfer_strategy(amips_update);
//
//    swap32->split().set_new_attribute_strategy(
//        target_edge_length_attribute,
//        wmtk::operations::SplitBasicStrategy::Copy,
//        wmtk::operations::SplitRibBasicStrategy::Mean);
//    swap32->collapse().set_new_attribute_strategy(
//        target_edge_length_attribute,
//        wmtk::operations::CollapseBasicStrategy::None);
//
//    swap32->add_transfer_strategy(amips_update);
//
//    // hack
//    swap32->collapse().add_invariant(inversion_invariant);
//    // swap32->collapse().add_invariant(envelope_invariant);
//
//    for (const auto& attr : pass_through_attributes) {
//        swap32->split().set_new_attribute_strategy(
//            attr,
//            wmtk::operations::SplitBasicStrategy::None,
//            wmtk::operations::SplitRibBasicStrategy::None);
//        swap32->collapse().set_new_attribute_strategy(
//            attr,
//            wmtk::operations::CollapseBasicStrategy::None);
//    }
//}
} // namespace

std::shared_ptr<wmtk::operations::Operation> configure_swap(
    Mesh& mesh,
    const IsotropicRemeshingOptions& options)
{
    if (mesh.top_simplex_type() == PrimitiveType::Triangle) {
        return std::static_pointer_cast<wmtk::operations::Operation>(
            tri_swap(static_cast<TriMesh&>(mesh), options));
    } else {
        // auto swap_all = std::make_shared<OrOperationSequence>(*mesh);
        // swap_all->add_operation(swap32);
        // swap_all->add_operation(swap44);
        // swap_all->add_operation(swap56);
        // swap_all->add_transfer_strategy(tag_update);
        // swap_all->add_transfer_strategy(energy_filter_update);
    }
    return {};
}
} // namespace wmtk::components::isotropic_remeshing::internal
