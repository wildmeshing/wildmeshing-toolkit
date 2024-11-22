
#include <wmtk/function/utils/amips.hpp>
#include <wmtk/invariants/Swap23EnergyBeforeInvariant.hpp>
#include <wmtk/invariants/Swap32EnergyBeforeInvariant.hpp>
#include <wmtk/invariants/Swap44EnergyBeforeInvariant.hpp>
#include <wmtk/invariants/Swap56EnergyBeforeInvariant.hpp>
#include <wmtk/invariants/EdgeValenceInvariant.hpp>
#include <wmtk/operations/MinOperationSequence.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/operations/composite/TetEdgeSwap.hpp>
#include "../IsotropicRemeshingOptions.hpp"
#include "configure_collapse.hpp"
#include "configure_split.hpp"
#include "configure_swap.hpp"
#include <wmtk/utils/orient.hpp>
namespace wmtk::components::isotropic_remeshing::internal {

namespace {

auto compute_tet_amips = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
    // tet
    std::array<double, 12> pts;
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            pts[3 * i + j] = P(j, i);
        }
    }
    const double a = wmtk::function::utils::Tet_AMIPS_energy(pts);
    return Eigen::VectorXd::Constant(1, a);
};
auto get_tet_amips_transfer_strategy(
    const wmtk::attribute::MeshAttributeHandle& position_handle,
    const wmtk::attribute::MeshAttributeHandle& amips_handle)
{
    return std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
        amips_handle,
        position_handle,
        compute_tet_amips);
}

auto initiate_amips_transfer_attribute(
    wmtk::Mesh& mesh,
    const wmtk::attribute::MeshAttributeHandle& position_handle,
    const std::string& amips_attribute_name = "amips",
    bool initiate_values = true)
{
    auto amips_handle =
        mesh.register_attribute<double>(amips_attribute_name, PrimitiveType::Tetrahedron, 1);

    if (initiate_values) {
        auto amips_update = get_tet_amips_transfer_strategy(position_handle, amips_handle);
        amips_update->run_on_all();
    }
    return amips_handle;
}


void configure_core_tet_swap(
    operations::composite::TetEdgeSwap& swap,
    const IsotropicRemeshingOptions& options)
{
    swap.collapse().add_invariant(collapse_core_invariants(swap.mesh(), options));

    for (const auto& pos_attr : options.all_positions()) {
        swap.collapse().set_new_attribute_strategy(
            pos_attr,
            operations::CollapseBasicStrategy::CopyOther);
        swap.split().set_new_attribute_strategy(pos_attr);
    }
    if (const auto& sfa_opt = options.sizing_field_attribute; sfa_opt.has_value()) {
        swap.collapse().set_new_attribute_strategy(
            sfa_opt.value(),
            operations::CollapseBasicStrategy::CopyOther);
        swap.split().set_new_attribute_strategy(sfa_opt.value());
    }
    // swap.split().add_transfer_strategy(amips_update);
    // swap.collapse().add_transfer_strategy(amips_update);
    if (const auto& flag_h = options.visited_edge_flag; flag_h.has_value()) {
        swap.split().set_new_attribute_strategy(
            flag_h.value(),
            wmtk::operations::SplitBasicStrategy::None,
            wmtk::operations::SplitRibBasicStrategy::None);
        swap.collapse().set_new_attribute_strategy(
            flag_h.value(),
            wmtk::operations::CollapseBasicStrategy::None);
    }

    if (const auto& edge_length_h = options.target_edge_length; edge_length_h.has_value()) {
        swap.split().set_new_attribute_strategy(
            edge_length_h.value(),
            wmtk::operations::SplitBasicStrategy::Copy,
            wmtk::operations::SplitRibBasicStrategy::Mean);
        swap.collapse().set_new_attribute_strategy(
            edge_length_h.value(),
            wmtk::operations::CollapseBasicStrategy::None);
    }
    for (const auto& attr : options.pass_through_attributes) {
        swap.split().set_new_attribute_strategy(
            attr,
            wmtk::operations::SplitBasicStrategy::None,
            wmtk::operations::SplitRibBasicStrategy::None);
        swap.collapse().set_new_attribute_strategy(
            attr,
            wmtk::operations::CollapseBasicStrategy::None);
    }
}

} // namespace
std::shared_ptr<wmtk::operations::Operation>
tet_swap56(TetMesh& mesh, const IsotropicRemeshingOptions& options, int64_t index)
{
    TetMesh& tetmesh = static_cast<TetMesh&>(mesh);
    auto swap56 = std::make_shared<operations::MinOperationSequence>(mesh);
    for (int i = 0; i < 5; ++i) {
        auto swap = std::make_shared<operations::composite::TetEdgeSwap>(mesh, i);

        configure_core_tet_swap(*swap, options);

        assert(options.position_attribute.holds<wmtk::Rational>());
        swap->add_invariant(std::make_shared<Swap56EnergyBeforeInvariant>(
            mesh,
            options.position_attribute.as<Rational>(),
            i));

        auto amips_handle = initiate_amips_transfer_attribute(mesh, options.position_attribute);
        swap->add_transfer_strategy(
            get_tet_amips_transfer_strategy(options.position_attribute, amips_handle));


        // swap->collapse().add_invariant(envelope_invariant);


        finalize_swap(*swap, options);
        swap56->add_operation(swap);
    }
    return std::static_pointer_cast<wmtk::operations::Operation>(swap56);
}
//
//
std::shared_ptr<wmtk::operations::Operation>
tet_swap44(TetMesh& mesh, const IsotropicRemeshingOptions& options, int64_t index)
{
    // swap44

    auto swap44 = std::make_shared<operations::MinOperationSequence>(mesh);
    for (int i = 0; i < 2; ++i) {
        auto swap = std::make_shared<operations::composite::TetEdgeSwap>(mesh, i);

        configure_core_tet_swap(*swap, options);

        swap->add_invariant(std::make_shared<Swap44EnergyBeforeInvariant>(
            mesh,
            options.position_attribute.as<Rational>(),
            i));


        // swap->collapse().add_invariant(envelope_invariant);


        swap44->add_operation(swap);
    }

    auto swap44_energy_check = [&](int64_t idx, const simplex::Simplex& t) -> double {
        constexpr static PrimitiveType PV = PrimitiveType::Vertex;
        constexpr static PrimitiveType PE = PrimitiveType::Edge;
        constexpr static PrimitiveType PF = PrimitiveType::Triangle;
        constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;

        assert(options.position_attribute.holds<wmtk::Rational>());
        auto accessor = mesh.create_const_accessor(options.position_attribute.as<Rational>());

        // get the coords of the vertices
        // input edge end points
        const Tuple e0 = t.tuple();
        const Tuple e1 = mesh.switch_tuple(e0, PV);
        // other four vertices
        std::array<Tuple, 4> v;
        auto iter_tuple = e0;
        for (int64_t i = 0; i < 4; ++i) {
            v[i] = mesh.switch_tuples(iter_tuple, {PE, PV});
            iter_tuple = mesh.switch_tuples(iter_tuple, {PF, PT});
        }

        if (iter_tuple != e0) return 0;
        assert(iter_tuple == e0);

        std::array<Eigen::Vector3<Rational>, 6> positions = {
            {accessor.const_vector_attribute(v[(idx + 0) % 4]),
             accessor.const_vector_attribute(v[(idx + 1) % 4]),
             accessor.const_vector_attribute(v[(idx + 2) % 4]),
             accessor.const_vector_attribute(v[(idx + 3) % 4]),
             accessor.const_vector_attribute(e0),
             accessor.const_vector_attribute(e1)}};
        std::array<Eigen::Vector3d, 6> positions_double = {
            {positions[0].cast<double>(),
             positions[1].cast<double>(),
             positions[2].cast<double>(),
             positions[3].cast<double>(),
             positions[4].cast<double>(),
             positions[5].cast<double>()}};

        std::array<std::array<int, 4>, 4> new_tets = {
            {{{0, 1, 2, 4}}, {{0, 2, 3, 4}}, {{0, 1, 2, 5}}, {{0, 2, 3, 5}}}};

        double new_energy_max = std::numeric_limits<double>::lowest();

        for (int i = 0; i < 4; ++i) {
            if (wmtk::utils::wmtk_orient3d(
                    positions[new_tets[i][0]],
                    positions[new_tets[i][1]],
                    positions[new_tets[i][2]],
                    positions[new_tets[i][3]]) > 0) {
                auto energy = wmtk::function::utils::Tet_AMIPS_energy({{
                    positions_double[new_tets[i][0]][0],
                    positions_double[new_tets[i][0]][1],
                    positions_double[new_tets[i][0]][2],
                    positions_double[new_tets[i][1]][0],
                    positions_double[new_tets[i][1]][1],
                    positions_double[new_tets[i][1]][2],
                    positions_double[new_tets[i][2]][0],
                    positions_double[new_tets[i][2]][1],
                    positions_double[new_tets[i][2]][2],
                    positions_double[new_tets[i][3]][0],
                    positions_double[new_tets[i][3]][1],
                    positions_double[new_tets[i][3]][2],
                }});

                if (energy > new_energy_max) new_energy_max = energy;
            } else {
                auto energy = wmtk::function::utils::Tet_AMIPS_energy({{
                    positions_double[new_tets[i][1]][0],
                    positions_double[new_tets[i][1]][1],
                    positions_double[new_tets[i][1]][2],
                    positions_double[new_tets[i][0]][0],
                    positions_double[new_tets[i][0]][1],
                    positions_double[new_tets[i][0]][2],
                    positions_double[new_tets[i][2]][0],
                    positions_double[new_tets[i][2]][1],
                    positions_double[new_tets[i][2]][2],
                    positions_double[new_tets[i][3]][0],
                    positions_double[new_tets[i][3]][1],
                    positions_double[new_tets[i][3]][2],
                }});

                if (energy > new_energy_max) new_energy_max = energy;
            }
        }

        return new_energy_max;
    };

    swap44->set_value_function(swap44_energy_check);
    swap44->add_invariant(std::make_shared<wmtk::invariants::EdgeValenceInvariant>(mesh, 4));
    return swap44;
}
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
//    swap32->collapse().set_new_attribute_strategy(pt_attribute,
//    CollapseBasicStrategy::CopyOther); swap32->collapse().set_new_attribute_strategy(
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
std::shared_ptr<wmtk::operations::Operation> tet_swap(
    TetMesh& mesh,
    const IsotropicRemeshingOptions& options)
{
    // auto swap_all = std::make_shared<OrOperationSequence>(*mesh);
    // swap_all->add_operation(swap32);
    // swap_all->add_operation(swap44);
    // swap_all->add_operation(swap56);
    // swap_all->add_transfer_strategy(tag_update);
    // swap_all->add_transfer_strategy(energy_filter_update);
}
} // namespace wmtk::components::isotropic_remeshing::internal
