#include "periodic_optimization.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/components/base/get_attributes.hpp>
#include <wmtk/multimesh/consolidate.hpp>
#include <wmtk/utils/Logger.hpp>


#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>

#include <wmtk/operations/AMIPSOptimizationSmoothing.hpp>
#include <wmtk/operations/AndOperationSequence.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/MinOperationSequence.hpp>
#include <wmtk/operations/OptimizationSmoothing.hpp>
#include <wmtk/operations/OrOperationSequence.hpp>
#include <wmtk/operations/Rounding.hpp>
#include <wmtk/operations/TetWildTangentialLaplacianSmoothing.hpp>
#include <wmtk/operations/composite/ProjectOperation.hpp>
#include <wmtk/operations/composite/TetEdgeSwap.hpp>
#include <wmtk/operations/composite/TetFaceSwap.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>


#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/function/PerSimplexFunction.hpp>
#include <wmtk/function/simplex/AMIPS.hpp>

#include <wmtk/function/utils/amips.hpp>
#include <wmtk/invariants/CollapseEnergyBeforeInvariantDouble.hpp>
#include <wmtk/invariants/EdgeValenceInvariant.hpp>
#include <wmtk/invariants/EnergyFilterInvariant.hpp>
#include <wmtk/invariants/EnvelopeInvariant.hpp>
#include <wmtk/invariants/FunctionInvariant.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorSimplexInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/MaxFunctionInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/NoBoundaryCollapseToInteriorInvariant.hpp>
#include <wmtk/invariants/NoChildMeshAttachingInvariant.hpp>
#include <wmtk/invariants/RoundedInvariant.hpp>
#include <wmtk/invariants/SeparateSubstructuresInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/Swap32EnergyBeforeInvariantDouble.hpp>
#include <wmtk/invariants/Swap44EnergyBeforeInvariantDouble.hpp>
#include <wmtk/invariants/Swap56EnergyBeforeInvariantDouble.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>

#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>

#include <wmtk/utils/orient.hpp>

#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>

#include <queue>
#include <wmtk/simplex/k_ring.hpp>
#include <wmtk/simplex/link.hpp>

#include <fstream>


namespace wmtk::components::internal {

using namespace simplex;
using namespace operations;
using namespace operations::tri_mesh;
using namespace operations::tet_mesh;
using namespace operations::composite;
using namespace function;
using namespace invariants;

void write(
    const Mesh& mesh,
    const std::string& out_dir,
    const std::string& name,
    const std::string& vname,
    const int64_t index,
    const bool intermediate_output);

void adjust_sizing_field(
    Mesh& m,
    const TypedAttributeHandle<double>& coordinate_handle,
    const TypedAttributeHandle<double>& edge_length_handle,
    const TypedAttributeHandle<double>& sizing_field_scalar_handle,
    const TypedAttributeHandle<double>& energy_handle,
    const TypedAttributeHandle<double>& target_edge_length_handle,
    const TypedAttributeHandle<char>& visited_handle,
    const double stop_energy,
    const double current_max_energy,
    const double initial_target_edge_length,
    const double min_target_edge_length);

void set_operation_energy_filter(
    Mesh& m,
    const TypedAttributeHandle<double>& coordinate_handle,
    const TypedAttributeHandle<double>& energy_handle,
    const TypedAttributeHandle<char>& energy_filter_handle,
    const TypedAttributeHandle<char>& visited_handle,
    const double stop_energy,
    const double current_max_energy,
    const double initial_target_edge_length);

void periodic_optimization(
    Mesh& periodic_mesh,
    Mesh& position_mesh,
    Mesh& surface_mesh,
    const double target_edge_length,
    const double max_amips_energy,
    const int64_t passes,
    const double envelope_size,
    const bool intermediate_output,
    std::vector<attribute::MeshAttributeHandle>& pass_through_attributes,
    std::string output_dir,
    std::string output)
{
    auto position_handle =
        position_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto surface_position_handle =
        surface_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    /////////////////////////////
    // envelope
    /////////////////////////////

    auto envelope_invariant = std::make_shared<EnvelopeInvariant>(
        surface_position_handle,
        std::sqrt(2) * envelope_size,
        surface_position_handle);

    auto propagate_to_child_position =
        [](const Eigen::MatrixX<double>& P) -> Eigen::VectorX<double> { return P; };

    auto propagate_to_parent_position =
        [](const Eigen::MatrixX<double>& P) -> Eigen::VectorX<double> {
        assert(P.cols() == 1);
        return P.col(0);
    };

    auto update_parent_position = std::make_shared<SingleAttributeTransferStrategy<double, double>>(
        position_handle,
        surface_position_handle,
        propagate_to_parent_position);

    auto update_child_position = std::make_shared<SingleAttributeTransferStrategy<double, double>>(
        surface_position_handle,
        position_handle,
        propagate_to_child_position);

    /////////////////////////////
    // amips energy
    /////////////////////////////

    auto amips_handle =
        position_mesh.register_attribute<double>("amips", PrimitiveType::Tetrahedron, 1);
    auto amips_accessor = position_mesh.create_accessor(amips_handle.as<double>());

    auto compute_amips = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        // tet
        std::array<double, 12> pts;
        for (size_t i = 0; i < 4; ++i) {
            for (size_t j = 0; j < 3; ++j) {
                pts[3 * i + j] = P(j, i);
            }
        }
        const double a = Tet_AMIPS_energy(pts);
        return Eigen::VectorXd::Constant(1, a);
    };
    auto amips_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            amips_handle,
            position_handle,
            compute_amips);
    amips_update->run_on_all();

    double max_amips = std::numeric_limits<double>::lowest();
    double min_amips = std::numeric_limits<double>::max();

    for (const auto& t : position_mesh.get_all(PrimitiveType::Tetrahedron)) {
        double e = amips_accessor.scalar_attribute(t);
        max_amips = std::max(max_amips, e);
        min_amips = std::min(min_amips, e);
    }

    logger().info("Initial Max AMIPS Energy: {}, Min AMIPS Energy: {}", max_amips, min_amips);

    /////////////////////////////
    // target edge length
    /////////////////////////////

    auto target_edge_length_handle = position_mesh.register_attribute<double>(
        "target_edge_length",
        PrimitiveType::Edge,
        1,
        false,
        target_edge_length); // defaults to target edge length

    /////////////////////////////
    // edge length
    /////////////////////////////

    auto edge_length_handle =
        position_mesh.register_attribute<double>("edge_length", PrimitiveType::Edge, 1);
    auto edge_length_accessor = position_mesh.create_accessor(edge_length_handle.as<double>());
    // Edge length update
    auto compute_edge_length = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        return Eigen::Vector3d::Constant(1, sqrt((P.col(0) - P.col(1)).squaredNorm()));
    };
    auto edge_length_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            edge_length_handle,
            position_handle,
            compute_edge_length);
    edge_length_update->run_on_all();

    /////////////////////////////
    // sizing field scalar
    /////////////////////////////

    auto sizing_field_scalar_handle = position_mesh.register_attribute<double>(
        "sizing_field_scalar",
        PrimitiveType::Vertex,
        1,
        false,
        1); // defaults to 1

    //////////////////////////////////
    // sizing field update flags
    //////////////////////////////////
    auto visited_vertex_flag_handle =
        position_mesh
            .register_attribute<char>("visited_vertex", PrimitiveType::Vertex, 1, false, char(1));

    //////////////////////////////////
    // energy filter flag
    //////////////////////////////////
    auto energy_filter_handle =
        position_mesh
            .register_attribute<char>("energy_filter", PrimitiveType::Vertex, 1, false, char(1));

    auto energy_filter_accessor = position_mesh.create_accessor<char>(energy_filter_handle);

    auto update_energy_filter_func = [](const Eigen::MatrixX<double>& P) -> Eigen::VectorX<char> {
        return Eigen::VectorX<char>::Constant(1, char(1));
    };
    auto energy_filter_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<char, double>>(
            energy_filter_handle,
            position_handle,
            update_energy_filter_func);

    //////////////////////////////////
    // renew flags
    //////////////////////////////////
    auto visited_edge_flag_handle =
        position_mesh
            .register_attribute<char>("visited_edge", PrimitiveType::Edge, 1, false, char(1));

    auto update_flag_func = [](const Eigen::MatrixX<double>& P) -> Eigen::VectorX<char> {
        return Eigen::VectorX<char>::Constant(1, char(1));
    };
    auto tag_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<char, double>>(
            visited_edge_flag_handle,
            position_handle,
            update_flag_func);

    //////////////////////////////////
    // pass through
    //////////////////////////////////
    pass_through_attributes.push_back(edge_length_handle);
    pass_through_attributes.push_back(amips_handle);
    pass_through_attributes.push_back(visited_vertex_flag_handle);
    pass_through_attributes.push_back(energy_filter_handle);

    //////////////////////////////////
    // invariants
    //////////////////////////////////

    auto inversion_invariant = std::make_shared<SimplexInversionInvariant<double>>(
        position_mesh,
        position_handle.as<double>());

    std::shared_ptr<function::PerSimplexFunction> amips =
        std::make_shared<AMIPS>(position_mesh, position_handle);

    auto function_invariant =
        std::make_shared<MaxFunctionInvariant>(position_mesh.top_simplex_type(), amips);

    auto link_condition = std::make_shared<MultiMeshLinkConditionInvariant>(position_mesh);

    auto todo_larger = std::make_shared<TodoLargerInvariant>(
        position_mesh,
        edge_length_handle.as<double>(),
        target_edge_length_handle.as<double>(),
        4.0 / 3.0);

    auto todo_smaller = std::make_shared<TodoSmallerInvariant>(
        position_mesh,
        edge_length_handle.as<double>(),
        target_edge_length_handle.as<double>(),
        4.0 / 5.0);

    auto valence_3 = std::make_shared<EdgeValenceInvariant>(position_mesh, 3);
    auto valence_4 = std::make_shared<EdgeValenceInvariant>(position_mesh, 4);
    auto valence_5 = std::make_shared<EdgeValenceInvariant>(position_mesh, 5);

    auto invariant_separate_substructures =
        std::make_shared<invariants::SeparateSubstructuresInvariant>(position_mesh);

    //////////////////////////////////
    // operations
    //////////////////////////////////

    std::vector<std::shared_ptr<Operation>> ops;
    std::vector<std::string> ops_name;

    auto long_edges_first = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return -edge_length_accessor.scalar_attribute(s.tuple());
    };
    auto short_edges_first = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return edge_length_accessor.scalar_attribute(s.tuple());
    };

    //////////////////////////////////
    // 1) EdgeSplit
    //////////////////////////////////
    auto split = std::make_shared<EdgeSplit>(position_mesh);
    split->set_priority(long_edges_first);

    split->add_invariant(todo_larger);
    split->add_invariant(
        std::make_shared<EnergyFilterInvariant>(position_mesh, energy_filter_handle.as<char>()));
    split->add_invariant(inversion_invariant);

    split->set_new_attribute_strategy(position_handle);
    split->set_new_attribute_strategy(sizing_field_scalar_handle);
    split->set_new_attribute_strategy(
        visited_edge_flag_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    split->set_new_attribute_strategy(
        target_edge_length_handle,
        wmtk::operations::SplitBasicStrategy::Copy,
        wmtk::operations::SplitRibBasicStrategy::Mean);

    for (const auto& attr : pass_through_attributes) {
        split->set_new_attribute_strategy(
            attr,
            wmtk::operations::SplitBasicStrategy::None,
            wmtk::operations::SplitRibBasicStrategy::None);
    }

    split->add_transfer_strategy(amips_update);
    split->add_transfer_strategy(edge_length_update);
    split->add_transfer_strategy(tag_update); // for renew the queue
    split->add_transfer_strategy(energy_filter_update);

    ops.emplace_back(split);
    ops_name.emplace_back("SPLIT");

    //////////////////////////////////
    // 1) EdgeCollapse
    //////////////////////////////////

    auto clps_strat1 = std::make_shared<CollapseNewAttributeStrategy<double>>(position_handle);
    clps_strat1->set_strategy(CollapseBasicStrategy::CopyOther);

    auto clps_strat2 = std::make_shared<CollapseNewAttributeStrategy<double>>(position_handle);
    clps_strat2->set_strategy(CollapseBasicStrategy::CopyTuple);

    auto setup_collapse = [&](std::shared_ptr<EdgeCollapse>& collapse) {
        collapse->add_invariant(invariant_separate_substructures);
        collapse->add_invariant(link_condition);
        collapse->add_invariant(inversion_invariant);
        collapse->add_invariant(envelope_invariant);

        collapse->set_new_attribute_strategy(
            visited_edge_flag_handle,
            wmtk::operations::CollapseBasicStrategy::None);

        collapse->add_transfer_strategy(tag_update);
        collapse->add_transfer_strategy(energy_filter_update);
        for (const auto& attr : pass_through_attributes) {
            collapse->set_new_attribute_strategy(
                attr,
                wmtk::operations::CollapseBasicStrategy::None);
        }
        collapse->set_new_attribute_strategy(
            target_edge_length_handle,
            wmtk::operations::CollapseBasicStrategy::None);

        collapse->add_transfer_strategy(amips_update);
        collapse->add_transfer_strategy(edge_length_update);

        collapse->add_transfer_strategy(update_child_position);
    };

    auto collapse1 = std::make_shared<EdgeCollapse>(position_mesh);
    collapse1->add_invariant(std::make_shared<CollapseEnergyBeforeInvariantDouble>(
        position_mesh,
        position_handle.as<double>(),
        amips_handle.as<double>(),
        1));

    collapse1->set_new_attribute_strategy(position_handle, clps_strat1);
    collapse1->set_new_attribute_strategy(sizing_field_scalar_handle, clps_strat1);
    setup_collapse(collapse1);

    auto collapse2 = std::make_shared<EdgeCollapse>(position_mesh);
    collapse2->add_invariant(std::make_shared<CollapseEnergyBeforeInvariantDouble>(
        position_mesh,
        position_handle.as<double>(),
        amips_handle.as<double>(),
        0));

    collapse2->set_new_attribute_strategy(position_handle, clps_strat2);
    collapse2->set_new_attribute_strategy(sizing_field_scalar_handle, clps_strat2);
    setup_collapse(collapse2);

    auto collapse = std::make_shared<OrOperationSequence>(position_mesh);
    collapse->add_operation(collapse1);
    collapse->add_operation(collapse2);
    collapse->set_priority(short_edges_first);
    collapse->add_invariant(
        std::make_shared<EnergyFilterInvariant>(position_mesh, energy_filter_handle.as<char>()));
    collapse->add_invariant(todo_smaller);

    collapse->add_transfer_strategy(update_child_position);

    ops.emplace_back(collapse);
    ops_name.emplace_back("COLLAPSE");

    //////////////////////////////////
    // 1) EdgeSwap
    //////////////////////////////////

    // swap56

    auto swap56 = std::make_shared<MinOperationSequence>(position_mesh);
    for (int i = 0; i < 5; ++i) {
        auto swap = std::make_shared<TetEdgeSwap>(position_mesh, i);
        swap->collapse().add_invariant(invariant_separate_substructures);
        swap->collapse().add_invariant(link_condition);
        swap->collapse().set_new_attribute_strategy(
            position_handle,
            CollapseBasicStrategy::CopyOther);
        swap->collapse().set_new_attribute_strategy(
            sizing_field_scalar_handle,
            CollapseBasicStrategy::CopyOther);
        swap->split().set_new_attribute_strategy(position_handle);
        swap->split().set_new_attribute_strategy(sizing_field_scalar_handle);

        swap->split().set_new_attribute_strategy(
            visited_edge_flag_handle,
            wmtk::operations::SplitBasicStrategy::None,
            wmtk::operations::SplitRibBasicStrategy::None);
        swap->collapse().set_new_attribute_strategy(
            visited_edge_flag_handle,
            wmtk::operations::CollapseBasicStrategy::None);

        swap->split().set_new_attribute_strategy(
            target_edge_length_handle,
            wmtk::operations::SplitBasicStrategy::Copy,
            wmtk::operations::SplitRibBasicStrategy::Mean);
        swap->collapse().set_new_attribute_strategy(
            target_edge_length_handle,
            wmtk::operations::CollapseBasicStrategy::None);

        swap->add_invariant(std::make_shared<Swap56EnergyBeforeInvariantDouble>(
            position_mesh,
            position_handle.as<double>(),
            i));

        swap->add_transfer_strategy(amips_update);

        swap->collapse().add_invariant(inversion_invariant);

        swap->collapse().add_invariant(envelope_invariant);

        for (const auto& attr : pass_through_attributes) {
            swap->split().set_new_attribute_strategy(
                attr,
                wmtk::operations::SplitBasicStrategy::None,
                wmtk::operations::SplitRibBasicStrategy::None);
            swap->collapse().set_new_attribute_strategy(
                attr,
                wmtk::operations::CollapseBasicStrategy::None);
        }

        swap56->add_operation(swap);
    }

    auto swap56_energy_check = [&](int64_t idx, const simplex::Simplex& t) -> double {
        constexpr static PrimitiveType PV = PrimitiveType::Vertex;
        constexpr static PrimitiveType PE = PrimitiveType::Edge;
        constexpr static PrimitiveType PF = PrimitiveType::Triangle;
        constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;

        auto accessor = position_mesh.create_const_accessor(position_handle.as<double>());

        const Tuple e0 = t.tuple();
        const Tuple e1 = position_mesh.switch_tuple(e0, PV);

        std::array<Tuple, 5> v;
        auto iter_tuple = e0;
        for (int64_t i = 0; i < 5; ++i) {
            v[i] = position_mesh.switch_tuples(iter_tuple, {PE, PV});
            iter_tuple = position_mesh.switch_tuples(iter_tuple, {PF, PT});
        }
        if (iter_tuple != e0) return 0;
        assert(iter_tuple == e0);

        // five iterable vertices remap to 0-4 by m_collapse_index, 0: m_collapse_index, 5:
        // e0, 6: e1
        std::array<Eigen::Vector3<double>, 7> positions = {
            {accessor.const_vector_attribute(v[(idx + 0) % 5]),
             accessor.const_vector_attribute(v[(idx + 1) % 5]),
             accessor.const_vector_attribute(v[(idx + 2) % 5]),
             accessor.const_vector_attribute(v[(idx + 3) % 5]),
             accessor.const_vector_attribute(v[(idx + 4) % 5]),
             accessor.const_vector_attribute(e0),
             accessor.const_vector_attribute(e1)}};

        std::array<Eigen::Vector3d, 7> positions_double = {
            {positions[0],
             positions[1],
             positions[2],
             positions[3],
             positions[4],
             positions[5],
             positions[6]}};

        std::array<std::array<int, 4>, 6> new_tets = {
            {{{0, 1, 2, 5}},
             {{0, 2, 3, 5}},
             {{0, 3, 4, 5}},
             {{0, 1, 2, 6}},
             {{0, 2, 3, 6}},
             {{0, 3, 4, 6}}}};

        double new_energy_max = std::numeric_limits<double>::lowest();

        for (int i = 0; i < 6; ++i) {
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

    swap56->set_value_function(swap56_energy_check);
    swap56->add_invariant(std::make_shared<EdgeValenceInvariant>(position_mesh, 5));


    // swap44
    auto swap44 = std::make_shared<MinOperationSequence>(position_mesh);
    for (int i = 0; i < 2; ++i) {
        auto swap = std::make_shared<TetEdgeSwap>(position_mesh, i);
        swap->collapse().add_invariant(invariant_separate_substructures);
        swap->collapse().add_invariant(link_condition);
        swap->collapse().set_new_attribute_strategy(
            position_handle,
            CollapseBasicStrategy::CopyOther);
        swap->collapse().set_new_attribute_strategy(
            sizing_field_scalar_handle,
            CollapseBasicStrategy::CopyOther);
        swap->split().set_new_attribute_strategy(position_handle);
        swap->split().set_new_attribute_strategy(sizing_field_scalar_handle);
        swap->split().set_new_attribute_strategy(
            visited_edge_flag_handle,
            wmtk::operations::SplitBasicStrategy::None,
            wmtk::operations::SplitRibBasicStrategy::None);
        swap->collapse().set_new_attribute_strategy(
            visited_edge_flag_handle,
            wmtk::operations::CollapseBasicStrategy::None);

        // swap->split().add_transfer_strategy(amips_update);
        // swap->collapse().add_transfer_strategy(amips_update);

        swap->split().set_new_attribute_strategy(
            target_edge_length_handle,
            wmtk::operations::SplitBasicStrategy::Copy,
            wmtk::operations::SplitRibBasicStrategy::Mean);
        swap->collapse().set_new_attribute_strategy(
            target_edge_length_handle,
            wmtk::operations::CollapseBasicStrategy::None);

        swap->add_transfer_strategy(amips_update);

        swap->add_invariant(std::make_shared<Swap44EnergyBeforeInvariantDouble>(
            position_mesh,
            position_handle.as<double>(),
            i));

        // swap->add_invariant(inversion_invariant);
        swap->collapse().add_invariant(inversion_invariant);

        swap->collapse().add_invariant(envelope_invariant);

        for (const auto& attr : pass_through_attributes) {
            swap->split().set_new_attribute_strategy(
                attr,
                wmtk::operations::SplitBasicStrategy::None,
                wmtk::operations::SplitRibBasicStrategy::None);
            swap->collapse().set_new_attribute_strategy(
                attr,
                wmtk::operations::CollapseBasicStrategy::None);
        }

        swap44->add_operation(swap);
    }

    auto swap44_energy_check = [&](int64_t idx, const simplex::Simplex& t) -> double {
        constexpr static PrimitiveType PV = PrimitiveType::Vertex;
        constexpr static PrimitiveType PE = PrimitiveType::Edge;
        constexpr static PrimitiveType PF = PrimitiveType::Triangle;
        constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;

        auto accessor = position_mesh.create_const_accessor(position_handle.as<double>());

        // get the coords of the vertices
        // input edge end points
        const Tuple e0 = t.tuple();
        const Tuple e1 = position_mesh.switch_tuple(e0, PV);
        // other four vertices
        std::array<Tuple, 4> v;
        auto iter_tuple = e0;
        for (int64_t i = 0; i < 4; ++i) {
            v[i] = position_mesh.switch_tuples(iter_tuple, {PE, PV});
            iter_tuple = position_mesh.switch_tuples(iter_tuple, {PF, PT});
        }

        if (iter_tuple != e0) return 0;
        assert(iter_tuple == e0);

        std::array<Eigen::Vector3<double>, 6> positions = {
            {accessor.const_vector_attribute(v[(idx + 0) % 4]),
             accessor.const_vector_attribute(v[(idx + 1) % 4]),
             accessor.const_vector_attribute(v[(idx + 2) % 4]),
             accessor.const_vector_attribute(v[(idx + 3) % 4]),
             accessor.const_vector_attribute(e0),
             accessor.const_vector_attribute(e1)}};
        std::array<Eigen::Vector3d, 6> positions_double = {
            {positions[0], positions[1], positions[2], positions[3], positions[4], positions[5]}};

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
    swap44->add_invariant(std::make_shared<EdgeValenceInvariant>(position_mesh, 4));

    // swap 32
    auto swap32 = std::make_shared<TetEdgeSwap>(position_mesh, 0);
    swap32->add_invariant(std::make_shared<EdgeValenceInvariant>(position_mesh, 3));
    swap32->add_invariant(std::make_shared<Swap32EnergyBeforeInvariantDouble>(
        position_mesh,
        position_handle.as<double>()));

    swap32->collapse().add_invariant(invariant_separate_substructures);
    swap32->collapse().add_invariant(link_condition);
    swap32->collapse().set_new_attribute_strategy(
        position_handle,
        CollapseBasicStrategy::CopyOther);
    swap32->collapse().set_new_attribute_strategy(
        sizing_field_scalar_handle,
        CollapseBasicStrategy::CopyOther);
    // swap32->add_invariant(inversion_invariant);
    swap32->split().set_new_attribute_strategy(position_handle);
    swap32->split().set_new_attribute_strategy(sizing_field_scalar_handle);
    swap32->split().set_new_attribute_strategy(
        visited_edge_flag_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    swap32->collapse().set_new_attribute_strategy(
        visited_edge_flag_handle,
        wmtk::operations::CollapseBasicStrategy::None);

    swap32->split().set_new_attribute_strategy(
        target_edge_length_handle,
        wmtk::operations::SplitBasicStrategy::Copy,
        wmtk::operations::SplitRibBasicStrategy::Mean);
    swap32->collapse().set_new_attribute_strategy(
        target_edge_length_handle,
        wmtk::operations::CollapseBasicStrategy::None);

    swap32->add_transfer_strategy(amips_update);

    // hack
    swap32->collapse().add_invariant(inversion_invariant);
    swap32->collapse().add_invariant(envelope_invariant);

    for (const auto& attr : pass_through_attributes) {
        swap32->split().set_new_attribute_strategy(
            attr,
            wmtk::operations::SplitBasicStrategy::None,
            wmtk::operations::SplitRibBasicStrategy::None);
        swap32->collapse().set_new_attribute_strategy(
            attr,
            wmtk::operations::CollapseBasicStrategy::None);
    }

    auto swap_all = std::make_shared<OrOperationSequence>(position_mesh);
    swap_all->add_operation(swap32);
    swap_all->add_operation(swap44);
    swap_all->add_operation(swap56);
    swap_all->add_transfer_strategy(tag_update);
    swap_all->add_transfer_strategy(energy_filter_update);
    swap_all->add_invariant(
        std::make_shared<EnergyFilterInvariant>(position_mesh, energy_filter_handle.as<char>()));
    swap_all->add_invariant(std::make_shared<InteriorEdgeInvariant>(position_mesh));
    swap_all->add_invariant(std::make_shared<NoChildMeshAttachingInvariant>(position_mesh));

    swap_all->set_priority(long_edges_first);

    swap_all->add_transfer_strategy(update_child_position);

    ops.push_back(swap_all);
    ops_name.push_back("EDGE SWAP");

    //////////////////////////////////
    // 4) Smoothing
    //////////////////////////////////

    using MeshConstrainPair = ProjectOperation::MeshConstrainPair;
    std::vector<MeshConstrainPair> mesh_constraint_pairs;
    mesh_constraint_pairs.emplace_back(surface_position_handle, surface_position_handle);

    auto smoothing = std::make_shared<AMIPSOptimizationSmoothing>(position_mesh, position_handle);
    smoothing->add_invariant(inversion_invariant);
    smoothing->add_transfer_strategy(update_child_position);
    auto proj_smoothing = std::make_shared<ProjectOperation>(smoothing, mesh_constraint_pairs);
    proj_smoothing->use_random_priority() = true;

    proj_smoothing->add_invariant(envelope_invariant);
    proj_smoothing->add_invariant(inversion_invariant);
    proj_smoothing->add_invariant(
        std::make_shared<EnergyFilterInvariant>(position_mesh, energy_filter_handle.as<char>()));

    proj_smoothing->add_transfer_strategy(amips_update);
    proj_smoothing->add_transfer_strategy(edge_length_update);
    proj_smoothing->add_transfer_strategy(update_parent_position);

    proj_smoothing->add_transfer_strategy(update_child_position);

    //////////////////////////////////
    // Scheduler
    //////////////////////////////////
    Scheduler scheduler;

    int success = 0;

    double old_max_energy = max_amips;
    double old_avg_energy = 0;

    for (int64_t i = 0; i < passes; ++i) {
        logger().info("--------------------------- Pass {} ---------------------------", i);

        SchedulerStats pass_stats;
        int jj = 0;

        for (auto& op : ops) {
            SchedulerStats stats;
            if (op->primitive_type() == PrimitiveType::Edge) {
                stats = scheduler.run_operation_on_all(*op, visited_edge_flag_handle.as<char>());
            } else {
                stats = scheduler.run_operation_on_all(*op);
            }
            pass_stats += stats;
            logger().info(
                "Executed {}, {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, "
                "executing: {}",
                ops_name[jj],
                stats.number_of_performed_operations(),
                stats.number_of_successful_operations(),
                stats.number_of_failed_operations(),
                stats.collecting_time,
                stats.sorting_time,
                stats.executing_time);

            success = stats.number_of_successful_operations();

            // compute energy
            double avg_energy = 0;
            double max_energy = std::numeric_limits<double>::lowest();
            double min_energy = std::numeric_limits<double>::max();
            for (const auto& t : position_mesh.get_all(position_mesh.top_simplex_type())) {
                double e = amips_accessor.scalar_attribute(t);
                max_energy = std::max(max_energy, e);
                min_energy = std::min(min_energy, e);
                avg_energy += e;
            }

            avg_energy =
                avg_energy / position_mesh.get_all(position_mesh.top_simplex_type()).size();

            logger().info(
                "Max AMIPS Energy: {}, Min AMIPS Energy: {}, Avg AMIPS Energy: {}",
                max_energy,
                min_energy,
                avg_energy);

            ++jj;
        }

        logger().info(
            "Executed {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, executing: {}",
            pass_stats.number_of_performed_operations(),
            pass_stats.number_of_successful_operations(),
            pass_stats.number_of_failed_operations(),
            pass_stats.collecting_time,
            pass_stats.sorting_time,
            pass_stats.executing_time);

        multimesh::consolidate(periodic_mesh);

        write(position_mesh, output_dir, output, "vertices", i + 1, intermediate_output);

        double max_energy = std::numeric_limits<double>::lowest();
        double min_energy = std::numeric_limits<double>::max();
        double avg_energy = 0;
        for (const auto& t : position_mesh.get_all(position_mesh.top_simplex_type())) {
            double e = amips_accessor.scalar_attribute(t);
            max_energy = std::max(max_energy, e);
            min_energy = std::min(min_energy, e);
            avg_energy += e;
        }

        avg_energy = avg_energy / position_mesh.get_all(position_mesh.top_simplex_type()).size();

        logger().info(
            "Max AMIPS Energy: {}, Min AMIPS Energy: {}, Avg AMIPS Energy: {}",
            max_energy,
            min_energy,
            avg_energy);

        if (i > 0 && old_max_energy - max_energy < 5e-1 &&
            (old_avg_energy - avg_energy) / avg_energy < 0.1) {
            wmtk::logger().info("adjusting sizing field ...");

            adjust_sizing_field(
                position_mesh,
                position_handle.as<double>(),
                edge_length_handle.as<double>(),
                sizing_field_scalar_handle.as<double>(),
                amips_handle.as<double>(),
                target_edge_length_handle.as<double>(),
                visited_vertex_flag_handle.as<char>(),
                max_amips_energy,
                max_energy,
                target_edge_length,
                0.00001);

            wmtk::logger().info("adjusting sizing field finished");
            for (const auto& v : position_mesh.get_all(PrimitiveType::Vertex)) {
                energy_filter_accessor.scalar_attribute(v) = char(1);
            }
            wmtk::logger().info("reset energy filter");
        } else {
            wmtk::logger().info("setting energy filter ...");
            set_operation_energy_filter(
                position_mesh,
                position_handle.as<double>(),
                amips_handle.as<double>(),
                energy_filter_handle.as<char>(),
                visited_vertex_flag_handle.as<char>(),
                max_amips_energy,
                max_energy,
                target_edge_length);
            wmtk::logger().info("setting energy filter finished");
        }

        old_max_energy = max_energy;
        old_avg_energy = avg_energy;

        // stop at good quality
        if (max_energy <= max_amips_energy) break;
    }
}

void write(
    const Mesh& mesh,
    const std::string& out_dir,
    const std::string& name,
    const std::string& vname,
    const int64_t index,
    const bool intermediate_output)
{
    if (intermediate_output) {
        // write tetmesh
        const std::filesystem::path data_dir = "";
        wmtk::io::ParaviewWriter writer(
            data_dir / (name + "_" + std::to_string(index)),
            "vertices",
            mesh,
            true,
            true,
            true,
            true);
        mesh.serialize(writer);
    }

} // namespace

void adjust_sizing_field(
    Mesh& m,
    const TypedAttributeHandle<double>& coordinate_handle,
    const TypedAttributeHandle<double>& edge_length_handle,
    const TypedAttributeHandle<double>& sizing_field_scalar_handle,
    const TypedAttributeHandle<double>& energy_handle,
    const TypedAttributeHandle<double>& target_edge_length_handle,
    const TypedAttributeHandle<char>& visited_handle,
    const double stop_energy,
    const double current_max_energy,
    const double initial_target_edge_length,
    const double min_target_edge_length)
{
    if (m.top_simplex_type() != PrimitiveType::Tetrahedron) return;

    std::cout << "in here" << std::endl;

    const auto coordinate_accessor = m.create_const_accessor<double>(coordinate_handle);
    const auto edge_length_accessor = m.create_const_accessor<double>(edge_length_handle);
    const auto energy_accessor = m.create_const_accessor<double>(energy_handle);

    auto sizing_field_scalar_accessor = m.create_accessor<double>(sizing_field_scalar_handle);
    auto target_edge_length_accessor = m.create_accessor<double>(target_edge_length_handle);
    auto visited_accessor = m.create_accessor<char>(visited_handle);

    const double stop_filter_energy = stop_energy * 0.8;
    double filter_energy = std::max(current_max_energy / 100, stop_filter_energy);
    filter_energy = std::min(filter_energy, 100.0);

    const double recover_scalar = 1.5;
    const double refine_scalar = 0.5;
    const double min_refine_scalar = min_target_edge_length / initial_target_edge_length;

    auto vertices_all = m.get_all(PrimitiveType::Vertex);
    int64_t v_cnt = vertices_all.size();

    std::vector<Vector3d> centroids;
    std::queue<Tuple> v_queue;
    // std::vector<double> scale_multipliers(v_cnt, recover_scalar);

    // get centroids and initial v_queue
    for (const auto& t : m.get_all(PrimitiveType::Tetrahedron)) {
        // if (std::cbrt(energy_accessor.const_scalar_attribute(t)) < filter_energy) {
        if (energy_accessor.const_scalar_attribute(t) < filter_energy) {
            // skip good tets
            continue;
        }
        auto vertices = m.orient_vertices(t);
        Vector3d c(0, 0, 0);
        for (int i = 0; i < 4; ++i) {
            c += coordinate_accessor.const_vector_attribute(vertices[i]);
            v_queue.emplace(vertices[i]);
        }
        centroids.emplace_back(c / 4.0);
    }

    wmtk::logger().info(
        "filter energy: {}, low quality tets num: {}",
        filter_energy,
        centroids.size());

    const double R = initial_target_edge_length * 1.8; // update field radius

    for (const auto& v : vertices_all) { // reset visited flag
        visited_accessor.scalar_attribute(v) = char(0);
    }

    // TODO: use efficient data structure
    auto get_nearest_dist = [&](const Tuple& v) -> double {
        Tuple nearest_tuple;
        double min_dist = std::numeric_limits<double>::max();
        const Vector3d v_pos = coordinate_accessor.const_vector_attribute(v);
        for (const auto& c_pos : centroids) {
            double dist = (c_pos - v_pos).norm();
            min_dist = std::min(min_dist, dist);
        }
        return min_dist;
    };

    while (!v_queue.empty()) {
        auto v = v_queue.front();
        v_queue.pop();

        if (visited_accessor.scalar_attribute(v) == char(1)) continue;
        visited_accessor.scalar_attribute(v) = char(1);

        double dist = std::max(0., get_nearest_dist(v));

        if (dist > R) {
            visited_accessor.scalar_attribute(v) = char(0);
            continue;
        }

        double scale_multiplier = std::min(
            recover_scalar,
            dist / R * (1 - refine_scalar) + refine_scalar); // linear interpolate

        auto new_scale = sizing_field_scalar_accessor.scalar_attribute(v) * scale_multiplier;
        if (new_scale > 1) {
            sizing_field_scalar_accessor.scalar_attribute(v) = 1;
        } else if (new_scale < min_refine_scalar) {
            sizing_field_scalar_accessor.scalar_attribute(v) = min_refine_scalar;
        } else {
            sizing_field_scalar_accessor.scalar_attribute(v) = new_scale;
        }

        // push one ring vertices into the queue
        for (const auto& v_one_ring : simplex::link(m, simplex::Simplex::vertex(m, v))
                                          .simplex_vector(PrimitiveType::Vertex)) {
            if (visited_accessor.scalar_attribute(v_one_ring) == char(1)) continue;
            v_queue.push(v_one_ring.tuple());
        }
    }

    // update the rest
    for (const auto& v : vertices_all) {
        if (visited_accessor.scalar_attribute(v) = char(1)) continue;
        auto new_scale = sizing_field_scalar_accessor.scalar_attribute(v) * 1.5;
        if (new_scale > 1) {
            sizing_field_scalar_accessor.scalar_attribute(v) = 1;
        } else if (new_scale < min_refine_scalar) {
            sizing_field_scalar_accessor.scalar_attribute(v) = min_refine_scalar;
        } else {
            sizing_field_scalar_accessor.scalar_attribute(v) = new_scale;
        }
    }

    // update target edge length
    for (const auto& e : m.get_all(PrimitiveType::Edge)) {
        target_edge_length_accessor.scalar_attribute(e) =
            initial_target_edge_length *
            (sizing_field_scalar_accessor.scalar_attribute(e) +
             sizing_field_scalar_accessor.scalar_attribute(
                 m.switch_tuple(e, PrimitiveType::Vertex))) /
            2;
    }
}

void set_operation_energy_filter(
    Mesh& m,
    const TypedAttributeHandle<double>& coordinate_handle,
    const TypedAttributeHandle<double>& energy_handle,
    const TypedAttributeHandle<char>& energy_filter_handle,
    const TypedAttributeHandle<char>& visited_handle,
    const double stop_energy,
    const double current_max_energy,
    const double initial_target_edge_length)
{
    // two ring version
    if (m.top_simplex_type() != PrimitiveType::Tetrahedron) return;

    const auto coordinate_accessor = m.create_const_accessor<double>(coordinate_handle);
    const auto energy_accessor = m.create_const_accessor<double>(energy_handle);

    auto energy_filter_accessor = m.create_accessor<char>(energy_filter_handle);
    auto visited_accessor = m.create_accessor<char>(visited_handle);

    const double stop_filter_energy = stop_energy * 0.8;
    double filter_energy = std::max(current_max_energy / 100, stop_filter_energy);
    filter_energy = std::min(filter_energy, 100.0);

    auto vertices_all = m.get_all(PrimitiveType::Vertex);

    for (const auto& v : vertices_all) { // reset visited flag
        visited_accessor.scalar_attribute(v) = char(0);
        energy_filter_accessor.scalar_attribute(v) = char(0);
    }

    // get centroids and initial v_queue
    for (const auto& t : m.get_all(PrimitiveType::Tetrahedron)) {
        if (energy_accessor.const_scalar_attribute(t) < filter_energy) {
            // skip good tets
            continue;
        }
        auto vertices = m.orient_vertices(t);
        for (const auto& v : vertices) {
            energy_filter_accessor.scalar_attribute(v) = char(1);
            for (const auto& vv : simplex::k_ring(m, simplex::Simplex::vertex(m, v), 1)
                                      .simplex_vector(PrimitiveType::Vertex)) {
                energy_filter_accessor.scalar_attribute(vv) = char(1);
            }
        }
    }
}


} // namespace wmtk::components::internal