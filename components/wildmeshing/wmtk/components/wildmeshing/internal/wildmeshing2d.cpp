#include "wildmeshing2d.hpp"

#include "WildmeshingOptions.hpp"
#include "wildmeshing_utils.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/cast_attribute.hpp>

#include <wmtk/components/utils/get_attributes.hpp>
#include <wmtk/multimesh/consolidate.hpp>
#include <wmtk/utils/Logger.hpp>


#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/operations/attribute_update/make_cast_attribute_transfer_strategy.hpp>

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
#include <wmtk/invariants/CollapseEnergyBeforeInvariant.hpp>
#include <wmtk/invariants/CollapseSoftEnergyBeforeInvariant.hpp>
#include <wmtk/invariants/EdgeValenceInvariant.hpp>
#include <wmtk/invariants/EnergyFilterInvariant.hpp>
#include <wmtk/invariants/EnvelopeInvariant.hpp>
#include <wmtk/invariants/FrozenVertexInvariant.hpp>
#include <wmtk/invariants/FunctionInvariant.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorSimplexInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/MaxFunctionInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/MultiMeshMapValidInvariant.hpp>
#include <wmtk/invariants/NoBoundaryCollapseToInteriorInvariant.hpp>
#include <wmtk/invariants/NoChildMeshAttachingInvariant.hpp>
#include <wmtk/invariants/RoundedInvariant.hpp>
#include <wmtk/invariants/SeparateSubstructuresInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/Swap2dUnroundedVertexInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>

#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>

#include <wmtk/utils/Rational.hpp>
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

std::vector<std::pair<std::shared_ptr<Mesh>, std::string>> wildmeshing2d(
    const WildMeshingOptions& options)
{
    auto mesh = options.input_mesh;

    if (!mesh->is_connectivity_valid()) {
        throw std::runtime_error("input mesh for wildmeshing connectivity invalid");
    }

    wmtk::logger().trace("Getting rational point handle");

    //////////////////////////////////
    // Retriving vertices
    //
    if (options.replace_double_coordinate) {
        wmtk::logger().trace("Found double attribugte");
        auto pt_double_attribute =
            mesh->get_attribute_handle<double>(options.input_mesh_position, PrimitiveType::Vertex);

        if (!mesh->has_attribute<Rational>(options.input_mesh_position, PrimitiveType::Vertex)) {
            wmtk::utils::cast_attribute<wmtk::Rational>(
                pt_double_attribute,
                *mesh,
                options.input_mesh_position);


        } else {
            auto pt_attribute = mesh->get_attribute_handle<Rational>(
                options.input_mesh_position,
                PrimitiveType::Vertex);
            wmtk::utils::cast_attribute<wmtk::Rational>(pt_double_attribute, pt_attribute);
        }
        mesh->delete_attribute(pt_double_attribute);
    }
    auto pt_attribute =
        mesh->get_attribute_handle<Rational>(options.input_mesh_position, PrimitiveType::Vertex);
    wmtk::logger().trace("Getting rational point accessor");
    auto pt_accessor = mesh->create_accessor(pt_attribute.as<Rational>());

    wmtk::logger().trace("Computing bounding box diagonal");
    //////////////////////////////////
    // computing bbox diagonal
    Eigen::VectorXd bmin(mesh->top_cell_dimension());
    bmin.setConstant(std::numeric_limits<double>::max());
    Eigen::VectorXd bmax(mesh->top_cell_dimension());
    bmax.setConstant(std::numeric_limits<double>::lowest());

    const auto vertices = mesh->get_all(PrimitiveType::Vertex);
    for (const auto& v : vertices) {
        const auto p = pt_accessor.vector_attribute(v).cast<double>();
        for (int64_t d = 0; d < bmax.size(); ++d) {
            bmin[d] = std::min(bmin[d], p[d]);
            bmax[d] = std::max(bmax[d], p[d]);
        }
    }


    const double bbdiag = (bmax - bmin).norm();

    wmtk::logger().info("bbox max {}, bbox min {}, diag {}", bmax, bmin, bbdiag);

    const double target_edge_length = options.target_edge_length * bbdiag;

    // const double target_edge_length =
    //     options.target_edge_length * bbdiag /
    //     std::min(
    //         2.0,
    //         1 + options.target_edge_length * 2); // min to prevent bad option.target_edge_length

    wmtk::logger().info("target edge length: {}", target_edge_length);

    //////////////////////////////////
    // store amips
    auto amips_attribute =
        mesh->register_attribute<double>("wildmeshing_amips", mesh->top_simplex_type(), 1);
    auto amips_accessor = mesh->create_accessor(amips_attribute.as<double>());
    // amips update
    auto compute_amips = [](const Eigen::MatrixX<Rational>& P) -> Eigen::VectorXd {
        assert(P.rows() == 2 || P.rows() == 3); // rows --> attribute dimension
        assert(P.cols() == P.rows() + 1);
        if (P.cols() == 3) {
            // triangle
            assert(P.rows() == 2);
            std::array<double, 6> pts;
            for (size_t i = 0; i < 3; ++i) {
                for (size_t j = 0; j < 2; ++j) {
                    pts[2 * i + j] = P(j, i).to_double();
                }
            }
            const double a = Tri_AMIPS_energy(pts);
            return Eigen::VectorXd::Constant(1, a);
        } else {
            // tet
            assert(P.rows() == 3);
            std::array<double, 12> pts;
            for (size_t i = 0; i < 4; ++i) {
                for (size_t j = 0; j < 3; ++j) {
                    pts[3 * i + j] = P(j, i).to_double();
                }
            }
            const double a = Tet_AMIPS_energy(pts);
            return Eigen::VectorXd::Constant(1, a);
        }
    };
    auto amips_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, Rational>>(
            amips_attribute,
            pt_attribute,
            compute_amips);
    amips_update->run_on_all();

    double max_amips = std::numeric_limits<double>::lowest();
    double min_amips = std::numeric_limits<double>::max();

    for (const auto& t : mesh->get_all(mesh->top_simplex_type())) {
        // double e = amips->get_value(simplex::Simplex(mesh->top_simplex_type(), t));
        double e = amips_accessor.scalar_attribute(t);
        max_amips = std::max(max_amips, e);
        min_amips = std::min(min_amips, e);
    }

    logger().info("Initial Max AMIPS Energy: {}, Min AMIPS Energy: {}", max_amips, min_amips);

    //////////////////////////////////
    // sizing field scalar
    //////////////////////////////////

    // TODO: add sizing field for 2d

    // auto sizing_field_scalar_attribute = mesh->register_attribute<double>(
    //     "sizing_field_scalar",
    //     PrimitiveType::Vertex,
    //     1,
    //     false,
    //     1); // defaults to 1


    //////////////////////////////////
    // Storing target edge length
    auto target_edge_length_attribute = mesh->register_attribute<double>(
        "wildmeshing_target_edge_length",
        PrimitiveType::Edge,
        1,
        false,
        target_edge_length); // defaults to target edge length

    // Target edge length update
    const double min_edge_length = [&]() -> double {
        if (options.envelopes.empty()) {
            return 1e-6; // some default value if no envelope exists
        } else {
            // use envelope thickness if available
            double r = 0;
            for (const auto& e : options.envelopes) {
                r = std::max(r, e.thickness);
            }
            assert(r > 0);
            return r;
        }
    }();
    const double target_max_amips = options.target_max_amips;


    //////////////////////////////////
    // Storing edge lengths
    auto edge_length_attribute =
        mesh->register_attribute<double>("edge_length", PrimitiveType::Edge, 1);
    auto edge_length_accessor = mesh->create_accessor(edge_length_attribute.as<double>());
    // Edge length update
    auto compute_edge_length = [](const Eigen::MatrixX<Rational>& P) -> Eigen::VectorXd {
        assert(P.cols() == 2);
        assert(P.rows() == 2 || P.rows() == 3);
        return Eigen::VectorXd::Constant(1, sqrt((P.col(0) - P.col(1)).squaredNorm().to_double()));
    };
    auto edge_length_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, Rational>>(
            edge_length_attribute,
            pt_attribute,
            compute_edge_length);
    edge_length_update->run_on_all();


    //////////////////////////////////
    // compute frozen vertices
    //////////////////////////////////
    auto frozen_vertex_attribute =
        mesh->register_attribute<int64_t>("frozen_vertex", PrimitiveType::Vertex, 1);
    auto frozen_vertex_accessor = mesh->create_accessor(frozen_vertex_attribute.as<int64_t>());

    auto input_ptr = mesh->get_child_meshes().front();

    int64_t frozen_cnt = 0;
    for (const auto& v : input_ptr->get_all(PrimitiveType::Vertex)) {
        if (input_ptr->is_boundary(PrimitiveType::Vertex, v)) {
            const auto& parent_v =
                input_ptr->map_to_parent(simplex::Simplex::vertex(*input_ptr, v));
            frozen_vertex_accessor.scalar_attribute(parent_v) = 1;
            frozen_cnt++;
        } else {
            // frozen_vertex_accessor.scalar_attribute(parent_v) = 0; // redundant, just for safe
        }
    }

    frozen_cnt = 0;

    for (const auto& v : mesh->get_all(PrimitiveType::Vertex)) {
        if (frozen_vertex_accessor.scalar_attribute(v) == 1) {
            frozen_cnt++;
        }
    }

    wmtk::logger().info("mesh has {} frozen vertices", frozen_cnt);

    //////////////////////////////////
    // default transfer
    auto pass_through_attributes = options.pass_through;
    pass_through_attributes.push_back(edge_length_attribute);
    pass_through_attributes.push_back(amips_attribute);
    // pass_through_attributes.push_back(target_edge_length_attribute);


    //////////////////////////////////
    // Lambdas for priority
    //////////////////////////////////
    auto long_edges_first = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return -edge_length_accessor.scalar_attribute(s.tuple());
    };
    auto short_edges_first = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return edge_length_accessor.scalar_attribute(s.tuple());
    };


    //////////////////////////////////
    // envelopes
    //////////////////////////////////

    using MeshConstrainPair = ProjectOperation::MeshConstrainPair;

    auto envelope_invariant = std::make_shared<InvariantCollection>(*mesh);
    std::vector<std::shared_ptr<AttributeTransferStrategyBase>> update_child_position,
        update_parent_position;
    std::vector<std::shared_ptr<Mesh>> envelopes;
    std::vector<MeshConstrainPair> mesh_constraint_pairs;

    std::vector<std::pair<std::shared_ptr<Mesh>, std::string>> multimesh_meshes;

    for (const auto& e : options.envelopes) {
        auto constrained_mesh = e.envelope_constrained_mesh;
        auto geometry_mesh = e.envelope_geometry_mesh;

        wmtk::logger().info(
            "wildmeshing2d: registered {} mesh as envelope constraints",
            e.envelope_name);

        const bool geometry_has_double_pos =
            geometry_mesh->has_attribute<double>(e.geometry_position_name, PrimitiveType::Vertex);
        const bool geometry_has_rational_pos =
            geometry_mesh->has_attribute<Rational>(e.geometry_position_name, PrimitiveType::Vertex);
        assert(geometry_has_double_pos || geometry_has_rational_pos);

        auto geometry_pt_handle = geometry_has_double_pos
                                      ? geometry_mesh->get_attribute_handle<double>(
                                            e.geometry_position_name,
                                            PrimitiveType::Vertex)
                                      : geometry_mesh->get_attribute_handle<Rational>(
                                            e.geometry_position_name,
                                            PrimitiveType::Vertex);

        auto constrained_pt_handle = constrained_mesh->get_attribute_handle<Rational>(
            e.constrained_position_name,
            PrimitiveType::Vertex);

        multimesh_meshes.push_back(std::make_pair(constrained_mesh, e.envelope_name));
        pass_through_attributes.emplace_back(constrained_pt_handle);

        mesh_constraint_pairs.emplace_back(geometry_pt_handle, constrained_pt_handle);

        envelope_invariant->add(std::make_shared<EnvelopeInvariant>(
            geometry_pt_handle,
            e.thickness * bbdiag,
            constrained_pt_handle));

        update_parent_position.emplace_back(attribute_update::make_cast_attribute_transfer_strategy(
            /*source=*/constrained_pt_handle,
            /*target=*/pt_attribute));

        update_child_position.emplace_back(attribute_update::make_cast_attribute_transfer_strategy(
            /*source=*/pt_attribute,
            /*target=*/constrained_pt_handle));
    }


    //////////////////////////////////
    // Invariants
    //////////////////////////////////

    wmtk::logger().trace("Going through invariants");
    auto inversion_invariant =
        std::make_shared<SimplexInversionInvariant<Rational>>(*mesh, pt_attribute.as<Rational>());

    std::shared_ptr<function::PerSimplexFunction> amips =
        std::make_shared<AMIPS>(*mesh, pt_attribute);
    // auto function_invariant = std::make_shared<FunctionInvariant>(mesh->top_simplex_type(),
    // amips);
    auto function_invariant =
        std::make_shared<MaxFunctionInvariant>(mesh->top_simplex_type(), amips);

    auto link_condition = std::make_shared<MultiMeshLinkConditionInvariant>(*mesh);

    auto todo_larger = std::make_shared<TodoLargerInvariant>(
        *mesh,
        edge_length_attribute.as<double>(),
        target_edge_length_attribute.as<double>(),
        4.0 / 3.0);

    auto todo_smaller = std::make_shared<TodoSmallerInvariant>(
        *mesh,
        edge_length_attribute.as<double>(),
        target_edge_length_attribute.as<double>(),
        4.0 / 5.0);


    auto interior_edge = std::make_shared<InteriorEdgeInvariant>(*mesh);

    for (const auto& em : multimesh_meshes) {
        interior_edge->add_boundary(*(em.first));
    }

    auto invariant_separate_substructures =
        std::make_shared<invariants::SeparateSubstructuresInvariant>(*mesh);

    auto frozen_vertex_invariant = std::make_shared<invariants::FrozenVertexInvariant>(
        *mesh,
        frozen_vertex_attribute.as<int64_t>());
    auto frozen_opp_vertex_invariant = std::make_shared<invariants::FrozenOppVertexInvariant>(
        *mesh,
        frozen_vertex_attribute.as<int64_t>());

    //////////////////////////////////
    // renew flags
    //////////////////////////////////
    auto visited_edge_flag =
        mesh->register_attribute<char>("visited_edge", PrimitiveType::Edge, 1, false, char(1));

    auto update_flag_func = [](const Eigen::MatrixX<Rational>& P) -> Eigen::VectorX<char> {
        assert(P.cols() == 2);
        assert(P.rows() == 2 || P.rows() == 3);
        return Eigen::VectorX<char>::Constant(1, char(1));
    };
    auto tag_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<char, Rational>>(
            visited_edge_flag,
            pt_attribute,
            update_flag_func);

    //////////////////////////////////
    // sizing field update flags
    //////////////////////////////////


    // TODO: add with sizing field
    // auto visited_vertex_flag =
    //     mesh->register_attribute<char>("visited_vertex", PrimitiveType::Vertex, 1, false,
    //     char(1));
    // pass_through_attributes.push_back(visited_vertex_flag);

    //////////////////////////////////
    // energy filter flag
    //////////////////////////////////

    // TODO: add energy filter

    // auto energy_filter_attribute =
    //     mesh->register_attribute<char>("energy_filter", PrimitiveType::Vertex, 1, false,
    //     char(1));

    // auto energy_filter_accessor = mesh->create_accessor<char>(energy_filter_attribute);

    // auto update_energy_filter_func = [](const Eigen::MatrixX<Rational>& P) ->
    // Eigen::VectorX<char> {
    //     // assert(P.cols() == 2);
    //     // assert(P.rows() == 2 || P.rows() == 3);
    //     return Eigen::VectorX<char>::Constant(1, char(1));
    // };
    // auto energy_filter_update =
    //     std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<char, Rational>>(
    //         energy_filter_attribute,
    //         pt_attribute,
    //         update_energy_filter_func);

    // pass_through_attributes.push_back(energy_filter_attribute);

    //////////////////////////////////
    // Creation of the 4 ops
    //////////////////////////////////
    std::vector<std::shared_ptr<Operation>> ops;
    std::vector<std::string> ops_name;

    //////////////////////////////////
    // 0) Rounding
    //////////////////////////////////
    auto rounding_pt_attribute = mesh->get_attribute_handle_typed<Rational>(
        options.input_mesh_position,
        PrimitiveType::Vertex);
    auto rounding = std::make_shared<Rounding>(*mesh, rounding_pt_attribute);
    rounding->add_invariant(
        std::make_shared<RoundedInvariant>(*mesh, pt_attribute.as<Rational>(), true));
    rounding->add_invariant(inversion_invariant);


    //////////////////////////////////
    // 1) EdgeSplit
    //////////////////////////////////
    auto split = std::make_shared<EdgeSplit>(*mesh);
    split->set_priority(long_edges_first);

    split->add_invariant(todo_larger);
    split->add_invariant(inversion_invariant);

    split->set_new_attribute_strategy(pt_attribute);
    // split->set_new_attribute_strategy(sizing_field_scalar_attribute);
    split->set_new_attribute_strategy(
        visited_edge_flag,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);

    split->set_new_attribute_strategy(
        frozen_vertex_attribute,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    split->set_new_attribute_strategy(
        target_edge_length_attribute,
        wmtk::operations::SplitBasicStrategy::Copy,
        wmtk::operations::SplitRibBasicStrategy::Mean);

    for (const auto& attr : pass_through_attributes) {
        split->set_new_attribute_strategy(
            attr,
            wmtk::operations::SplitBasicStrategy::None,
            wmtk::operations::SplitRibBasicStrategy::None);
    }

    // split->add_transfer_strategy(amips_update);
    split->add_transfer_strategy(edge_length_update);
    split->add_transfer_strategy(tag_update); // for renew the queue
    // split->add_transfer_strategy(energy_filter_update);

    // split->add_transfer_strategy(target_edge_length_update);

    auto split_then_round = std::make_shared<AndOperationSequence>(*mesh);
    split_then_round->add_operation(split);
    split_then_round->add_operation(rounding);

    for (auto& s : update_child_position) {
        split_then_round->add_transfer_strategy(s);
    }

    // split unrounded
    auto split_unrounded = std::make_shared<EdgeSplit>(*mesh);
    split_unrounded->set_priority(long_edges_first);

    split_unrounded->add_invariant(todo_larger);

    auto split_unrounded_transfer_strategy =
        std::make_shared<SplitNewAttributeStrategy<Rational>>(pt_attribute);
    split_unrounded_transfer_strategy->set_strategy(
        [](const Eigen::VectorX<Rational>& a, const std::bitset<2>&) {
            return std::array<Eigen::VectorX<Rational>, 2>{{a, a}};
        });
    split_unrounded_transfer_strategy->set_rib_strategy(
        [](const Eigen::VectorX<Rational>& p0_d,
           const Eigen::VectorX<Rational>& p1_d,
           const std::bitset<2>& bs) -> Eigen::VectorX<Rational> {
            Eigen::VectorX<Rational> p0(p0_d.size());
            Eigen::VectorX<Rational> p1(p1_d.size());
            for (int i = 0; i < p0_d.size(); ++i) {
                p0[i] = Rational(p0_d[i], false);
                p1[i] = Rational(p1_d[i], false);
            }
            if (bs[0] == bs[1]) {
                return (p0 + p1) / Rational(2, false);
            } else if (bs[0]) {
                return p0;

            } else {
                return p1;
            }
        });

    split_unrounded->set_new_attribute_strategy(pt_attribute, split_unrounded_transfer_strategy);
    // split_unrounded->set_new_attribute_strategy(sizing_field_scalar_attribute);
    split_unrounded->set_new_attribute_strategy(
        visited_edge_flag,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    split_unrounded->set_new_attribute_strategy(
        frozen_vertex_attribute,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    for (const auto& attr : pass_through_attributes) {
        split_unrounded->set_new_attribute_strategy(
            attr,
            wmtk::operations::SplitBasicStrategy::None,
            wmtk::operations::SplitRibBasicStrategy::None);
    }
    split_unrounded->set_new_attribute_strategy(
        target_edge_length_attribute,
        wmtk::operations::SplitBasicStrategy::Copy,
        wmtk::operations::SplitRibBasicStrategy::Mean);

    split_unrounded->add_transfer_strategy(amips_update);
    split_unrounded->add_transfer_strategy(edge_length_update);
    split_unrounded->add_transfer_strategy(tag_update); // for renew the queue
    // split_unrounded->add_transfer_strategy(energy_filter_update);

    // split->add_transfer_strategy(target_edge_length_update);

    auto split_sequence = std::make_shared<OrOperationSequence>(*mesh);
    split_sequence->add_operation(split_then_round);
    split_sequence->add_operation(split_unrounded);
    // split_sequence->add_invariant(
    //     std::make_shared<EnergyFilterInvariant>(*mesh, energy_filter_attribute.as<char>()));

    split_sequence->set_priority(long_edges_first);


    if (!options.skip_split) {
        ops.emplace_back(split_sequence);
        ops_name.emplace_back("SPLIT");

        ops.emplace_back(rounding);
        ops_name.emplace_back("rounding");
    }


    //////////////////////////////////
    // collapse transfer
    //////////////////////////////////
    auto clps_strat1 = std::make_shared<CollapseNewAttributeStrategy<Rational>>(pt_attribute);
    // clps_strat1->set_simplex_predicate(BasicSimplexPredicate::IsInterior);
    // clps_strat1->set_strategy(CollapseBasicStrategy::Default);
    clps_strat1->set_strategy(CollapseBasicStrategy::CopyOther);

    auto clps_strat2 = std::make_shared<CollapseNewAttributeStrategy<Rational>>(pt_attribute);
    // clps_strat2->set_simplex_predicate(BasicSimplexPredicate::IsInterior);
    // clps_strat2->set_strategy(CollapseBasicStrategy::Default);
    clps_strat2->set_strategy(CollapseBasicStrategy::CopyTuple);

    //////////////////////////////////
    // 2) EdgeCollapse
    //////////////////////////////////

    auto setup_collapse = [&](std::shared_ptr<EdgeCollapse>& collapse) {
        collapse->add_invariant(invariant_separate_substructures);
        collapse->add_invariant(std::make_shared<MultiMeshMapValidInvariant>(*mesh));
        collapse->add_invariant(link_condition);
        collapse->add_invariant(inversion_invariant);
        collapse->add_invariant(function_invariant);
        collapse->add_invariant(envelope_invariant);

        collapse->set_new_attribute_strategy(
            visited_edge_flag,
            wmtk::operations::CollapseBasicStrategy::None);

        collapse->add_transfer_strategy(tag_update);
        // collapse->add_transfer_strategy(energy_filter_update);
        for (const auto& attr : pass_through_attributes) {
            collapse->set_new_attribute_strategy(
                attr,
                wmtk::operations::CollapseBasicStrategy::None);
        }
        collapse->set_new_attribute_strategy(
            target_edge_length_attribute,
            wmtk::operations::CollapseBasicStrategy::None);
        // THis triggers a segfault in release
        // solved somehow
        // collapse->set_priority(short_edges_first);

        collapse->add_transfer_strategy(amips_update);
        collapse->add_transfer_strategy(edge_length_update);
        // collapse->add_transfer_strategy(target_edge_length_update);

        for (auto& s : update_child_position) {
            collapse->add_transfer_strategy(s);
        }
    };

    auto collapse1 = std::make_shared<EdgeCollapse>(*mesh);

    // TODO: implement for 2d
    // collapse1->add_invariant(std::make_shared<CollapseEnergyBeforeInvariant>(
    //     *mesh,
    //     pt_attribute.as<Rational>(),
    //     amips_attribute.as<double>(),
    //     1));

    collapse1->add_invariant(frozen_vertex_invariant);
    collapse1->set_new_attribute_strategy(pt_attribute, clps_strat1);
    collapse1->set_new_attribute_strategy(
        frozen_vertex_attribute,
        CollapseBasicStrategy::CopyOther);
    // collapse1->set_new_attribute_strategy(sizing_field_scalar_attribute, clps_strat1);
    setup_collapse(collapse1);

    auto collapse2 = std::make_shared<EdgeCollapse>(*mesh);
    // collapse2->add_invariant(std::make_shared<CollapseEnergyBeforeInvariant>(
    //     *mesh,
    //     pt_attribute.as<Rational>(),
    //     amips_attribute.as<double>(),
    //     0));

    collapse2->add_invariant(frozen_opp_vertex_invariant);
    collapse2->set_new_attribute_strategy(pt_attribute, clps_strat2);
    collapse2->set_new_attribute_strategy(
        frozen_vertex_attribute,
        CollapseBasicStrategy::CopyTuple);
    // collapse2->set_new_attribute_strategy(sizing_field_scalar_attribute, clps_strat2);
    setup_collapse(collapse2);

    auto collapse = std::make_shared<OrOperationSequence>(*mesh);
    collapse->add_operation(collapse1);
    collapse->add_operation(collapse2);
    collapse->add_invariant(todo_smaller);

    auto collapse_then_round = std::make_shared<AndOperationSequence>(*mesh);
    collapse_then_round->add_operation(collapse);
    collapse_then_round->add_operation(rounding);

    collapse_then_round->set_priority(short_edges_first);
    // collapse_then_round->add_invariant(
    //     std::make_shared<EnergyFilterInvariant>(*mesh, energy_filter_attribute.as<char>()));


    for (auto& s : update_child_position) {
        collapse_then_round->add_transfer_strategy(s);
    }

    if (!options.skip_collapse) {
        ops.emplace_back(collapse_then_round);
        ops_name.emplace_back("COLLAPSE");

        ops.emplace_back(rounding);
        ops_name.emplace_back("rounding");
    }

    //////////////////////////////////
    // 3) Swap
    //////////////////////////////////

    auto setup_swap = [&](Operation& op,
                          EdgeCollapse& collapse,
                          EdgeSplit& split,
                          std::shared_ptr<Invariant> simplex_invariant,
                          bool is_edge = true) {
        if (is_edge) op.set_priority(long_edges_first);

        op.add_invariant(simplex_invariant);
        op.add_invariant(
            std::make_shared<Swap2dUnroundedVertexInvariant>(*mesh, pt_attribute.as<Rational>()));
        op.add_invariant(inversion_invariant);
        op.add_invariant(function_invariant);

        op.add_transfer_strategy(amips_update);
        op.add_transfer_strategy(edge_length_update);
        op.add_transfer_strategy(tag_update);
        // op.add_transfer_strategy(target_edge_length_update);
        // for (auto& s : update_child_position) {
        //     op.add_transfer_strategy(s);
        // }

        collapse.add_invariant(invariant_separate_substructures);
        collapse.add_invariant(link_condition);


        collapse.set_new_attribute_strategy(pt_attribute, CollapseBasicStrategy::CopyOther);
        split.set_new_attribute_strategy(pt_attribute);

        split.set_new_attribute_strategy(
            visited_edge_flag,
            wmtk::operations::SplitBasicStrategy::None,
            wmtk::operations::SplitRibBasicStrategy::None);

        collapse.set_new_attribute_strategy(
            visited_edge_flag,
            wmtk::operations::CollapseBasicStrategy::None);

        split.set_new_attribute_strategy(
            frozen_vertex_attribute,
            wmtk::operations::SplitBasicStrategy::None,
            wmtk::operations::SplitRibBasicStrategy::None);

        collapse.set_new_attribute_strategy(
            frozen_vertex_attribute,
            CollapseBasicStrategy::CopyOther);

        split.set_new_attribute_strategy(
            target_edge_length_attribute,
            wmtk::operations::SplitBasicStrategy::Copy,
            wmtk::operations::SplitRibBasicStrategy::Mean);
        collapse.set_new_attribute_strategy(
            target_edge_length_attribute,
            wmtk::operations::CollapseBasicStrategy::None);

        // this might not be necessary
        // for (auto& s : update_child_position) {
        //     collapse.add_transfer_strategy(s);
        //     split.add_transfer_strategy(s);
        // }


        for (const auto& attr : pass_through_attributes) {
            split.set_new_attribute_strategy(
                attr,
                wmtk::operations::SplitBasicStrategy::None,
                wmtk::operations::SplitRibBasicStrategy::None);
            collapse.set_new_attribute_strategy(
                attr,
                wmtk::operations::CollapseBasicStrategy::None);
        }
    };


    auto swap = std::make_shared<TriEdgeSwap>(*mesh);
    setup_swap(*swap, swap->collapse(), swap->split(), interior_edge);

    if (!options.skip_swap) {
        ops.push_back(swap);
        ops_name.push_back("swap");

        ops.emplace_back(rounding);
        ops_name.emplace_back("rounding");
    }

    // 4) Smoothing
    // //////////////////////////////////////
    // // special smoothing on surface
    // //////////////////////////////////////

    // if (mesh->top_simplex_type() == PrimitiveType::Tetrahedron) {
    //     for (auto& pair : mesh_constraint_pairs) {
    //         if (pair.second.mesh().top_simplex_type() != PrimitiveType::Triangle) continue;

    //         auto& child_mesh = pair.second.mesh();
    //         auto& child_position_handle = pair.second;

    //         auto lap_smoothing = std::make_shared<TetWildTangentialLaplacianSmoothing>(
    //             child_mesh,
    //             child_position_handle.as<Rational>());
    //         lap_smoothing->add_invariant(std::make_shared<RoundedInvariant>(
    //             child_mesh,
    //             child_position_handle.as<Rational>()));
    //         lap_smoothing->add_invariant(inversion_invariant);

    //         auto proj_lap_smoothing =
    //             std::make_shared<ProjectOperation>(lap_smoothing, mesh_constraint_pairs);
    //         proj_lap_smoothing->use_random_priority() = true;

    //         proj_lap_smoothing->add_invariant(envelope_invariant);
    //         proj_lap_smoothing->add_invariant(inversion_invariant);
    //         proj_lap_smoothing->add_invariant(
    //             std::make_shared<EnergyFilterInvariant>(*mesh,
    //             energy_filter_attribute.as<char>()));

    //         proj_lap_smoothing->add_transfer_strategy(amips_update);
    //         proj_lap_smoothing->add_transfer_strategy(edge_length_update);
    //         for (auto& s : update_parent_position) { // TODO::this should from only one child
    //             proj_lap_smoothing->add_transfer_strategy(s);
    //         }
    //         for (auto& s : update_child_position) {
    //             proj_lap_smoothing->add_transfer_strategy(s);
    //         }

    //         ops.push_back(proj_lap_smoothing);
    //         ops_name.push_back("LAPLACIAN SMOOTHING");
    //     }
    // }

    // auto energy =
    //     std::make_shared<function::LocalNeighborsSumFunction>(*mesh, pt_attribute, *amips);
    // auto smoothing = std::make_shared<OptimizationSmoothing>(energy);
    auto smoothing = std::make_shared<AMIPSOptimizationSmoothing>(*mesh, pt_attribute);
    smoothing->add_invariant(
        std::make_shared<RoundedInvariant>(*mesh, pt_attribute.as<Rational>()));
    smoothing->add_invariant(frozen_vertex_invariant);
    smoothing->add_invariant(inversion_invariant);
    for (auto& s : update_child_position) {
        smoothing->add_transfer_strategy(s);
    }

    // test code
    // smoothing->add_invariant(envelope_invariant);
    // smoothing->add_transfer_strategy(edge_length_update);
    // ops.push_back(smoothing);
    // ops_name.push_back("SMOOTHING");
    // ops.emplace_back(rounding);
    // ops_name.emplace_back("rounding");
    //--------

    auto proj_smoothing = std::make_shared<ProjectOperation>(smoothing, mesh_constraint_pairs);
    proj_smoothing->use_random_priority() = true;
    proj_smoothing->add_invariant(frozen_vertex_invariant);
    proj_smoothing->add_invariant(envelope_invariant);
    proj_smoothing->add_invariant(inversion_invariant);
    // proj_smoothing->add_invariant(
    //     std::make_shared<EnergyFilterInvariant>(*mesh, energy_filter_attribute.as<char>()));

    proj_smoothing->add_transfer_strategy(amips_update);
    proj_smoothing->add_transfer_strategy(edge_length_update);
    for (auto& s : update_parent_position) {
        proj_smoothing->add_transfer_strategy(s);
    }

    for (auto& s : update_child_position) {
        proj_smoothing->add_transfer_strategy(s);
    }
    // proj_smoothing->add_transfer_strategy(target_edge_length_update);

    if (!options.skip_smooth) {
        for (int i = 0; i < 1; ++i) {
            // some old code to do smoothing several times, maybe useful later
            ops.push_back(proj_smoothing);
            ops_name.push_back("SMOOTHING");
        }

        ops.emplace_back(rounding);
        ops_name.emplace_back("rounding");
    }

    write(
        mesh,
        options.intermediate_output_path,
        options.intermediate_output_name,
        options.input_mesh_position,
        0,
        options.intermediate_output);

    //////////////////////////////////
    // Running all ops in order n times
    Scheduler scheduler;
    {
        const size_t freq = options.scheduler_update_frequency;
        scheduler.set_update_frequency(freq == 0 ? std::optional<size_t>{} : freq);
    }
    int64_t success = 10;

    //////////////////////////////////
    // preprocessing
    //////////////////////////////////

    // debug code
    for (const auto& t : mesh->get_all(mesh->top_simplex_type())) {
        const auto vertices = mesh->orient_vertices(t);
        std::vector<Vector2r> pos;
        for (int i = 0; i < vertices.size(); ++i) {
            pos.push_back(pt_accessor.const_vector_attribute(vertices[i]));
        }
        if (wmtk::utils::wmtk_orient2d(pos[0], pos[1], pos[2]) <= 0) {
            wmtk::logger().error("Flipped triangle!");
        }
    }

    SchedulerStats pre_stats;

    logger().info("----------------------- Preprocess Collapse -----------------------");

    logger().info("Executing collapse ...");


    wmtk::attribute::TypedAttributeHandle<char> visited_edge_flag_t = visited_edge_flag.as<char>();

    pre_stats = scheduler.run_operation_on_all(*collapse_then_round, visited_edge_flag_t);
    logger().info(
        "Executed {}, {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, "
        "executing: {}",
        "preprocessing collapse",
        pre_stats.number_of_performed_operations(),
        pre_stats.number_of_successful_operations(),
        pre_stats.number_of_failed_operations(),
        pre_stats.collecting_time,
        pre_stats.sorting_time,
        pre_stats.executing_time);

    success = pre_stats.number_of_successful_operations();

    // verbose logger, can be removed
    int64_t unrounded = 0;
    int64_t frozen = 0;
    for (const auto& v : mesh->get_all(PrimitiveType::Vertex)) {
        const auto p = pt_accessor.vector_attribute(v);
        for (int64_t d = 0; d < 2; ++d) {
            if (!p[d].is_rounded()) {
                ++unrounded;
                break;
            }
        }

        if (frozen_vertex_accessor.scalar_attribute(v) == 1) {
            frozen++;
        }
    }

    logger().info("Mesh has {} unrounded vertices", unrounded);
    logger().error("Mesh has {} frozen vertices", frozen);

    // debug code
    for (const auto& t : mesh->get_all(mesh->top_simplex_type())) {
        const auto vertices = mesh->orient_vertices(t);
        std::vector<Vector2r> pos;
        for (int i = 0; i < vertices.size(); ++i) {
            pos.push_back(pt_accessor.const_vector_attribute(vertices[i]));
        }
        if (wmtk::utils::wmtk_orient2d(pos[0], pos[1], pos[2]) <= 0) {
            wmtk::logger().error("Flipped triangle!");
        }
    }


    // compute max energy
    double max_energy = std::numeric_limits<double>::lowest();
    double min_energy = std::numeric_limits<double>::max();
    double avg_energy = 0;
    for (const auto& t : mesh->get_all(mesh->top_simplex_type())) {
        // double e = amips->get_value(simplex::Simplex(mesh->top_simplex_type(), t));
        double e = amips_accessor.scalar_attribute(t);
        max_energy = std::max(max_energy, e);
        min_energy = std::min(min_energy, e);
        avg_energy += e;
    }

    avg_energy = avg_energy / mesh->get_all(mesh->top_simplex_type()).size();

    logger().info(
        "Max AMIPS Energy: {}, Min AMIPS Energy: {}, Avg AMIPS Energy: {}",
        max_energy,
        min_energy,
        avg_energy);

    // std::ofstream file0("quality_plot_pre.csv");
    // file0 << "tid, quality" << std::endl;
    // int64_t t_cnt = 0;
    // for (const auto& t : mesh->get_all(PrimitiveType::Tetrahedron)) {
    //     t_cnt++;
    //     file0 << t_cnt << ", " << amips_accessor.scalar_attribute(t) << std::endl;
    // }


    double old_max_energy = max_energy;
    double old_avg_energy = avg_energy;
    int iii = 0;
    bool is_double = false;
    for (int64_t i = 0; i < options.max_passes; ++i) {
        logger().info("--------------------------- Pass {} ---------------------------", i);

        SchedulerStats pass_stats;
        int jj = 0;
        for (auto& op : ops) {
            logger().info("Executing {} ...", ops_name[jj]);
            SchedulerStats stats;
            if (op->primitive_type() == PrimitiveType::Edge) {
                stats = scheduler.run_operation_on_all(*op, visited_edge_flag_t);
                // } else if (ops_name[jj] == "SMOOTHING") {
                //     // stats = scheduler.run_operation_on_all(*op);
                //     stats =
                //         scheduler.run_operation_on_all_coloring(*op,
                //         coloring_attribute.as<int64_t>());
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

            // verbose logger, can be removed
            int64_t unrounded = 0;
            for (const auto& v : mesh->get_all(PrimitiveType::Vertex)) {
                const auto p = pt_accessor.vector_attribute(v);
                for (int64_t d = 0; d < 2; ++d) {
                    if (!p[d].is_rounded()) {
                        ++unrounded;
                        break;
                    }
                }
            }

            logger().info("Mesh has {} unrounded vertices", unrounded);

            // debug code
            for (const auto& t : mesh->get_all(mesh->top_simplex_type())) {
                const auto vertices = mesh->orient_vertices(t);
                std::vector<Vector2r> pos;
                for (int i = 0; i < vertices.size(); ++i) {
                    pos.push_back(pt_accessor.const_vector_attribute(vertices[i]));
                }
                if (wmtk::utils::wmtk_orient2d(pos[0], pos[1], pos[2]) <= 0) {
                    wmtk::logger().error("Flipped triangle!");
                }
            }

            avg_energy = 0;

            // compute max energy
            max_energy = std::numeric_limits<double>::lowest();
            min_energy = std::numeric_limits<double>::max();
            for (const auto& t : mesh->get_all(mesh->top_simplex_type())) {
                // double e = amips->get_value(simplex::Simplex(mesh->top_simplex_type(), t));
                double e = amips_accessor.scalar_attribute(t);
                max_energy = std::max(max_energy, e);
                min_energy = std::min(min_energy, e);
                avg_energy += e;
            }

            avg_energy = avg_energy / mesh->get_all(mesh->top_simplex_type()).size();

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

        multimesh::consolidate(*mesh);

        write(
            mesh,
            options.intermediate_output_path,
            options.intermediate_output_name,
            options.input_mesh_position,
            i + 1,
            options.intermediate_output);

        assert(mesh->is_connectivity_valid());

        // compute max energy
        max_energy = std::numeric_limits<double>::lowest();
        min_energy = std::numeric_limits<double>::max();
        avg_energy = 0;
        for (const auto& t : mesh->get_all(mesh->top_simplex_type())) {
            // double e = amips->get_value(simplex::Simplex(mesh->top_simplex_type(), t));
            double e = amips_accessor.scalar_attribute(t);
            max_energy = std::max(max_energy, e);
            min_energy = std::min(min_energy, e);
            avg_energy += e;
        }

        avg_energy = avg_energy / mesh->get_all(mesh->top_simplex_type()).size();

        int64_t unrounded = 0;
        if (!is_double) {
            bool rational = false;
            for (const auto& v : mesh->get_all(PrimitiveType::Vertex)) {
                const auto p = pt_accessor.vector_attribute(v);
                for (int64_t d = 0; d < bmax.size(); ++d) {
                    if (!p[d].is_rounded()) {
                        rational = true;
                        ++unrounded;
                        break;
                    }
                }
            }

            is_double = !rational;
        }

        logger().info("Mesh has {} unrounded vertices", unrounded);
        logger().info(
            "Max AMIPS Energy: {}, Min AMIPS Energy: {}, Avg AMIPS Energy: {}",
            max_energy,
            min_energy,
            avg_energy);


        // adjust sizing field
        // if (i > 0 && old_max_energy - max_energy < 5e-1 &&
        //     (old_avg_energy - avg_energy) / avg_energy < 0.1) {
        //     wmtk::logger().info("adjusting sizing field ...");

        //     adjust_sizing_field(
        //         *mesh,
        //         pt_attribute.as<Rational>(),
        //         edge_length_attribute.as<double>(),
        //         sizing_field_scalar_attribute.as<double>(),
        //         amips_attribute.as<double>(),
        //         target_edge_length_attribute.as<double>(),
        //         visited_vertex_flag.as<char>(),
        //         target_max_amips,
        //         max_energy,
        //         target_edge_length,
        //         min_edge_length);

        //     wmtk::logger().info("adjusting sizing field finished");

        //     // wmtk::logger().info("setting energy filter ...");
        //     // set_operation_energy_filter_after_sizing_field(
        //     //     *mesh,
        //     //     pt_attribute.as<Rational>(),
        //     //     amips_attribute.as<double>(),
        //     //     energy_filter_attribute.as<char>(),
        //     //     visited_vertex_flag.as<char>(),
        //     //     target_max_amips,
        //     //     max_energy,
        //     //     target_edge_length);
        //     // wmtk::logger().info("setting energy filter finished");

        //     // int64_t e_cnt = 0;
        //     // for (const auto& e : mesh->get_all(PrimitiveType::Edge)) {
        //     //     if (energy_filter_accessor.scalar_attribute(e) == char(1) ||
        //     //         energy_filter_accessor.scalar_attribute(
        //     //             mesh->switch_tuple(e, PrimitiveType::Vertex)) == char(1)) {
        //     //         e_cnt++;
        //     //     }
        //     // }
        //     // wmtk::logger().info(
        //     //     "{} edges are going to be executed out of {}",
        //     //     e_cnt,
        //     //     mesh->get_all(PrimitiveType::Edge).size());

        //     for (const auto& v : mesh->get_all(PrimitiveType::Vertex)) {
        //         energy_filter_accessor.scalar_attribute(v) = char(1);
        //     }
        //     wmtk::logger().info("reset energy filter");
        // } else {
        //     wmtk::logger().info("setting energy filter ...");
        //     set_operation_energy_filter(
        //         *mesh,
        //         pt_attribute.as<Rational>(),
        //         amips_attribute.as<double>(),
        //         energy_filter_attribute.as<char>(),
        //         visited_vertex_flag.as<char>(),
        //         target_max_amips,
        //         max_energy,
        //         target_edge_length);
        //     wmtk::logger().info("setting energy filter finished");

        //     int64_t e_cnt = 0;
        //     for (const auto& e : mesh->get_all(PrimitiveType::Edge)) {
        //         if (energy_filter_accessor.scalar_attribute(e) == char(1) ||
        //             energy_filter_accessor.scalar_attribute(
        //                 mesh->switch_tuple(e, PrimitiveType::Vertex)) == char(1)) {
        //             e_cnt++;
        //         }
        //     }
        //     wmtk::logger().info(
        //         "{} edges are going to be executed out of {}",
        //         e_cnt,
        //         mesh->get_all(PrimitiveType::Edge).size());
        // }

        old_max_energy = max_energy;
        old_avg_energy = avg_energy;

        // stop at good quality
        if (max_energy <= target_max_amips && is_double) break;
    }

    logger().info("----------------------- Postprocess Collapse -----------------------");

    logger().info("Executing collapse ...");

    auto post_stats = scheduler.run_operation_on_all(*collapse_then_round, visited_edge_flag_t);
    logger().info(
        "Executed {}, {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, "
        "executing: {}",
        "preprocessing collapse",
        post_stats.number_of_performed_operations(),
        post_stats.number_of_successful_operations(),
        post_stats.number_of_failed_operations(),
        post_stats.collecting_time,
        post_stats.sorting_time,
        post_stats.executing_time);

    // compute max energy
    max_energy = std::numeric_limits<double>::lowest();
    min_energy = std::numeric_limits<double>::max();
    avg_energy = 0;
    for (const auto& t : mesh->get_all(mesh->top_simplex_type())) {
        // double e = amips->get_value(simplex::Simplex(mesh->top_simplex_type(), t));
        double e = amips_accessor.scalar_attribute(t);
        max_energy = std::max(max_energy, e);
        min_energy = std::min(min_energy, e);
        avg_energy += e;
    }

    avg_energy = avg_energy / mesh->get_all(mesh->top_simplex_type()).size();

    logger().info(
        "Max AMIPS Energy: {}, Min AMIPS Energy: {}, Avg AMIPS Energy: {}",
        max_energy,
        min_energy,
        avg_energy);

    multimesh::consolidate(*mesh);

    std::vector<std::pair<std::shared_ptr<Mesh>, std::string>> all_meshes;
    all_meshes.push_back(std::make_pair(mesh, "main"));

    for (const auto& p : multimesh_meshes) {
        all_meshes.push_back(p);
    }

    return all_meshes;
}

} // namespace wmtk::components::internal
