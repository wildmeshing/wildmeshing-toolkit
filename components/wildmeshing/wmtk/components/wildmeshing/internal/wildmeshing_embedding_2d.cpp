#include "wildmeshing2d.hpp"

#include "WildmeshingOptions.hpp"
#include "wildmeshing_utils.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/cast_attribute.hpp>

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
#include <wmtk/operations/composite/ProjectOperation.hpp>
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

#include <wmtk/submesh/Embedding.hpp>
#include <wmtk/submesh/SubMesh.hpp>
#include <wmtk/submesh/utils/submesh_from_multimesh.hpp>

#include <wmtk/utils/Rational.hpp>
#include <wmtk/utils/bbox_from_mesh.hpp>
#include <wmtk/utils/orient.hpp>

#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>

#include <wmtk/simplex/cofaces_single_dimension_iterable.hpp>

namespace wmtk::components::internal {

using namespace simplex;
using namespace operations;
using namespace operations::tri_mesh;
using namespace operations::tet_mesh;
using namespace operations::composite;
using namespace function;
using namespace invariants;

double compute_max_amips(const Mesh& mesh, const attribute::MeshAttributeHandle& amips_handle)
{
    const auto amips_accessor = mesh.create_const_accessor<double>(amips_handle);

    // compute max energy
    double max_energy = std::numeric_limits<double>::lowest();
    double min_energy = std::numeric_limits<double>::max();
    double avg_energy = 0;
    for (const Tuple& t : mesh.get_all(mesh.top_simplex_type())) {
        double e = amips_accessor.const_scalar_attribute(t);
        max_energy = std::max(max_energy, e);
        min_energy = std::min(min_energy, e);
        avg_energy += e;
    }

    avg_energy = avg_energy / mesh.get_all(mesh.top_simplex_type()).size();

    logger().info(
        "Max AMIPS Energy: {:>6.2f}, Min AMIPS Energy: {:>6.2f}, Avg AMIPS Energy: {:>6.2f}",
        max_energy,
        min_energy,
        avg_energy);

    return max_energy;
}

void print_stats(const wmtk::SchedulerStats& stats, const std::string& name = "")
{
    if (name.empty()) {
        logger().info(
            "Executed {} ops (S/F) {}/{}. Time executing: {:.4f}",
            stats.number_of_performed_operations(),
            stats.number_of_successful_operations(),
            stats.number_of_failed_operations(),
            stats.executing_time);
    } else {
        logger().info(
            "Executed {}, {} ops (S/F) {}/{}. Time executing: {:.4f}",
            name,
            stats.number_of_performed_operations(),
            stats.number_of_successful_operations(),
            stats.number_of_failed_operations(),
            stats.executing_time);
    }
}

void print_unrounded_vertices(const Mesh& mesh, const attribute::MeshAttributeHandle& pt_handle)
{
    const auto pt_accessor = mesh.create_const_accessor<Rational>(pt_handle);

    // verbose logger, can be removed
    int64_t unrounded = 0;
    for (const auto& v : mesh.get_all(PrimitiveType::Vertex)) {
        const auto p = pt_accessor.const_vector_attribute(v);
        for (int64_t d = 0; d < 2; ++d) {
            if (!p[d].is_rounded()) {
                ++unrounded;
                break;
            }
        }
    }

    if (unrounded > 0) {
        logger().info("Mesh has {} unrounded vertices", unrounded);
    }
}

void test_flipped_triangles(const Mesh& mesh, const attribute::MeshAttributeHandle& pt_handle)
{
    const auto pt_accessor = mesh.create_const_accessor<Rational>(pt_handle);

    for (const Tuple& t : mesh.get_all(mesh.top_simplex_type())) {
        const auto vertices = mesh.orient_vertices(t);
        std::vector<Vector2r> pos;
        for (int i = 0; i < vertices.size(); ++i) {
            pos.push_back(pt_accessor.const_vector_attribute(vertices[i]));
        }
        if (wmtk::utils::wmtk_orient2d(pos[0], pos[1], pos[2]) <= 0) {
            logger().error("Flipped triangle!");
        }
    }
}

std::vector<std::pair<std::shared_ptr<Mesh>, std::string>> wildmeshing_embedding_2d(
    const WildMeshingOptions& options)
{
    auto& mesh = *options.input_mesh;

    if (!mesh.is_connectivity_valid()) {
        log_and_throw_error("input mesh for wildmeshing connectivity invalid");
    }

    logger().trace("Getting rational point handle");

    //////////////////////////////////
    // Retriving vertices
    //
    if (options.replace_double_coordinate) {
        logger().trace("Found double attribute");
        auto pt_double_attribute =
            mesh.get_attribute_handle<double>(options.input_mesh_position, PrimitiveType::Vertex);

        if (!mesh.has_attribute<Rational>(options.input_mesh_position, PrimitiveType::Vertex)) {
            wmtk::utils::cast_attribute<wmtk::Rational>(
                pt_double_attribute,
                mesh,
                options.input_mesh_position);


        } else {
            auto pt_attribute = mesh.get_attribute_handle<Rational>(
                options.input_mesh_position,
                PrimitiveType::Vertex);
            wmtk::utils::cast_attribute<wmtk::Rational>(pt_double_attribute, pt_attribute);
        }
        mesh.delete_attribute(pt_double_attribute);
    }
    auto pt_attribute =
        mesh.get_attribute_handle<Rational>(options.input_mesh_position, PrimitiveType::Vertex);
    logger().trace("Getting rational point accessor");
    auto pt_accessor = mesh.create_accessor(pt_attribute.as<Rational>());

    //////////////////////////////////
    // computing bbox diagonal
    const double bbdiag = wmtk::utils::bbox_diagonal_from_mesh(pt_attribute);
    logger().info("bbox diag = {}", bbdiag);

    const double target_edge_length = options.target_edge_length * bbdiag;
    logger().info("target edge length: {}", target_edge_length);

    //////////////////////////////////
    // store amips
    auto amips_attribute =
        mesh.register_attribute<double>("wildmeshing_amips", mesh.top_simplex_type(), 1);
    auto amips_accessor = mesh.create_accessor(amips_attribute.as<double>());
    // amips update
    auto compute_amips = [](const Eigen::MatrixX<Rational>& P) -> Eigen::VectorXd {
        assert(P.rows() == 2 || P.rows() == 3); // rows --> attribute dimension
        assert(P.cols() == 3);
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
    };
    auto amips_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, Rational>>(
            amips_attribute,
            pt_attribute,
            compute_amips);
    amips_update->run_on_all();

    double max_amips;
    max_amips = compute_max_amips(mesh, amips_attribute);

    //////////////////////////////////
    // Storing target edge length
    auto target_edge_length_attribute = mesh.register_attribute<double>(
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
            for (const EnvelopeOptions& e : options.envelopes) {
                r = std::max(r, e.thickness);
            }
            assert(r > 0);
            return r;
        }
    }();
    const double& target_max_amips = options.target_max_amips;


    //////////////////////////////////
    // Storing edge lengths
    auto edge_length_attribute =
        mesh.register_attribute<double>("edge_length", PrimitiveType::Edge, 1);
    auto edge_length_accessor = mesh.create_accessor(edge_length_attribute.as<double>());
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
    // substructures
    //////////////////////////////////
    auto emb_ptr = submesh::utils::submesh_from_multimesh(options.input_mesh);
    submesh::Embedding emb = *emb_ptr;

    //////////////////////////////////
    // compute frozen vertices
    //////////////////////////////////
    auto frozen_vertex_attribute =
        mesh.register_attribute<int64_t>("frozen_vertex", PrimitiveType::Vertex, 1);
    auto frozen_vertex_accessor = mesh.create_accessor(frozen_vertex_attribute.as<int64_t>());

    for (const auto& sub_ptr : emb.get_child_meshes()) {
        submesh::SubMesh& sub = *sub_ptr;
        for (const simplex::IdSimplex& v : sub.get_all_id_simplex(PrimitiveType::Vertex)) {
            int64_t counter = 0;
            for (const Tuple& cof : cofaces_single_dimension_iterable(
                     mesh,
                     mesh.get_simplex(v),
                     sub.top_simplex_type())) {
                if (sub.contains(cof, sub.top_simplex_type())) {
                    ++counter;
                }
            }
            if (counter != 2) {
                frozen_vertex_accessor.scalar_attribute(v) = 1;
            }
        }
    }

    // auto input_ptr = mesh.get_child_meshes().front();
    //
    // for (const Tuple& v : input_ptr->get_all(PrimitiveType::Vertex)) {
    //     if (input_ptr->is_boundary(PrimitiveType::Vertex, v)) {
    //         const auto& parent_v =
    //             input_ptr->map_to_parent(simplex::Simplex::vertex(*input_ptr, v));
    //         frozen_vertex_accessor.scalar_attribute(parent_v) = 1;
    //     }
    // }

    {
        int64_t frozen_cnt = 0;
        for (const auto& v : mesh.get_all(PrimitiveType::Vertex)) {
            if (frozen_vertex_accessor.scalar_attribute(v) == 1) {
                frozen_cnt++;
            }
        }
        logger().info("mesh has {} frozen vertices", frozen_cnt);
    }

    //////////////////////////////////
    // default transfer
    auto pass_through_attributes = options.pass_through;
    pass_through_attributes.push_back(edge_length_attribute);
    pass_through_attributes.push_back(amips_attribute);


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

    auto envelope_invariant = std::make_shared<InvariantCollection>(mesh);

    std::vector<std::shared_ptr<Mesh>> envelopes;
    std::vector<MeshConstrainPair> mesh_constraint_pairs; // TODO remove

    std::vector<std::pair<std::shared_ptr<Mesh>, std::string>> multimesh_meshes;

    for (const EnvelopeOptions& e : options.envelopes) {
        Mesh& constrained_mesh = *e.envelope_constrained_mesh;
        Mesh& geometry_mesh = *e.envelope_geometry_mesh;

        logger().info("wildmeshing2d: registered {} mesh as envelope constraints", e.envelope_name);

        // auto constrained_pt_handle = constrained_mesh.get_attribute_handle<Rational>(
        //     e.constrained_position_name,
        //     PrimitiveType::Vertex);

        // multimesh_meshes.push_back(std::make_pair(e.envelope_constrained_mesh, e.envelope_name));
        //  pass_through_attributes.emplace_back(constrained_pt_handle);

        // mesh_constraint_pairs.emplace_back(geometry_pt_handle, constrained_pt_handle);

        submesh::SubMesh& sub = *emb.get_child_meshes()[0];

        envelope_invariant->add(
            std::make_shared<EnvelopeInvariant>(pt_attribute, e.thickness * bbdiag, sub));
    }


    //////////////////////////////////
    // Invariants
    //////////////////////////////////

    logger().trace("Going through invariants");
    auto inversion_invariant =
        std::make_shared<SimplexInversionInvariant<Rational>>(mesh, pt_attribute.as<Rational>());

    std::shared_ptr<function::PerSimplexFunction> amips =
        std::make_shared<AMIPS>(mesh, pt_attribute);
    // auto function_invariant = std::make_shared<FunctionInvariant>(mesh.top_simplex_type(),
    // amips);
    auto function_invariant =
        std::make_shared<MaxFunctionInvariant>(mesh.top_simplex_type(), amips);

    auto link_condition = std::make_shared<MultiMeshLinkConditionInvariant>(mesh);

    auto todo_larger = std::make_shared<TodoLargerInvariant>(
        mesh,
        edge_length_attribute.as<double>(),
        target_edge_length_attribute.as<double>(),
        4.0 / 3.0);

    auto todo_smaller = std::make_shared<TodoSmallerInvariant>(
        mesh,
        edge_length_attribute.as<double>(),
        target_edge_length_attribute.as<double>(),
        4.0 / 5.0);


    auto interior_edge = std::make_shared<InteriorEdgeInvariant>(mesh);

    for (const auto& em : multimesh_meshes) {
        interior_edge->add_boundary(*(em.first));
    }

    auto invariant_separate_substructures =
        std::make_shared<invariants::SeparateSubstructuresInvariant>(
            mesh); // TODO remove for submesh

    auto frozen_vertex_invariant = std::make_shared<invariants::FrozenVertexInvariant>(
        mesh,
        frozen_vertex_attribute.as<int64_t>());
    auto frozen_opp_vertex_invariant = std::make_shared<invariants::FrozenOppVertexInvariant>(
        mesh,
        frozen_vertex_attribute.as<int64_t>());

    //////////////////////////////////
    // renew flags
    //////////////////////////////////
    auto visited_edge_flag =
        mesh.register_attribute<char>("visited_edge", PrimitiveType::Edge, 1, false, char(1));

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
    // Creation of the 4 ops
    //////////////////////////////////
    std::vector<std::shared_ptr<Operation>> ops;
    std::vector<std::string> ops_name;

    //////////////////////////////////
    // 0) Rounding
    //////////////////////////////////
    auto rounding_pt_attribute = mesh.get_attribute_handle_typed<Rational>(
        options.input_mesh_position,
        PrimitiveType::Vertex);
    auto rounding = std::make_shared<Rounding>(mesh, rounding_pt_attribute);
    rounding->add_invariant(
        std::make_shared<RoundedInvariant>(mesh, pt_attribute.as<Rational>(), true));
    rounding->add_invariant(inversion_invariant);


    //////////////////////////////////
    // 1) EdgeSplit
    //////////////////////////////////
    auto split = std::make_shared<EdgeSplit>(mesh);
    split->set_priority(long_edges_first);

    split->add_invariant(todo_larger);
    split->add_invariant(inversion_invariant);

    split->set_new_attribute_strategy(pt_attribute);
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

    auto split_then_round = std::make_shared<AndOperationSequence>(mesh);
    split_then_round->add_operation(split);
    split_then_round->add_operation(rounding);

    // split unrounded
    auto split_unrounded = std::make_shared<EdgeSplit>(mesh);
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

    auto split_sequence = std::make_shared<OrOperationSequence>(mesh);
    split_sequence->add_operation(split_then_round);
    split_sequence->add_operation(split_unrounded);

    split_sequence->set_priority(long_edges_first);

    split_sequence->add_transfer_strategy(amips_update);
    split_sequence->add_transfer_strategy(edge_length_update);
    split_sequence->add_transfer_strategy(tag_update); // for renew the queue


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
    clps_strat1->set_strategy(CollapseBasicStrategy::CopyOther);

    auto clps_strat2 = std::make_shared<CollapseNewAttributeStrategy<Rational>>(pt_attribute);
    clps_strat2->set_strategy(CollapseBasicStrategy::CopyTuple);

    //////////////////////////////////
    // 2) EdgeCollapse
    //////////////////////////////////

    auto setup_collapse = [&](std::shared_ptr<EdgeCollapse>& collapse) {
        collapse->add_invariant(invariant_separate_substructures);
        collapse->add_invariant(std::make_shared<MultiMeshMapValidInvariant>(mesh));
        collapse->add_invariant(link_condition);
        collapse->add_invariant(inversion_invariant);
        collapse->add_invariant(function_invariant);
        collapse->add_invariant(envelope_invariant);

        collapse->set_new_attribute_strategy(
            visited_edge_flag,
            wmtk::operations::CollapseBasicStrategy::None);

        for (const auto& attr : pass_through_attributes) {
            collapse->set_new_attribute_strategy(
                attr,
                wmtk::operations::CollapseBasicStrategy::None);
        }
        collapse->set_new_attribute_strategy(
            target_edge_length_attribute,
            wmtk::operations::CollapseBasicStrategy::None);

        collapse->add_transfer_strategy(tag_update);
        collapse->add_transfer_strategy(amips_update);
        collapse->add_transfer_strategy(edge_length_update);
    };

    auto collapse1 = std::make_shared<EdgeCollapse>(mesh);

    collapse1->add_invariant(frozen_vertex_invariant);
    collapse1->set_new_attribute_strategy(pt_attribute, clps_strat1);
    collapse1->set_new_attribute_strategy(
        frozen_vertex_attribute,
        CollapseBasicStrategy::CopyOther);
    setup_collapse(collapse1);

    auto collapse2 = std::make_shared<EdgeCollapse>(mesh);

    collapse2->add_invariant(frozen_opp_vertex_invariant);
    collapse2->set_new_attribute_strategy(pt_attribute, clps_strat2);
    collapse2->set_new_attribute_strategy(
        frozen_vertex_attribute,
        CollapseBasicStrategy::CopyTuple);
    setup_collapse(collapse2);

    auto collapse = std::make_shared<OrOperationSequence>(mesh);
    collapse->add_operation(collapse1);
    collapse->add_operation(collapse2);
    collapse->add_invariant(todo_smaller);

    auto collapse_then_round = std::make_shared<AndOperationSequence>(mesh);
    collapse_then_round->add_operation(collapse);
    collapse_then_round->add_operation(rounding);

    collapse_then_round->set_priority(short_edges_first);

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
            std::make_shared<Swap2dUnroundedVertexInvariant>(mesh, pt_attribute.as<Rational>()));
        op.add_invariant(inversion_invariant);
        op.add_invariant(function_invariant);

        op.add_transfer_strategy(amips_update);
        op.add_transfer_strategy(edge_length_update);
        op.add_transfer_strategy(tag_update);

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


    auto swap = std::make_shared<TriEdgeSwap>(mesh);
    setup_swap(*swap, swap->collapse(), swap->split(), interior_edge);

    if (!options.skip_swap) {
        ops.push_back(swap);
        ops_name.push_back("swap");

        ops.emplace_back(rounding);
        ops_name.emplace_back("rounding");
    }

    /////////////////////////////////////////
    // 4) Smoothing
    /////////////////////////////////////////
    auto smoothing = std::make_shared<AMIPSOptimizationSmoothing>(mesh, pt_attribute);
    smoothing->add_invariant(std::make_shared<RoundedInvariant>(mesh, pt_attribute.as<Rational>()));
    smoothing->add_invariant(frozen_vertex_invariant);
    smoothing->add_invariant(inversion_invariant);

    auto proj_smoothing = std::make_shared<ProjectOperation>(smoothing, mesh_constraint_pairs);
    // proj_smoothing->use_random_priority() = true;
    proj_smoothing->add_invariant(frozen_vertex_invariant);
    proj_smoothing->add_invariant(envelope_invariant);
    proj_smoothing->add_invariant(inversion_invariant);

    proj_smoothing->add_transfer_strategy(amips_update);
    proj_smoothing->add_transfer_strategy(edge_length_update);


    if (!options.skip_smooth) {
        for (int i = 0; i < 1; ++i) {
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

    //////////////////////////////////
    // preprocessing
    //////////////////////////////////

    // debug code
    test_flipped_triangles(mesh, pt_attribute);

    {
        logger().info("----------------------- Preprocess Collapse -----------------------");
        // logger().info("Executing collapse ...");

        SchedulerStats pre_stats =
            scheduler.run_operation_on_all(*collapse_then_round, visited_edge_flag.as<char>());
        print_stats(pre_stats, "preprocessing collapse");

        // verbose logger, can be removed
        print_unrounded_vertices(mesh, pt_attribute);
        test_flipped_triangles(mesh, pt_attribute);
        max_amips = compute_max_amips(mesh, amips_attribute);
    }

    int iii = 0;
    bool is_double = false;
    for (int64_t i = 0; i < options.max_passes; ++i) {
        logger().info("--------------------------- Pass {} ---------------------------", i);

        SchedulerStats pass_stats;
        int jj = 0;
        for (auto& op : ops) {
            // logger().info("Executing {} ...", ops_name[jj]);

            SchedulerStats stats;
            if (op->primitive_type() == PrimitiveType::Edge) {
                stats = scheduler.run_operation_on_all(*op, visited_edge_flag.as<char>());
            } else {
                stats = scheduler.run_operation_on_all(*op);
            }
            pass_stats += stats;
            print_stats(stats, ops_name[jj]);

            // verbose logger, can be removed
            print_unrounded_vertices(mesh, pt_attribute);
            test_flipped_triangles(mesh, pt_attribute);
            max_amips = compute_max_amips(mesh, amips_attribute);

            ++jj;
        }

        print_stats(pass_stats);

        multimesh::consolidate(mesh);

        write(
            mesh,
            options.intermediate_output_path,
            options.intermediate_output_name,
            options.input_mesh_position,
            i + 1,
            options.intermediate_output);

        assert(mesh.is_connectivity_valid());

        print_unrounded_vertices(mesh, pt_attribute);
        max_amips = compute_max_amips(mesh, amips_attribute);

        // stop at good quality
        if (max_amips <= target_max_amips && is_double) {
            break;
        }
    }

    logger().info("----------------------- Postprocess Collapse -----------------------");

    // logger().info("Executing collapse ...");

    auto post_stats =
        scheduler.run_operation_on_all(*collapse_then_round, visited_edge_flag.as<char>());
    print_stats(post_stats, "preprocessing collapse");

    max_amips = compute_max_amips(mesh, amips_attribute);

    multimesh::consolidate(mesh);

    std::vector<std::pair<std::shared_ptr<Mesh>, std::string>> all_meshes;
    all_meshes.push_back(std::make_pair(options.input_mesh, "main"));

    for (const auto& p : multimesh_meshes) {
        all_meshes.push_back(p);
    }

    return all_meshes;
}

} // namespace wmtk::components::internal
