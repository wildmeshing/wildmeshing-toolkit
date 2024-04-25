#include "wildmeshing.hpp"

#include "WildmeshingOptions.hpp"

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
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/OperationSequence.hpp>
#include <wmtk/operations/OptimizationSmoothing.hpp>
#include <wmtk/operations/Rounding.hpp>
#include <wmtk/operations/composite/ProjectOperation.hpp>
#include <wmtk/operations/composite/TetEdgeSwap.hpp>
#include <wmtk/operations/composite/TetFaceSwap.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>


#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/function/PerSimplexFunction.hpp>
#include <wmtk/function/simplex/AMIPS.hpp>

#include <wmtk/invariants/EdgeValenceInvariant.hpp>
#include <wmtk/invariants/EnvelopeInvariant.hpp>
#include <wmtk/invariants/FunctionInvariant.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorSimplexInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/MaxFunctionInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/NoBoundaryCollapseToInteriorInvariant.hpp>
#include <wmtk/invariants/RoundedInvariant.hpp>
#include <wmtk/invariants/SeparateSubstructuresInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/Swap23EnergyBeforeInvariant.hpp>
#include <wmtk/invariants/Swap32EnergyBeforeInvariant.hpp>
#include <wmtk/invariants/Swap44EnergyBeforeInvariant.hpp>
#include <wmtk/invariants/Swap44_2EnergyBeforeInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>

#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>

#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>

#include <wmtk/utils/Rational.hpp>


namespace wmtk::components {

using namespace simplex;
using namespace operations;
using namespace operations::tri_mesh;
using namespace operations::tet_mesh;
using namespace operations::composite;
using namespace function;
using namespace invariants;

namespace {
void write(
    const std::shared_ptr<Mesh>& mesh,
    const std::string& out_dir,
    const std::string& name,
    const std::string& vname,
    const int64_t index,
    const bool intermediate_output)
{
    if (intermediate_output) {
        if (mesh->top_simplex_type() == PrimitiveType::Triangle) {
            // write trimesh
            const std::filesystem::path data_dir = "";
            wmtk::io::ParaviewWriter writer(
                data_dir / (name + "_" + std::to_string(index)),
                "vertices",
                *mesh,
                true,
                true,
                true,
                false);
            mesh->serialize(writer);
        } else if (mesh->top_simplex_type() == PrimitiveType::Tetrahedron) {
            // write tetmesh
            const std::filesystem::path data_dir = "";
            wmtk::io::ParaviewWriter writer(
                data_dir / (name + "_" + std::to_string(index)),
                "vertices",
                *mesh,
                true,
                true,
                true,
                true);
            mesh->serialize(writer);
        } else if (mesh->top_simplex_type() == PrimitiveType::Edge) {
            // write edgemesh
            const std::filesystem::path data_dir = "";
            wmtk::io::ParaviewWriter writer(
                data_dir / (name + "_" + std::to_string(index)),
                "vertices",
                *mesh,
                true,
                true,
                false,
                false);
            mesh->serialize(writer);
        }
    }
}

} // namespace

void wildmeshing(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    //////////////////////////////////
    // Load mesh from settings
    WildmeshingOptions options = j.get<WildmeshingOptions>();
    const std::filesystem::path& file = options.input;
    auto mesh = cache.read_mesh(options.input);

    if (!mesh->is_connectivity_valid()) {
        throw std::runtime_error("input mesh for wildmeshing connectivity invalid");
    }


    //////////////////////////////////
    // Retriving vertices
    auto pt_attribute =
        mesh->get_attribute_handle<Rational>(options.attributes.position, PrimitiveType::Vertex);
    auto pt_accessor = mesh->create_accessor(pt_attribute.as<Rational>());

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
    // Storing target edge length
    auto target_edge_length_attribute = mesh->register_attribute<double>(
        "wildmeshing_target_edge_length",
        PrimitiveType::Edge,
        1,
        false,
        target_edge_length); // defaults to target edge length

    // Target edge length update
    const double min_edge_length =
        options.envelopes.empty()
            ? 1e-6 // some default value if no envelope exists
            : options.envelopes[0].thickness; // use envelope thickness if available
    const double target_max_amips = options.target_max_amips;

    auto compute_target_edge_length =
        [target_edge_length,
         target_max_amips,
         min_edge_length,
         target_edge_length_attribute,
         &mesh](const Eigen::MatrixXd& P, const std::vector<Tuple>& neighs) -> Eigen::VectorXd {
        auto target_edge_length_accessor =
            mesh->create_accessor(target_edge_length_attribute.as<double>());

        assert(P.rows() == 1); // rows --> attribute dimension
        assert(!neighs.empty());
        assert(P.cols() == neighs.size());
        const double current_target_edge_length =
            target_edge_length_accessor.const_scalar_attribute(neighs[0]);
        const double max_amips = P.maxCoeff();

        double new_target_edge_length = current_target_edge_length;
        if (max_amips > target_max_amips) {
            new_target_edge_length *= 0.5;
        } else {
            new_target_edge_length *= 1.5;
        }
        new_target_edge_length =
            std::min(new_target_edge_length, target_edge_length); // upper bound
        new_target_edge_length = std::max(new_target_edge_length, min_edge_length); // lower bound

        return Eigen::VectorXd::Constant(1, new_target_edge_length);
    };
    auto target_edge_length_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            target_edge_length_attribute,
            amips_attribute,
            compute_target_edge_length);

    //// Example for some other target edge length
    // auto compute_target_edge_length = [](const Eigen::MatrixX<Rational>& P) -> Eigen::VectorXd {
    //    assert(P.cols() == 2); // cols --> number of neighbors
    //    assert(P.rows() == 2 || P.rows() == 3); // rows --> attribute dimension
    //    const double x_avg = 0.5 * (P(0, 0) + P(0, 1)).to_double();
    //    const double target_length = x_avg * x_avg;
    //    return Eigen::VectorXd::Constant(1, target_length);
    //};
    // auto target_edge_length_update =
    //    std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, Rational>>(
    //        target_edge_length_attribute,
    //        rpt_attribute,
    //        compute_target_edge_length);
    // target_edge_length_update->run_on_all();


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
    // default transfer
    auto pass_through_attributes = base::get_attributes(cache, *mesh, options.pass_through);
    pass_through_attributes.push_back(edge_length_attribute);
    pass_through_attributes.push_back(amips_attribute);
    pass_through_attributes.push_back(target_edge_length_attribute);

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

    auto propagate_to_child_position =
        [](const Eigen::MatrixX<Rational>& P) -> Eigen::VectorX<Rational> { return P; };

    auto propagate_to_parent_position =
        [](const Eigen::MatrixX<Rational>& P) -> Eigen::VectorX<Rational> {
        assert(P.cols() == 1);
        return P.col(0);
    };
    using MeshConstrainPair = ProjectOperation::MeshConstrainPair;

    auto envelope_invariant = std::make_shared<InvariantCollection>(*mesh);
    std::vector<std::shared_ptr<SingleAttributeTransferStrategy<Rational, Rational>>>
        update_child_positon, update_parent_positon;
    std::vector<std::shared_ptr<Mesh>> envelopes;
    std::vector<MeshConstrainPair> mesh_constraint_pairs;

    std::vector<std::shared_ptr<Mesh>> multimesh_meshes;

    assert(options.envelopes.size() == 4); // four kind of envelopes in tetwild [surface_mesh,
                                           // open_boudnary, nonmanifold_edges, is_boundary(bbox)]
    // TODO: add nonmanifold vertex point mesh

    for (const auto& v : options.envelopes) {
        auto envelope = cache.read_mesh(v.geometry.mesh);
        if (envelope == nullptr) {
            wmtk::logger().info("TetWild: no {} mesh for this mesh", v.geometry.mesh);
            continue;
        }
        envelopes.emplace_back(envelope);

        auto constrained = base::get_attributes(cache, *mesh, v.constrained_position);
        multimesh_meshes.push_back(constrained.front().mesh().shared_from_this());
        assert(constrained.size() == 1);
        pass_through_attributes.emplace_back(constrained.front());

        auto envelope_position_handle =
            envelope->get_attribute_handle<Rational>(v.geometry.position, PrimitiveType::Vertex);

        mesh_constraint_pairs.emplace_back(envelope_position_handle, constrained.front());

        envelope_invariant->add(std::make_shared<EnvelopeInvariant>(
            envelope_position_handle,
            v.thickness * bbdiag,
            constrained.front()));

        update_parent_positon.emplace_back(
            std::make_shared<SingleAttributeTransferStrategy<Rational, Rational>>(
                pt_attribute,
                constrained.front(),
                propagate_to_parent_position));

        update_child_positon.emplace_back(
            std::make_shared<SingleAttributeTransferStrategy<Rational, Rational>>(
                constrained.front(),
                pt_attribute,
                propagate_to_child_position));
    }

    //////////////////////////////////
    // collapse transfer
    //////////////////////////////////
    auto clps_strat = std::make_shared<CollapseNewAttributeStrategy<Rational>>(pt_attribute);
    clps_strat->set_simplex_predicate(BasicSimplexPredicate::IsInterior);
    // clps_strat->set_strategy(CollapseBasicStrategy::Default);
    clps_strat->set_strategy(CollapseBasicStrategy::CopyOther);


    //////////////////////////////////


    //////////////////////////////////
    // Invariants
    //////////////////////////////////

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
    auto interior_face = std::make_shared<InteriorSimplexInvariant>(*mesh, PrimitiveType::Triangle);

    for (const auto& em : multimesh_meshes) {
        interior_edge->add_boundary(*em);
        interior_face->add_boundary(*em);
    }

    auto valence_3 = std::make_shared<EdgeValenceInvariant>(*mesh, 3);
    auto valence_4 = std::make_shared<EdgeValenceInvariant>(*mesh, 4);
    auto swap44_energy_before =
        std::make_shared<Swap44EnergyBeforeInvariant>(*mesh, pt_attribute.as<Rational>());
    auto swap44_2_energy_before =
        std::make_shared<Swap44_2EnergyBeforeInvariant>(*mesh, pt_attribute.as<Rational>());
    auto swap32_energy_before =
        std::make_shared<Swap32EnergyBeforeInvariant>(*mesh, pt_attribute.as<Rational>());
    auto swap23_energy_before =
        std::make_shared<Swap23EnergyBeforeInvariant>(*mesh, pt_attribute.as<Rational>());

    auto invariant_separate_substructures =
        std::make_shared<invariants::SeparateSubstructuresInvariant>(*mesh);


    //////////////////////////////////
    // Creation of the 4 ops
    //////////////////////////////////
    std::vector<std::shared_ptr<Operation>> ops;
    std::vector<std::string> ops_name;

    //////////////////////////////////
    // 0) Rounding
    //////////////////////////////////
    auto rounding_pt_attribute = mesh->get_attribute_handle_typed<Rational>(
        options.attributes.position,
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

    split->set_new_attribute_strategy(pt_attribute);
    for (const auto& attr : pass_through_attributes) {
        split->set_new_attribute_strategy(attr);
    }

    split->add_transfer_strategy(amips_update);
    split->add_transfer_strategy(edge_length_update);
    // split->add_transfer_strategy(target_edge_length_update);
    for (auto& s : update_child_positon) {
        split->add_transfer_strategy(s);
    }

    auto split_then_round = std::make_shared<OperationSequence>(*mesh);
    split_then_round->add_operation(split);
    split_then_round->add_operation(rounding);

    ops.emplace_back(split_then_round);
    ops_name.emplace_back("split");

    // ops.emplace_back(split);
    // ops_name.emplace_back("split");

    ops.emplace_back(rounding);
    ops_name.emplace_back("rounding");

    //////////////////////////////////
    // 2) EdgeCollapse
    //////////////////////////////////
    auto collapse = std::make_shared<EdgeCollapse>(*mesh);
    // before
    collapse->add_invariant(todo_smaller);
    collapse->add_invariant(invariant_separate_substructures);
    collapse->add_invariant(link_condition);

    // after
    collapse->add_invariant(inversion_invariant);
    collapse->add_invariant(function_invariant);
    collapse->add_invariant(envelope_invariant);

    // update
    collapse->add_transfer_strategy(amips_update);
    collapse->add_transfer_strategy(edge_length_update);
    // collapse->add_transfer_strategy(target_edge_length_update);


    collapse->set_new_attribute_strategy(pt_attribute, clps_strat);
    for (const auto& attr : pass_through_attributes) {
        collapse->set_new_attribute_strategy(attr);
    }
    for (auto& s : update_child_positon) {
        collapse->add_transfer_strategy(s);
    }

    // auto proj_collapse = std::make_shared<ProjectOperation>(collapse, mesh_constraint_pairs);
    // proj_collapse->set_priority(short_edges_first);

    // proj_collapse->add_invariant(todo_smaller);
    // proj_collapse->add_invariant(envelope_invariant);
    // proj_collapse->add_invariant(inversion_invariant);
    // proj_collapse->add_invariant(function_invariant);

    // proj_collapse->add_transfer_strategy(amips_update);
    // proj_collapse->add_transfer_strategy(edge_length_update);
    // for (auto& s : update_parent_positon) {
    //     proj_collapse->add_transfer_strategy(s);
    // }
    // for (auto& s : update_child_positon) {
    //     proj_collapse->add_transfer_strategy(s);
    // }
    // proj_collapse->add_transfer_strategy(target_edge_length_update);

    // auto proj_collapse_then_round = std::make_shared<OperationSequence>(*mesh);
    // proj_collapse_then_round->add_operation(proj_collapse);
    // proj_collapse_then_round->add_operation(rounding);

    auto collapse_then_round = std::make_shared<OperationSequence>(*mesh);
    collapse_then_round->add_operation(collapse);
    collapse_then_round->add_operation(rounding);

    ops.emplace_back(collapse_then_round);
    ops_name.emplace_back("collapse");
    ops.emplace_back(rounding);
    ops_name.emplace_back("rounding");

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
        op.add_invariant(inversion_invariant);
        // op.add_invariant(function_invariant);

        op.add_transfer_strategy(amips_update);
        op.add_transfer_strategy(edge_length_update);
        // op.add_transfer_strategy(target_edge_length_update);
        for (auto& s : update_child_positon) {
            op.add_transfer_strategy(s);
        }

        collapse.add_invariant(invariant_separate_substructures);
        collapse.add_invariant(link_condition);


        collapse.set_new_attribute_strategy(pt_attribute, CollapseBasicStrategy::CopyOther);
        split.set_new_attribute_strategy(pt_attribute);

        // this might not be necessary
        for (auto& s : update_child_positon) {
            collapse.add_transfer_strategy(s);
            split.add_transfer_strategy(s);
        }


        for (const auto& attr : pass_through_attributes) {
            split.set_new_attribute_strategy(attr);
            collapse.set_new_attribute_strategy(attr);
        }
    };


    if (mesh->top_simplex_type() == PrimitiveType::Triangle) {
        auto swap = std::make_shared<TriEdgeSwap>(*mesh);
        setup_swap(*swap, swap->collapse(), swap->split(), interior_edge);

        ops.push_back(swap);
        ops_name.push_back("swap");

        ops.emplace_back(rounding);
        ops_name.emplace_back("rounding");

    } else if (mesh->top_simplex_type() == PrimitiveType::Tetrahedron) {
        // 3 - 1 - 1) TetEdgeSwap 4-4 1
        auto swap44 = std::make_shared<TetEdgeSwap>(*mesh, 0);
        setup_swap(*swap44, swap44->collapse(), swap44->split(), interior_edge);
        swap44->add_invariant(valence_4); // extra edge valance invariant
        swap44->add_invariant(swap44_energy_before); // check energy before swap

        ops.push_back(swap44);
        ops_name.push_back("swap44");
        ops.emplace_back(rounding);
        ops_name.emplace_back("rounding");

        // 3 - 1 - 2) TetEdgeSwap 4-4 2
        auto swap44_2 = std::make_shared<TetEdgeSwap>(*mesh, 1);
        setup_swap(*swap44_2, swap44_2->collapse(), swap44_2->split(), interior_edge);
        swap44_2->add_invariant(valence_4); // extra edge valance invariant
        swap44_2->add_invariant(swap44_2_energy_before); // check energy before swap
        ops.push_back(swap44_2);
        ops_name.push_back("swap44_2");
        ops.emplace_back(rounding);
        ops_name.emplace_back("rounding");

        // 3 - 2) TetEdgeSwap 3-2
        auto swap32 = std::make_shared<TetEdgeSwap>(*mesh, 0);
        setup_swap(*swap32, swap32->collapse(), swap32->split(), interior_edge);
        swap32->add_invariant(valence_3); // extra edge valance invariant
        swap32->add_invariant(swap32_energy_before); // check energy before swap
        ops.push_back(swap32);
        ops_name.push_back("swap32");
        ops.emplace_back(rounding);
        ops_name.emplace_back("rounding");

        // 3 - 3) TetFaceSwap 2-3
        auto swap23 = std::make_shared<TetFaceSwap>(*mesh);
        setup_swap(*swap23, swap23->collapse(), swap23->split(), interior_face, false);
        swap23->add_invariant(swap23_energy_before); // check energy before swap
        ops.push_back(swap23);
        ops_name.push_back("swap23");
        ops.emplace_back(rounding);
        ops_name.emplace_back("rounding");
    }

    // 4) Smoothing
    // auto energy =
    //     std::make_shared<function::LocalNeighborsSumFunction>(*mesh, pt_attribute, *amips);
    // auto smoothing = std::make_shared<OptimizationSmoothing>(energy);
    auto smoothing = std::make_shared<AMIPSOptimizationSmoothing>(*mesh, pt_attribute);
    smoothing->add_invariant(
        std::make_shared<RoundedInvariant>(*mesh, pt_attribute.as<Rational>()));
    smoothing->add_invariant(inversion_invariant);
    for (auto& s : update_child_positon) smoothing->add_transfer_strategy(s);

    auto proj_smoothing = std::make_shared<ProjectOperation>(smoothing, mesh_constraint_pairs);
    proj_smoothing->use_random_priority() = true;

    proj_smoothing->add_invariant(envelope_invariant);
    proj_smoothing->add_invariant(inversion_invariant);

    proj_smoothing->add_transfer_strategy(amips_update);
    proj_smoothing->add_transfer_strategy(edge_length_update);
    for (auto& s : update_parent_positon) {
        proj_smoothing->add_transfer_strategy(s);
    }

    for (auto& s : update_child_positon) {
        proj_smoothing->add_transfer_strategy(s);
    }
    // proj_smoothing->add_transfer_strategy(target_edge_length_update);

    ops.push_back(proj_smoothing);
    ops_name.push_back("smoothing");
    ops.emplace_back(rounding);
    ops_name.emplace_back("rounding");

    write(
        mesh,
        paths.output_dir,
        options.output,
        options.attributes.position,
        0,
        options.intermediate_output);

    //////////////////////////////////
    // Running all ops in order n times
    Scheduler scheduler;
    int64_t success = 10;

    //////////////////////////////////
    // preprocessing

    SchedulerStats pre_stats;

    while (success > 0) {
        pre_stats = scheduler.run_operation_on_all(*collapse_then_round);
        logger().info(
            "Executed {}, {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, "
            "executing: {}",
            "collapse",
            pre_stats.number_of_performed_operations(),
            pre_stats.number_of_successful_operations(),
            pre_stats.number_of_failed_operations(),
            pre_stats.collecting_time,
            pre_stats.sorting_time,
            pre_stats.executing_time);

        success = pre_stats.number_of_successful_operations();

        // verbose logger, can be removed
        int64_t unrounded = 0;
        for (const auto& v : mesh->get_all(PrimitiveType::Vertex)) {
            const auto p = pt_accessor.vector_attribute(v);
            for (int64_t d = 0; d < 3; ++d) {
                if (!p[d].is_rounded()) {
                    ++unrounded;
                    break;
                }
            }
        }

        logger().info("Mesh has {} unrounded vertices", unrounded);
    }

    int iii = 0;
    bool is_double = false;
    for (int64_t i = 0; i < options.passes; ++i) {
        logger().info("Pass {}", i);
        SchedulerStats pass_stats;
        int jj = 0;
        for (auto& op : ops) {
            int success = 10;
            while (success > 0) {
                auto stats = scheduler.run_operation_on_all(*op);
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
                    for (int64_t d = 0; d < 3; ++d) {
                        if (!p[d].is_rounded()) {
                            ++unrounded;
                            break;
                        }
                    }
                }

                logger().info("Mesh has {} unrounded vertices", unrounded);
            }
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
            paths.output_dir,
            options.output,
            options.attributes.position,
            i + 1,
            options.intermediate_output);

        assert(mesh->is_connectivity_valid());

        // compute max energy
        double max_energy = std::numeric_limits<double>::lowest();
        double min_energy = std::numeric_limits<double>::max();
        for (const auto& t : mesh->get_all(mesh->top_simplex_type())) {
            // double e = amips->get_value(simplex::Simplex(mesh->top_simplex_type(), t));
            double e = amips_accessor.scalar_attribute(t);
            max_energy = std::max(max_energy, e);
            min_energy = std::min(min_energy, e);
        }

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
        logger().info("Max AMIPS Energy: {}, Min AMIPS Energy: {}", max_energy, min_energy);


        // stop at good quality
        if (max_energy <= target_max_amips && is_double) break;
    }

    // output
    cache.write_mesh(*mesh, options.output);
}
} // namespace wmtk::components
