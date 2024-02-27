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

#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/OptimizationSmoothing.hpp>
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
#include <wmtk/invariants/MultiMeshMapValidInvariant.hpp>
#include <wmtk/invariants/NoBoundaryCollapseToInteriorInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>

#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>

#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>


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
        if (mesh->top_simplex_type() == PrimitiveType::Face) {
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
        mesh->get_attribute_handle<double>(options.attributes.position, PrimitiveType::Vertex);
    auto pt_accessor = mesh->create_accessor(pt_attribute.as<double>());


    //////////////////////////////////
    // Storing edge lengths
    auto edge_length_attribute =
        mesh->register_attribute<double>("edge_length", PrimitiveType::Edge, 1);
    auto edge_length_accessor = mesh->create_accessor(edge_length_attribute.as<double>());
    // Edge length update
    auto compute_edge_length = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        assert(P.cols() == 2);
        assert(P.rows() == 2 || P.rows() == 3);
        return Eigen::VectorXd::Constant(1, (P.col(0) - P.col(1)).norm());
    };
    auto edge_length_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            edge_length_attribute,
            pt_attribute,
            compute_edge_length);
    edge_length_update->run_on_all();


    //////////////////////////////////
    // computng bbox diagonal
    Eigen::VectorXd bmin(mesh->top_cell_dimension());
    bmin.setConstant(std::numeric_limits<double>::max());
    Eigen::VectorXd bmax(mesh->top_cell_dimension());
    bmax.setConstant(std::numeric_limits<double>::min());

    const auto vertices = mesh->get_all(PrimitiveType::Vertex);
    for (const auto& v : vertices) {
        const auto p = pt_accessor.vector_attribute(v);
        for (int64_t d = 0; d < bmax.size(); ++d) {
            bmin[d] = std::min(bmin[d], p[d]);
            bmax[d] = std::max(bmax[d], p[d]);
        }
    }

    const double bbdiag = (bmax - bmin).norm();
    const double target_edge_length = options.target_edge_length * bbdiag;

    //////////////////////////////////
    // default transfer
    auto pass_through_attributes = base::get_attributes(cache, *mesh, options.pass_through);
    pass_through_attributes.push_back(edge_length_attribute);

    //////////////////////////////////
    // Lambdas for priority
    //////////////////////////////////
    auto long_edges_first = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return std::vector<double>({-edge_length_accessor.scalar_attribute(s.tuple())});
    };
    auto short_edges_first = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return std::vector<double>({edge_length_accessor.scalar_attribute(s.tuple())});
    };


    //////////////////////////////////
    // envelopes
    //////////////////////////////////

    auto propagate_to_child_position = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        return P;
    };

    auto propagate_to_parent_position = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        assert(P.cols() == 1);
        return P.col(0);
    };
    using MeshConstrainPair = ProjectOperation::MeshConstrainPair;

    auto envelope_invariant = std::make_shared<InvariantCollection>(*mesh);
    std::vector<std::shared_ptr<SingleAttributeTransferStrategy<double, double>>>
        update_child_positon, update_parent_positon;
    std::vector<std::shared_ptr<Mesh>> envelopes;
    std::vector<MeshConstrainPair> mesh_constaint_pairs;

    std::vector<std::shared_ptr<Mesh>> multimesh_meshes;

    for (const auto& v : options.envelopes) {
        auto envelope = cache.read_mesh(v.geometry.mesh);
        envelopes.emplace_back(envelope);

        auto constrained = base::get_attributes(cache, *mesh, v.constrained_position);
        multimesh_meshes.push_back(constrained.front().mesh().shared_from_this());
        assert(constrained.size() == 1);
        pass_through_attributes.emplace_back(constrained.front());

        auto envelope_position_handle =
            envelope->get_attribute_handle<double>(v.geometry.position, PrimitiveType::Vertex);

        mesh_constaint_pairs.emplace_back(envelope_position_handle, constrained.front());

        envelope_invariant->add(std::make_shared<EnvelopeInvariant>(
            envelope_position_handle,
            v.thickness * bbdiag,
            constrained.front()));

        update_child_positon.emplace_back(
            std::make_shared<SingleAttributeTransferStrategy<double, double>>(
                constrained.front(),
                pt_attribute,
                propagate_to_child_position));


        update_parent_positon.emplace_back(
            std::make_shared<SingleAttributeTransferStrategy<double, double>>(
                pt_attribute,
                constrained.front(),
                propagate_to_parent_position));
    }

    //////////////////////////////////
    // collapse transfer
    //////////////////////////////////
    auto clps_strat = std::make_shared<CollapseNewAttributeStrategy<double>>(pt_attribute);
    clps_strat->set_simplex_predicate(BasicSimplexPredicate::IsInterior);
    clps_strat->set_strategy(CollapseBasicStrategy::Default);


    //////////////////////////////////


    //////////////////////////////////
    // Invariants
    //////////////////////////////////

    auto inversion_invariant =
        std::make_shared<SimplexInversionInvariant>(*mesh, pt_attribute.as<double>());

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
        4.0 / 3.0 * target_edge_length);

    auto todo_smaller = std::make_shared<TodoSmallerInvariant>(
        *mesh,
        edge_length_attribute.as<double>(),
        4.0 / 5.0 * target_edge_length);

    auto interior_edge = std::make_shared<InteriorEdgeInvariant>(*mesh);
    auto interior_face = std::make_shared<InteriorSimplexInvariant>(*mesh, PrimitiveType::Face);

    for (const auto& em : multimesh_meshes) {
        interior_edge->add_boundary(*em);
        interior_face->add_boundary(*em);
    }

    auto valence_3 = std::make_shared<EdgeValenceInvariant>(*mesh, 3);
    auto valence_4 = std::make_shared<EdgeValenceInvariant>(*mesh, 4);

    auto invariant_mm_map = std::make_shared<MultiMeshMapValidInvariant>(*mesh);


    //////////////////////////////////
    // Creation of the 4 ops
    //////////////////////////////////
    std::vector<std::shared_ptr<Operation>> ops;
    std::vector<std::string> ops_name;

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

    split->add_transfer_strategy(edge_length_update);
    for (auto& s : update_child_positon) split->add_transfer_strategy(s);

    ops.emplace_back(split);
    ops_name.emplace_back("split");

    //////////////////////////////////
    // 2) EdgeCollapse
    //////////////////////////////////
    auto collapse = std::make_shared<EdgeCollapse>(*mesh);
    collapse->add_invariant(link_condition);
    collapse->add_invariant(inversion_invariant);
    collapse->add_invariant(invariant_mm_map);

    collapse->set_new_attribute_strategy(pt_attribute, clps_strat);
    for (const auto& attr : pass_through_attributes) {
        collapse->set_new_attribute_strategy(attr);
    }
    for (auto& s : update_child_positon) collapse->add_transfer_strategy(s);

    auto proj_collapse = std::make_shared<ProjectOperation>(collapse, mesh_constaint_pairs);
    proj_collapse->set_priority(short_edges_first);

    proj_collapse->add_invariant(todo_smaller);
    proj_collapse->add_invariant(envelope_invariant);
    proj_collapse->add_invariant(inversion_invariant);
    proj_collapse->add_invariant(function_invariant);

    proj_collapse->add_transfer_strategy(edge_length_update);
    for (auto& s : update_parent_positon) proj_collapse->add_transfer_strategy(s);

    ops.emplace_back(proj_collapse);
    ops_name.emplace_back("collapse");


    //////////////////////////////////
    // 3) Swap
    //////////////////////////////////

    auto setup_swap = [&](Operation& op,
                          EdgeCollapse& collapse,
                          EdgeSplit& split,
                          std::shared_ptr<Invariant> simplex_invariant) {
        op.set_priority(long_edges_first);

        op.add_invariant(simplex_invariant);
        op.add_invariant(inversion_invariant);
        op.add_invariant(function_invariant);

        op.add_transfer_strategy(edge_length_update);
        for (auto& s : update_child_positon) op.add_transfer_strategy(s);

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


    if (mesh->top_simplex_type() == PrimitiveType::Face) {
        auto swap = std::make_shared<TriEdgeSwap>(*mesh);
        setup_swap(*swap, swap->collapse(), swap->split(), interior_edge);
        ops.push_back(swap);
        ops_name.push_back("swap");
    } else if (mesh->top_simplex_type() == PrimitiveType::Tetrahedron) {
        // 3 - 1 - 1) TetEdgeSwap 4-4 1
        auto swap44 = std::make_shared<TetEdgeSwap>(*mesh, 0);
        setup_swap(*swap44, swap44->collapse(), swap44->split(), interior_edge);
        swap44->add_invariant(valence_4); // extra edge valance invariant
        ops.push_back(swap44);
        ops_name.push_back("swap44");

        // 3 - 1 - 2) TetEdgeSwap 4-4 2
        auto swap44_2 = std::make_shared<TetEdgeSwap>(*mesh, 1);
        setup_swap(*swap44_2, swap44_2->collapse(), swap44_2->split(), interior_edge);
        swap44_2->add_invariant(valence_4); // extra edge valance invariant
        ops.push_back(swap44_2);
        ops_name.push_back("swap44_2");

        // 3 - 2) TetEdgeSwap 3-2
        auto swap32 = std::make_shared<TetEdgeSwap>(*mesh, 0);
        setup_swap(*swap32, swap32->collapse(), swap32->split(), interior_edge);
        swap32->add_invariant(valence_3); // extra edge valance invariant
        ops.push_back(swap32);
        ops_name.push_back("swap32");

        // 3 - 3) TetFaceSwap 2-3
        // auto swap23 = std::make_shared<TetFaceSwap>(*mesh);
        // setup_swap(*swap23, swap23->collapse(), swap23->split(), interior_face);
        // ops.push_back(swap23);
        // ops_name.push_back("swap23");
    }

    // 4) Smoothing
    auto energy =
        std::make_shared<function::LocalNeighborsSumFunction>(*mesh, pt_attribute, *amips);
    auto smoothing = std::make_shared<OptimizationSmoothing>(energy);
    smoothing->add_invariant(inversion_invariant);
    for (auto& s : update_child_positon) smoothing->add_transfer_strategy(s);

    auto proj_smoothing = std::make_shared<ProjectOperation>(smoothing, mesh_constaint_pairs);
    proj_smoothing->use_random_priority() = true;

    proj_smoothing->add_invariant(envelope_invariant);
    proj_smoothing->add_invariant(inversion_invariant);

    proj_smoothing->add_transfer_strategy(edge_length_update);
    for (auto& s : update_parent_positon) proj_smoothing->add_transfer_strategy(s);
    ops.push_back(proj_smoothing);
    ops_name.push_back("smoothing");

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
    int iii = 0;
    for (int64_t i = 0; i < options.passes; ++i) {
        logger().info("Pass {}", i);
        SchedulerStats pass_stats;
        int jj = 0;
        for (auto& op : ops) {
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
            ++jj;

            // if (!mesh->is_connectivity_valid()) {
            //     std::cout << "invalid connectivity after " + ops_name[jj] << std::endl;
            //     throw std::runtime_error("input mesh for wildmeshing connectivity invalid");
            // }
        }

        logger().info(
            "Executed {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, executing: {}",
            pass_stats.number_of_performed_operations(),
            pass_stats.number_of_successful_operations(),
            pass_stats.number_of_failed_operations(),
            pass_stats.collecting_time,
            pass_stats.sorting_time,
            pass_stats.executing_time);

        // if (!mesh->is_connectivity_valid()) {
        //     std::cout << "invalid connectivity before consolidate" << std::endl;
        //     throw std::runtime_error("input mesh for wildmeshing connectivity invalid");
        // }

        multimesh::consolidate(*mesh);

        // if (!mesh->is_connectivity_valid()) {
        //     std::cout << "invalid connectivity after consolidate" << std::endl;
        //     throw std::runtime_error("input mesh for wildmeshing connectivity invalid");
        // }

        write(
            mesh,
            paths.output_dir,
            options.output,
            options.attributes.position,
            i + 1,
            options.intermediate_output);

        assert(mesh->is_connectivity_valid());
    }


    // output
    cache.write_mesh(*mesh, options.output);
}
} // namespace wmtk::components
