#include "wildmeshing.hpp"

#include "WildmeshingOptions.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/utils/Logger.hpp>

#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/OptimizationSmoothing.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>

#include <wmtk/operations/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/tri_mesh/BasicSplitNewAttributeStrategy.hpp>

#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/function/PerSimplexFunction.hpp>
#include <wmtk/function/simplex/AMIPS.hpp>

#include <wmtk/invariants/FunctionInvariant.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/invariants/TriangleInversionInvariant.hpp>

#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>


namespace wmtk::components {

using namespace operations;
using namespace operations::composite;
using namespace function;
using namespace invariants;

namespace {
void write(
    const std::shared_ptr<Mesh>& mesh,
    const std::string& name,
    const long index,
    const bool intermediate_output)
{
    if (intermediate_output) {
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
    }
}
} // namespace

void wildmeshing(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files)
{
    //////////////////////////////////
    // Load mesh from settings
    WildmeshingOptions options = j.get<WildmeshingOptions>();
    const std::filesystem::path& file = options.input;
    std::shared_ptr<Mesh> mesh = read_mesh(file, options.planar);

    //////////////////////////////////
    // Storing edge lengths
    auto edge_length_attribute =
        mesh->register_attribute<double>("edge_length", PrimitiveType::Edge, 1);
    auto edge_length_accessor = mesh->create_accessor(edge_length_attribute);
    // Edge length is half after split
    auto& e_length_split_strat = edge_length_attribute.trimesh_standard_split_strategy();
    e_length_split_strat.set_split_strategy([](const Eigen::VectorXd& v) {
        std::array<Eigen::VectorXd, 2> res;
        res[0] = res[1] = v / 2.0;
        return res;
    });
    // auto& e_length_split_strat = edge_length_attribute.trimesh_standard_split_strategy();
    // e_length_split_strat.set_split_strategy([](const Eigen::VectorXd& v) {
    //     std::array<Eigen::VectorXd, 2> res;
    //     res[0] = res[1] = v / 2.0;
    //     return res;
    // });
    // TODO transfer of edge length

    //////////////////////////////////
    // Retriving vertices
    auto pt_attribute = mesh->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto pt_accessor = mesh->create_accessor(pt_attribute);

    //////////////////////////////////
    // computing edge lengths
    const auto edges = mesh->get_all(PrimitiveType::Edge);
    for (const auto& e : edges) {
        const auto p0 = pt_accessor.vector_attribute(e);
        const auto p1 = pt_accessor.vector_attribute(mesh->switch_vertex(e));

        edge_length_accessor.scalar_attribute(e) = (p0 - p1).norm();
    }

    //////////////////////////////////
    // computng bbox diagonal
    Eigen::VectorXd bmin(options.planar ? 2 : 3);
    bmin.setConstant(std::numeric_limits<double>::max());
    Eigen::VectorXd bmax(options.planar ? 2 : 3);
    bmax.setConstant(std::numeric_limits<double>::min());

    const auto vertices = mesh->get_all(PrimitiveType::Vertex);
    for (const auto& v : vertices) {
        const auto p = pt_accessor.vector_attribute(v);
        for (long d = 0; d < bmax.size(); ++d) {
            bmin[d] = std::min(bmin[d], p[d]);
            bmax[d] = std::max(bmax[d], p[d]);
        }
    }

    const double bbdiag = (bmax - bmin).norm();
    const double target_edge_length = options.target_edge_length * bbdiag;

    //////////////////////////////////
    // Lambdas for priority
    auto long_edges_first = [&](const Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return std::vector<double>({edge_length_accessor.scalar_attribute(s.tuple())});
    };
    auto short_edges_first = [&](const Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return std::vector<double>({-edge_length_accessor.scalar_attribute(s.tuple())});
    };

    //////////////////////////////////
    // Energy to optimize
    std::shared_ptr<function::PerSimplexFunction> amips =
        std::make_shared<AMIPS>(*mesh, pt_attribute);


    opt_logger().set_level(spdlog::level::level_enum::critical);


    //////////////////////////////////
    // Creation of the 4 ops
    std::vector<std::shared_ptr<Operation>> ops;


    // 1) EdgeSplit
    ops.emplace_back(std::make_shared<EdgeSplit>(*mesh));
    ops.back()->add_invariant(std::make_shared<TodoLargerInvariant>(
        *mesh,
        edge_length_attribute,
        4.0 / 3.0 * target_edge_length));
    ops.back()->set_priority(long_edges_first);

    // 2) EdgeCollapse
    // ops.emplace_back(std::make_shared<EdgeCollapse>(*mesh));
    // ops.back()->add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(*mesh));
    // ops.back()->add_invariant(std::make_shared<InteriorEdgeInvariant>(*mesh));
    // ops.back()->add_invariant(std::make_shared<TriangleInversionInvariant>(*mesh, pt_attribute));
    // ops.back()->add_invariant(std::make_shared<FunctionInvariant>(mesh->top_simplex_type(),
    // amips)); ops.back()->add_invariant(std::make_shared<TodoLargerInvariant>(
    //     *mesh,
    //     edge_length_attribute,
    //     4.0 / 5.0 * target_edge_length));
    // ops.back()->set_priority(short_edges_first);

    // 3) TriEdgeSwap
    // if (mesh->top_simplex_type() == PrimitiveType::Face) {
    //     auto op = std::make_shared<TriEdgeSwap>(*mesh);
    //     op->collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(*mesh));
    //     op->add_invariant(std::make_shared<InteriorEdgeInvariant>(*mesh));
    //     op->collapse().add_invariant(
    //         std::make_shared<TriangleInversionInvariant>(*mesh, pt_attribute));
    //     op->add_invariant(std::make_shared<FunctionInvariant>(mesh->top_simplex_type(), amips));
    //     op->set_priority(long_edges_first);
    //     ops.push_back(op);
    // } else // if (mesh->top_simplex_type() == PrimitiveType::Face) {
    // {
    //     throw std::runtime_error("unsupported");
    // }

    // 4) Smoothing
    // auto energy =
    //     std::make_shared<function::LocalNeighborsSumFunction>(*mesh, pt_attribute, *amips);
    // ops.emplace_back(std::make_shared<OptimizationSmoothing>(energy));
    // ops.back()->add_invariant(std::make_shared<TriangleInversionInvariant>(*mesh, pt_attribute));
    // ops.back()->add_invariant(std::make_shared<InteriorVertexInvariant>(*mesh));


    write(mesh, options.filename, 0, options.intermediate_output);

    //////////////////////////////////
    // Running all ops in order n times
    Scheduler scheduler;
    for (long i = 0; i < options.passes; ++i) {
        logger().info("Pass {}", i);
        SchedulerStats pass_stats;
        for (auto& op : ops) pass_stats += scheduler.run_operation_on_all(*op);

        logger().info(
            "Executed {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, executing: {}",
            pass_stats.number_of_performed_operations(),
            pass_stats.number_of_successful_operations(),
            pass_stats.number_of_failed_operations(),
            pass_stats.collecting_time,
            pass_stats.sorting_time,
            pass_stats.executing_time);

        write(mesh, options.filename, i + 1, options.intermediate_output);
    }
}
} // namespace wmtk::components