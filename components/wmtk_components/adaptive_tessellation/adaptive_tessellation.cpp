#include "adaptive_tessellation.hpp"

#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/operations/OptimizationSmoothing.hpp>
#include <wmtk/simplex/Simplex.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>

#include <wmtk/components/adaptive_tessellation/operations/internal/ATOptions.hpp>

#include <wmtk/components/adaptive_tessellation/function/simplex/PerTriangleAnalyticalIntegral.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/PerTriangleTextureIntegralAccuracyFunction.hpp>
#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/function/PerSimplexFunction.hpp>
#include <wmtk/function/simplex/AMIPS.hpp>
#include <wmtk/function/simplex/TriangleAMIPS.hpp>

#include <wmtk/invariants/FunctionInvariant.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>

#include <wmtk/components/adaptive_tessellation/image/Image.hpp>
#include <wmtk/components/adaptive_tessellation/image/Sampling.hpp>

namespace wmtk::components {
using namespace operations;
using namespace function;
using namespace invariants;
namespace AT = wmtk::components::adaptive_tessellation;

namespace {
void write(
    const std::shared_ptr<Mesh>& mesh,
    const std::string& name,
    const int64_t index,
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

void at(const nlohmann::json& j)
{
    //////////////////////////////////
    // Load mesh from settings
    ATOptions options = j.get<ATOptions>();
    const std::filesystem::path& file = options.input;
    std::shared_ptr<Mesh> mesh = read_mesh(file, options.planar);

    //////////////////////////////////
    // Storing edge lengths
    auto edge_length_attribute =
        mesh->register_attribute<double>("edge_length", PrimitiveType::Edge, 1);
    auto edge_length_accessor = mesh->create_accessor(edge_length_attribute.as<double>());


    //////////////////////////////////
    // Retriving vertices
    auto pt_attribute = mesh->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto pt_accessor = mesh->create_accessor(pt_attribute.as<double>());

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
        for (int64_t d = 0; d < bmax.size(); ++d) {
            bmin[d] = std::min(bmin[d], p[d]);
            bmax[d] = std::max(bmax[d], p[d]);
        }
    }

    const double bbdiag = (bmax - bmin).norm();
    const double target_edge_length = options.target_edge_length * bbdiag;

    //////////////////////////////////
    // Lambdas for priority
    auto long_edges_first = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return std::vector<double>({-edge_length_accessor.scalar_attribute(s.tuple())});
    };
    auto short_edges_first = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return std::vector<double>({edge_length_accessor.scalar_attribute(s.tuple())});
    };

    //////////////////////////////////
    // // Energy to optimize
    // std::shared_ptr<wmtk::function::PerSimplexFunction> amips =
    //     std::make_shared<wmtk::function::AMIPS>(*mesh, pt_attribute);


    std::shared_ptr<wmtk::function::TriangleAMIPS> amips =
        std::make_shared<wmtk::function::TriangleAMIPS>(*mesh, pt_attribute);

    std::array<std::shared_ptr<image::Image>, 3> images = {
        {std::make_shared<image::Image>(500, 500),
         std::make_shared<image::Image>(500, 500),
         std::make_shared<image::Image>(500, 500)}};
    auto u = [](const double& u, [[maybe_unused]] const double& v) -> double { return u; };
    auto v = []([[maybe_unused]] const double& u, const double& v) -> double { return v; };
    std::function<double(double, double)> height_function =
        [](const double& u, [[maybe_unused]] const double& v) -> double {
        // return u * u + v * v;
        return sin(2 * M_PI * u) * cos(2 * M_PI * v);
    };
    images[0]->set(u);
    images[1]->set(v);
    images[2]->set(height_function);
    std::shared_ptr<wmtk::function::PerTriangleTextureIntegralAccuracyFunction> texture =
        std::make_shared<wmtk::function::PerTriangleTextureIntegralAccuracyFunction>(
            *mesh,
            pt_attribute,
            images);

    std::array<std::shared_ptr<image::SamplingAnalyticFunction>, 3> funcs = {
        {std::make_shared<image::SamplingAnalyticFunction>(
             image::SamplingAnalyticFunction_FunctionType::Linear,
             1,
             0,
             0.),
         std::make_shared<image::SamplingAnalyticFunction>(
             image::SamplingAnalyticFunction_FunctionType::Linear,
             0,
             1,
             0.),
         std::make_shared<image::SamplingAnalyticFunction>(
             image::SamplingAnalyticFunction_FunctionType::Periodic,
             2,
             2,
             1.)}

    };
    std::shared_ptr<wmtk::function::PerTriangleAnalyticalIntegral> accuracy =
        std::make_shared<wmtk::function::PerTriangleAnalyticalIntegral>(*mesh, pt_attribute, funcs);


    opt_logger().set_level(spdlog::level::level_enum::critical);


    //////////////////////////////////
    // Creation of the 4 ops
    std::vector<std::shared_ptr<Operation>> ops;


    // 1) EdgeSplit
    auto split = std::make_shared<EdgeSplit>(*mesh);
    split->add_invariant(std::make_shared<TodoLargerInvariant>(
        *mesh,
        edge_length_attribute.as<double>(),
        4.0 / 3.0 * target_edge_length));
    split->set_priority(long_edges_first);

    split->set_new_attribute_strategy(edge_length_attribute);
    split->set_new_attribute_strategy(pt_attribute);

    split->add_transfer_strategy(edge_length_update);
    ops.emplace_back(split);


    // 2) EdgeCollapse
    auto collapse = std::make_shared<EdgeCollapse>(*mesh);
    collapse->add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(*mesh));
    collapse->add_invariant(std::make_shared<InteriorEdgeInvariant>(*mesh));
    collapse->add_invariant(
        std::make_shared<SimplexInversionInvariant>(*mesh, pt_attribute.as<double>()));
    collapse->add_invariant(
        std::make_shared<FunctionInvariant>(mesh->top_simplex_type(), accuracy));
    collapse->add_invariant(std::make_shared<TodoSmallerInvariant>(
        *mesh,
        edge_length_attribute.as<double>(),
        4.0 / 5.0 * target_edge_length));
    collapse->set_priority(short_edges_first);

    auto clps_strat = std::make_shared<CollapseNewAttributeStrategy<double>>(pt_attribute);
    clps_strat->set_simplex_predicate(BasicSimplexPredicate::IsInterior);
    clps_strat->set_strategy(CollapseBasicStrategy::Default);

    collapse->set_new_attribute_strategy(pt_attribute, clps_strat);
    collapse->set_new_attribute_strategy(edge_length_attribute);

    collapse->add_transfer_strategy(edge_length_update);
    ops.emplace_back(collapse);


    // 3) TriEdgeSwap
    if (mesh->top_simplex_type() == PrimitiveType::Face) {
        auto swap = std::make_shared<TriEdgeSwap>(*mesh);
        swap->collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(*mesh));
        swap->add_invariant(std::make_shared<InteriorEdgeInvariant>(*mesh));
        swap->add_invariant(
            std::make_shared<SimplexInversionInvariant>(*mesh, pt_attribute.as<double>()));
        swap->add_invariant(
            std::make_shared<FunctionInvariant>(mesh->top_simplex_type(), accuracy));
        swap->set_priority(long_edges_first);

        swap->collapse().set_new_attribute_strategy(edge_length_attribute);
        swap->split().set_new_attribute_strategy(edge_length_attribute);

        swap->split().set_new_attribute_strategy(pt_attribute);
        swap->collapse().set_new_attribute_strategy(pt_attribute, CollapseBasicStrategy::CopyOther);

        swap->add_transfer_strategy(edge_length_update);

        ops.push_back(swap);
    } else // if (mesh->top_simplex_type() == PrimitiveType::Face) {
    {
        throw std::runtime_error("unsupported");
    }

    // 4) Smoothing
    auto energy =
        std::make_shared<wmtk::function::LocalNeighborsSumFunction>(*mesh, pt_attribute, *accuracy);
    ops.emplace_back(std::make_shared<OptimizationSmoothing>(energy));
    ops.back()->add_invariant(
        std::make_shared<SimplexInversionInvariant>(*mesh, pt_attribute.as<double>()));
    ops.back()->add_invariant(std::make_shared<InteriorVertexInvariant>(*mesh));
    ops.back()->add_transfer_strategy(edge_length_update);
    ops.back()->use_random_priority() = true;


    //////////////////////////////////
    // Running all ops in order n times
    Scheduler scheduler;
    for (int64_t i = 0; i < options.passes; ++i) {
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