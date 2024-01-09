#include "adaptive_tessellation.hpp"

#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/io/HDF5Reader.hpp>
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

#include <wmtk/components/adaptive_tessellation/function/utils/AnalyticalFunctionTriangleQuadrature.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/ThreeChannelPositionMapEvaluator.hpp>

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
        wmtk::io::ParaviewWriter writer3d(
            data_dir / ("3d_" + std::to_string(index)),
            "vert_pos",
            *mesh,
            true,
            true,
            true,
            false);
        mesh->serialize(writer3d);
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

    auto vert_pos_attribute =
        mesh->register_attribute<double>("vert_pos", PrimitiveType::Vertex, 3);
    auto vert_pos_accessor = mesh->create_accessor(vert_pos_attribute.as<double>());

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
            vert_pos_attribute,
            compute_edge_length);

    //////////////////////////////////
    // computing edge lengths
    const auto edges = mesh->get_all(PrimitiveType::Edge);
    for (const auto& e : edges) {
        const auto p0 = vert_pos_accessor.vector_attribute(e);
        const auto p1 = vert_pos_accessor.vector_attribute(mesh->switch_vertex(e));

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
        //  std::make_shared<image::SamplingAnalyticFunction>(
        //      image::SamplingAnalyticFunction_FunctionType::Linear,
        //      0,
        //      0,
        //      0.)}

    };
    std::shared_ptr<wmtk::function::PerTriangleAnalyticalIntegral> accuracy =
        std::make_shared<wmtk::function::PerTriangleAnalyticalIntegral>(*mesh, pt_attribute, funcs);

    // vertex position update

    AT::function::utils::ThreeChannelPositionMapEvaluator evaluator(funcs);
    auto compute_vertex_position = [&evaluator](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        assert(P.cols() == 1);
        assert(P.rows() == 2);
        Eigen::Vector2d uv = P.col(0);
        return evaluator.uv_to_position(uv);
    };
    auto vert_position_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            vert_pos_attribute,
            pt_attribute,
            compute_vertex_position);


    /*{ // face error update
        auto face_error_attribute =
            mesh->register_attribute<double>("face_error", PrimitiveType::Face, 1);
        auto face_error_accessor = mesh->create_accessor(face_error_attribute.as<double>());

        auto compute_face_error = [&evaluator](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
            assert(P.cols() == 3);
            assert(P.rows() == 2);
            AT::function::utils::AnalyticalFunctionTriangleQuadrature analytical_quadrature(
                evaluator);
            Eigen::Vector2<double> uv0 = P.col(0);
            Eigen::Vector2<double> uv1 = P.col(1);
            Eigen::Vector2<double> uv2 = P.col(2);
            Eigen::VectorXd error(1);
            error(0) = analytical_quadrature.get_error_one_triangle_exact(uv0, uv1, uv2);
            return error;
        };
        auto face_error_update =
            std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
                face_error_attribute,
                pt_attribute,
                compute_face_error);
        for (auto& f : mesh->get_all(PrimitiveType::Face)) {
            if (!mesh->is_ccw(f)) {
                f = mesh->switch_vertex(f);
            }
            const Eigen::Vector2d v0 = pt_accessor.vector_attribute(f);
            const Eigen::Vector2d v1 = pt_accessor.vector_attribute(mesh->switch_vertex(f));
            const Eigen::Vector2d v2 =
                pt_accessor.vector_attribute(mesh->switch_vertex(mesh->switch_edge(f)));
            AT::function::utils::AnalyticalFunctionTriangleQuadrature analytical_quadrature(
                evaluator);

            auto res = analytical_quadrature.get_error_one_triangle_exact(v0, v1, v2);
            face_error_accessor.scalar_attribute(f) = res;
        }
    }*/

    // initialize this two fields
    for (const auto& v : vertices) {
        const auto p = pt_accessor.vector_attribute(v);
        vert_pos_accessor.vector_attribute(v) = compute_vertex_position(p);
    }

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
    split->set_new_attribute_strategy(vert_pos_attribute);
    // split->set_new_attribute_strategy(face_error_attribute);

    split->add_transfer_strategy(vert_position_update);
    // split->add_transfer_strategy(face_error_update);
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

    auto clps_strat2 = std::make_shared<CollapseNewAttributeStrategy<double>>(vert_pos_attribute);
    clps_strat2->set_simplex_predicate(BasicSimplexPredicate::IsInterior);
    clps_strat2->set_strategy(CollapseBasicStrategy::Default);

    collapse->set_new_attribute_strategy(vert_pos_attribute, clps_strat2);
    // collapse->set_new_attribute_strategy(face_error_attribute);

    collapse->add_transfer_strategy(vert_position_update);
    // collapse->add_transfer_strategy(face_error_update);
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

        swap->collapse().set_new_attribute_strategy(vert_pos_attribute);
        swap->split().set_new_attribute_strategy(vert_pos_attribute);
        swap->collapse().set_new_attribute_strategy(edge_length_attribute);
        swap->split().set_new_attribute_strategy(edge_length_attribute);

        swap->split().set_new_attribute_strategy(pt_attribute);
        swap->collapse().set_new_attribute_strategy(pt_attribute, CollapseBasicStrategy::CopyOther);
        swap->split().set_new_attribute_strategy(vert_pos_attribute);
        swap->collapse().set_new_attribute_strategy(
            vert_pos_attribute,
            CollapseBasicStrategy::CopyOther);

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
    for (int64_t i = 0; i < 20; ++i) {
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
    // write(mesh, "no_operation", 0, options.intermediate_output);
    const std::filesystem::path data_dir = "";
    wmtk::io::ParaviewWriter
        writer(data_dir / ("output_pos"), "vert_pos", *mesh, true, true, true, false);
    mesh->serialize(writer);
}
} // namespace wmtk::components