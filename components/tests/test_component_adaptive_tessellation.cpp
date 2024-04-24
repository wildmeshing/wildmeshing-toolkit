#include <wmtk/components/adaptive_tessellation/adaptive_tessellation.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/DistanceEnergy.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/PositionMapAMIPS.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/AnalyticalFunctionAvgDistanceToLimit.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/MaxDistanceToLimit.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/TextureMapAvgDistanceToLimit.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/ThreeChannelPositionMapEvaluator.hpp>
#include <wmtk/components/adaptive_tessellation/image/Image.hpp>
#include <wmtk/components/adaptive_tessellation/image/Sampling.hpp>
#include <wmtk/components/adaptive_tessellation/invariants/RGBSplitInvariant.hpp>
#include <wmtk/components/adaptive_tessellation/invariants/RGBSwapInvariant.hpp>
#include <wmtk/components/adaptive_tessellation/operations/RGBSplit.hpp>
#include <wmtk/components/adaptive_tessellation/operations/RGBSwap.hpp>
#include <wmtk/components/adaptive_tessellation/operations/internal/ATScheduler.hpp>
#include <wmtk/components/adaptive_tessellation/operations/internal/ATSetAttrs.cpp>
#include <wmtk/components/adaptive_tessellation/operations/utils/tag_todo_edges.hpp>
#include <wmtk/components/adaptive_tessellation/quadrature/LineQuadrature.hpp>
#include <wmtk/components/adaptive_tessellation/quadrature/Quadrature.hpp>

#include <wmtk/components/input/input.hpp>
#include <wmtk/function/PerSimplexAutodiffFunction.cpp>
#include <wmtk/function/simplex/AMIPS.cpp>
#include <wmtk/function/simplex/AMIPS.hpp>
#include <wmtk/function/simplex/TriangleAMIPS.hpp>
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/function/utils/SimplexGetter.hpp>
#include <wmtk/function/utils/amips.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/triangle_areas.hpp>

#include <predicates.h>
#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>
#include <finitediff.hpp>
#include <nlohmann/json.hpp>
#include <polysolve/Utils.hpp>
#include <wmtk/utils/Logger.hpp>

#include <chrono>
#include <random>

#include <tools/DEBUG_TriMesh.hpp>
#include <tools/TriMesh_examples.hpp>

using namespace wmtk;
using namespace wmtk::tests;
using DSVec = wmtk::function::PerSimplexAutodiffFunction::DSVec;
using DScalar = wmtk::function::PerSimplexAutodiffFunction::DScalar;
using DSVec2 = Eigen::Vector2<DScalar>;
using DSVec3 = Eigen::Vector3<DScalar>;

const std::filesystem::path data_dir = WMTK_DATA_DIR;
namespace wmtk::tests {
std::vector<DSVec> get_vertex_coordinates(
    const Mesh& mesh,
    const wmtk::attribute::Accessor<double>& accessor,
    const wmtk::simplex::Simplex& domain_simplex,
    const std::optional<wmtk::simplex::Simplex>& variable_simplex_opt)
{
    auto [attrs, index] = wmtk::function::utils::get_simplex_attributes(
        mesh,
        accessor,
        PrimitiveType::Vertex,
        domain_simplex,
        variable_simplex_opt.has_value() ? variable_simplex_opt->tuple() : std::optional<Tuple>());

    std::vector<DSVec> ret;
    ret.reserve(attrs.size());

    for (size_t i = 0; i < attrs.size(); ++i) {
        ret.emplace_back(
            i == index ? wmtk::function::utils::as_DScalar<DScalar>(attrs[i])
                       : attrs[i].cast<DScalar>());
    }

    return ret;
}

std::tuple<double, double, double> random_barycentric_coefficients()
{
    std::mt19937 rng(std::random_device{}());

    // Define the range for the random double values
    double min = 0.0;
    double max = 1.0;
    // Create a uniform_real_distribution object
    std::uniform_real_distribution<double> dist(min, max);
    // Generate a random double value
    double a = dist(rng);
    double b = dist(rng);
    if (a + b > 1.0) {
        a = 1.0 - a;
        b = 1.0 - b;
    }
    double c = 1.0 - a - b;
    assert(c > 0.);
    return {a, b, c};
}
} // namespace wmtk::tests

namespace wmtk::components {


TEST_CASE("at_test")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    nlohmann::json input_component_json = {
        {"name", "mesh"},
        {"file", (data_dir / "subdivided_quad.msh").string()},
        {"ignore_z", true}};

    wmtk::components::input(wmtk::components::base::Paths(), input_component_json, cache);

    nlohmann::json input = {
        {"passes", 10},
        {"input", "mesh"},
        {"planar", true},
        {"target_edge_length", 0.1},
        {"intermediate_output", true},
        {"output", "test_maxinv"}};

    CHECK_NOTHROW(
        wmtk::components::adaptive_tessellation(wmtk::components::base::Paths(), input, cache));
}

TEST_CASE("3damips_autodiff_performance", "[.][performance]")
{
    const std::filesystem::path mesh_path =
        data_dir / "adaptive_tessellation_test/subdivided_unit_square.msh";

    std::shared_ptr<Mesh> mesh_ptr = read_mesh(mesh_path, true);
    wmtk::attribute::MeshAttributeHandle m_uv_handle =
        mesh_ptr->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto accessor = mesh_ptr->create_accessor(m_uv_handle.as<double>());
    std::array<std::shared_ptr<image::Sampling>, 3> funcs = {{
        std::make_shared<image::SamplingAnalyticFunction>(
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
            image::SamplingAnalyticFunction_FunctionType::Linear,
            0,
            0,
            0.)
        // std::make_shared<image::SamplingAnalyticFunction>(
        //     image::SamplingAnalyticFunction_FunctionType::Gaussian,
        //     0.5,
        //     0.5,
        //     1.)
        //  std::make_shared<image::ProceduralFunction>(image::ProceduralFunctionType::Terrain)

    }};


    std::shared_ptr<function::utils::ThreeChannelPositionMapEvaluator> m_evaluator_ptr =
        std::make_shared<wmtk::components::function::utils::ThreeChannelPositionMapEvaluator>(
            funcs,
            image::SAMPLING_METHOD::Analytical);

    std::shared_ptr<wmtk::function::TriangleAMIPS> m_amips_energy =
        std::make_shared<wmtk::function::TriangleAMIPS>(*mesh_ptr, m_uv_handle);
    // iterate over all the triangles
    wmtk::function::AMIPS analytical_2damips(*mesh_ptr, m_uv_handle);

    SECTION("autodiff_gradient", "[performance]")
    {
        int cnt = 0;
        const auto start = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < 100; ++i) {
            for (const wmtk::Tuple& triangle : mesh_ptr->get_all(PrimitiveType::Triangle)) {
                cnt++;
                auto scope = wmtk::function::utils::AutoDiffRAII(2);
                std::vector<DSVec> attrs = wmtk::tests::get_vertex_coordinates(
                    *mesh_ptr,
                    accessor,
                    wmtk::simplex::Simplex(PrimitiveType::Triangle, triangle),
                    wmtk::simplex::Simplex(PrimitiveType::Vertex, triangle));
                DSVec2 a = attrs[0], b = attrs[1], c = attrs[2];
                DSVec3 p0 = m_evaluator_ptr->uv_to_position(a);
                DSVec3 p1 = m_evaluator_ptr->uv_to_position(b);
                DSVec3 p2 = m_evaluator_ptr->uv_to_position(c);

                DScalar amips = wmtk::function::utils::amips(p0, p1, p2);
                amips.getGradient();
            }
        }
        const auto stop = std::chrono::high_resolution_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        wmtk::logger().info("autodiff grad runtime: {} ms", duration.count());
        wmtk::logger().info("cnt = {}", cnt);
    }

    SECTION("autodiff_hessian", "[performance]")
    {
        int cnt = 0;
        const auto start = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < 100; ++i) {
            for (const wmtk::Tuple& triangle : mesh_ptr->get_all(PrimitiveType::Triangle)) {
                cnt++;
                auto scope = wmtk::function::utils::AutoDiffRAII(2);
                std::vector<DSVec> attrs = wmtk::tests::get_vertex_coordinates(
                    *mesh_ptr,
                    accessor,
                    wmtk::simplex::Simplex(PrimitiveType::Triangle, triangle),
                    wmtk::simplex::Simplex(PrimitiveType::Vertex, triangle));
                DSVec2 a = attrs[0], b = attrs[1], c = attrs[2];
                DSVec3 p0 = m_evaluator_ptr->uv_to_position(a);
                DSVec3 p1 = m_evaluator_ptr->uv_to_position(b);
                DSVec3 p2 = m_evaluator_ptr->uv_to_position(c);

                DScalar amips = wmtk::function::utils::amips(p0, p1, p2);
                amips.getHessian();
            }
        }
        const auto stop = std::chrono::high_resolution_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        wmtk::logger().info("autoiff hess runtime: {} ms", duration.count());
        wmtk::logger().info("cnt = {}", cnt);
    }

    SECTION("analytical_amips_grad", "[performance]")
    {
        int cnt = 0;
        const auto start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < 100; ++i) {
            for (const wmtk::Tuple& triangle : mesh_ptr->get_all(PrimitiveType::Triangle)) {
                cnt++;
                auto [attrs, index] = wmtk::function::utils::get_simplex_attributes(
                    *mesh_ptr,
                    accessor,
                    PrimitiveType::Vertex,
                    wmtk::simplex::Simplex(PrimitiveType::Triangle, triangle),
                    std::optional<Tuple>());
                Eigen::Vector2d a = attrs[0], b = attrs[1], c = attrs[2];
                Eigen::Vector3d p0 = m_evaluator_ptr->uv_to_position(a);
                Eigen::Vector3d p1 = m_evaluator_ptr->uv_to_position(b);
                Eigen::Vector3d p2 = m_evaluator_ptr->uv_to_position(c);

                std::array<double, 6> coords = wmtk::function::unbox<3, 2>(attrs, index);

                Eigen::Vector2d res;
                wmtk::function::Tri_AMIPS_jacobian(coords, res);
            }
        }
        const auto stop = std::chrono::high_resolution_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        wmtk::logger().info("analytical grad runtime: {} ms", duration.count());
        wmtk::logger().info("cnt = {}", cnt);
    }
    SECTION("analytical_amips_hessian", "[performance]")
    {
        int cnt = 0;
        const auto start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < 100; ++i) {
            for (const wmtk::Tuple& triangle : mesh_ptr->get_all(PrimitiveType::Triangle)) {
                cnt++;
                auto [attrs, index] = wmtk::function::utils::get_simplex_attributes(
                    *mesh_ptr,
                    accessor,
                    PrimitiveType::Vertex,
                    wmtk::simplex::Simplex(PrimitiveType::Triangle, triangle),
                    std::optional<Tuple>());
                Eigen::Vector2d a = attrs[0], b = attrs[1], c = attrs[2];
                Eigen::Vector3d p0 = m_evaluator_ptr->uv_to_position(a);
                Eigen::Vector3d p1 = m_evaluator_ptr->uv_to_position(b);
                Eigen::Vector3d p2 = m_evaluator_ptr->uv_to_position(c);

                std::array<double, 6> coords = wmtk::function::unbox<3, 2>(attrs, index);

                Eigen::Matrix2d res;
                wmtk::function::Tri_AMIPS_hessian(coords, res);
            }
        }
        const auto stop = std::chrono::high_resolution_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        wmtk::logger().info("analytical hess runtime: {} ms", duration.count());
        wmtk::logger().info("cnt = {}", cnt);
    }
}

TEST_CASE("3damips_autodiff_correctness")
{
    const std::filesystem::path mesh_path =
        data_dir / "adaptive_tessellation_test/subdivided_unit_square.msh";

    std::shared_ptr<Mesh> mesh_ptr = read_mesh(mesh_path, true);
    wmtk::attribute::MeshAttributeHandle m_uv_handle =
        mesh_ptr->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto accessor = mesh_ptr->create_accessor(m_uv_handle.as<double>());
    std::array<std::shared_ptr<image::Sampling>, 3> funcs = {{
        std::make_shared<image::SamplingAnalyticFunction>(
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
            image::SamplingAnalyticFunction_FunctionType::Linear,
            0,
            0,
            0.)
        // std::make_shared<image::SamplingAnalyticFunction>(
        //     image::SamplingAnalyticFunction_FunctionType::Gaussian,
        //     0.5,
        //     0.5,
        //     1.)
        //  std::make_shared<image::ProceduralFunction>(image::ProceduralFunctionType::Terrain)

    }};
    std::shared_ptr<function::utils::ThreeChannelPositionMapEvaluator> m_evaluator_ptr =
        std::make_shared<wmtk::components::function::utils::ThreeChannelPositionMapEvaluator>(
            funcs,
            image::SAMPLING_METHOD::Analytical);
    wmtk::function::AMIPS analytical_2damips(*mesh_ptr, m_uv_handle);
    SECTION("finitediff_vs_autodiff", "[correctness]")
    {
        // logger().set_level(spdlog::level::debug);
        logger().warn("check the displacement used. Analytical 2d amips check shall only be true "
                      "if using constant displacement");
        for (const wmtk::Tuple& triangle : mesh_ptr->get_all(PrimitiveType::Triangle)) {
            auto scope = wmtk::function::utils::AutoDiffRAII(2);
            std::vector<DSVec> attrs = wmtk::tests::get_vertex_coordinates(
                *mesh_ptr,
                accessor,
                wmtk::simplex::Simplex(PrimitiveType::Triangle, triangle),
                wmtk::simplex::Simplex(PrimitiveType::Vertex, triangle));
            DSVec2 a = attrs[0], b = attrs[1], c = attrs[2];
            DSVec3 p0 = m_evaluator_ptr->uv_to_position(a);
            DSVec3 p1 = m_evaluator_ptr->uv_to_position(b);
            DSVec3 p2 = m_evaluator_ptr->uv_to_position(c);
            DScalar autodiff_amips = wmtk::function::utils::amips(p0, p1, p2);
            auto autodiff_grad = autodiff_amips.getGradient();
            wmtk::logger().debug("autodiff grad: {}", autodiff_grad);

            Eigen::Vector2d uv0 = image::utils::get_double(a);
            Eigen::Vector3d double_p1 = image::utils::get_double(p1);
            Eigen::Vector3d double_p2 = image::utils::get_double(p2);
            auto double_amips = [&](const Eigen::Vector2d& x) -> double {
                Eigen::Vector3d p0 = m_evaluator_ptr->uv_to_position(x);
                return wmtk::function::utils::amips(p0, double_p1, double_p2);
            };


            Eigen::VectorXd finitediff_grad;
            fd::finite_gradient(uv0, double_amips, finitediff_grad, fd::AccuracyOrder::FOURTH);
            fd::compare_gradient(autodiff_grad, finitediff_grad);
            wmtk::logger().debug("finitediff grad: {}", finitediff_grad);
            REQUIRE(fd::compare_gradient(autodiff_grad, finitediff_grad, 1e-5));

            auto autodiff_hess = autodiff_amips.getHessian();
            wmtk::logger().debug("autodiff hess: {}", autodiff_hess);
            Eigen::MatrixXd finitediff_hess;
            fd::finite_hessian(uv0, double_amips, finitediff_hess, fd::AccuracyOrder::FOURTH);
            wmtk::logger().debug("finitediff hess: {}", finitediff_hess);
            REQUIRE(fd::compare_hessian(autodiff_hess, finitediff_hess, 1e-2));

            ///// compare the constatnt displacement hessian and gradient to 2d amips analytical hessiana and gradient
            //// if not constant displacement, this shall not be used

            Eigen::VectorXd analytical_2damips_gradient = analytical_2damips.get_gradient(
                wmtk::simplex::Simplex(PrimitiveType::Triangle, triangle),
                wmtk::simplex::Simplex(PrimitiveType::Vertex, triangle));
            REQUIRE(fd::compare_gradient(autodiff_grad, analytical_2damips_gradient, 1e-5));
            Eigen::MatrixXd analytical_2damips_hessian = analytical_2damips.get_hessian(
                wmtk::simplex::Simplex(PrimitiveType::Triangle, triangle),
                wmtk::simplex::Simplex(PrimitiveType::Vertex, triangle));
            REQUIRE(fd::compare_hessian(autodiff_hess, analytical_2damips_hessian, 1e-2));
        }
    }
}
// wmtk::components::function::utils::ThreeChannelPositionMapEvaluator image_evaluator(
//     images,
//     image::SAMPLING_METHOD::Bicubic,
//     image::IMAGE_WRAPPING_MODE::MIRROR_REPEAT);
// wmtk::components::function::utils::ThreeChannelPositionMapEvaluator func_evaluator(funcs);
// wmtk::components::operations::internal::_debug_sampling(
//     atdata.uv_mesh_ptr(),
//     atdata.uv_handle(),
//     image_evaluator,
//     func_evaluator);

// wmtk::components::operations::internal::_debug_texture_integral(
//     atdata.uv_mesh_ptr(),
//     atdata.uv_handle(),
//     image_evaluator,
//     func_evaluator);
TEST_CASE("avg_distance_energy_correctness")
{
    std::array<std::shared_ptr<image::Sampling>, 3> funcs = {
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
             image::SamplingAnalyticFunction_FunctionType::Linear,
             0,
             0,
             5.)

        }};
    std::shared_ptr<function::utils::ThreeChannelPositionMapEvaluator> m_evaluator_ptr =
        std::make_shared<wmtk::components::function::utils::ThreeChannelPositionMapEvaluator>(
            funcs,
            image::SAMPLING_METHOD::Analytical);

    // Seed the random number generator
    std::mt19937 rng(std::random_device{}());

    // Define the range for the random double values
    double min = 0.0;
    double max = 100.0;

    // Create a uniform_real_distribution object
    std::uniform_real_distribution<double> dist(min, max);

    for (int rand_trail = 0; rand_trail < 100; ++rand_trail) {
        // Generate a random double value
        double u0 = dist(rng);
        double v0 = dist(rng);

        double u1 = dist(rng);
        double v1 = dist(rng);

        double u2 = dist(rng);
        double v2 = dist(rng);
        Eigen::Vector2d uv0(u0, v0), uv1(u1, v1), uv2(u2, v2);

        if (orient2d(uv0.data(), uv1.data(), uv2.data()) < 0) {
            std::swap(uv1, uv2);
        }
        std::shared_ptr<wmtk::components::function::utils::AnalyticalFunctionAvgDistanceToLimit>
            analytical_function_distance_error = std::make_shared<
                wmtk::components::function::utils::AnalyticalFunctionAvgDistanceToLimit>(
                *m_evaluator_ptr);
        analytical_function_distance_error->m_debug = true;
        double error = analytical_function_distance_error->distance(uv0, uv1, uv2);
        REQUIRE(pow(error - 1, 2) < 1e-9);
    }
}

TEST_CASE("curved_edge_length_line_quadrature")
{
    std::array<std::shared_ptr<image::Sampling>, 3> funcs = {
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
             image::SamplingAnalyticFunction_FunctionType::Linear,
             0,
             0,
             1.)

        }};
    std::shared_ptr<function::utils::ThreeChannelPositionMapEvaluator> m_evaluator_ptr =
        std::make_shared<wmtk::components::function::utils::ThreeChannelPositionMapEvaluator>(
            funcs,
            image::SAMPLING_METHOD::Analytical);

    // Seed the random number generator
    std::mt19937 rng(std::random_device{}());

    // Define the range for the random double values
    double min = 0.0;
    double max = 100.0;

    // Create a uniform_real_distribution object
    std::uniform_real_distribution<double> dist(min, max);

    for (int rand_trail = 0; rand_trail < 100; ++rand_trail) {
        // Generate a random double value
        double u0 = dist(rng);
        double v0 = dist(rng);

        double u1 = dist(rng);
        double v1 = dist(rng);
        Eigen::Vector2d uv0(u0, v0), uv1(u1, v1);
        double arc_length = wmtk::components::operations::internal::ATOperations::
            curved_edge_length_on_displaced_surface(uv0, uv1, m_evaluator_ptr);

        REQUIRE(
            pow(arc_length -
                    (m_evaluator_ptr->uv_to_position(uv1) - m_evaluator_ptr->uv_to_position(uv0))
                        .norm(),
                2) < 1e-5);
    }
}

TEST_CASE("rgb_split")
{
    tests::DEBUG_TriMesh m = tests::unit_squre();

    wmtk::attribute::MeshAttributeHandle m_uv_handle =
        m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    wmtk::attribute::MeshAttributeHandle m_face_rgb_state_handle =
        m.register_attribute<int64_t>("face_rgb_state", PrimitiveType::Triangle, 2, true, 0);
    wmtk::attribute::MeshAttributeHandle m_edge_rgb_state_handle =
        m.register_attribute<int64_t>("edge_rgb_state", PrimitiveType::Edge, 2, true, 0);
    wmtk::attribute::Accessor<int64_t> m_face_rgb_state_accessor =
        m.create_accessor(m_face_rgb_state_handle.as<int64_t>());
    wmtk::attribute::Accessor<int64_t> m_edge_rgb_state_accessor =
        m.create_accessor(m_edge_rgb_state_handle.as<int64_t>());
    wmtk::attribute::Accessor<double> m_uv_accessor = m.create_accessor(m_uv_handle.as<double>());
    wmtk::operations::composite::RGBSplit op(
        m,
        m_uv_handle,
        m_uv_handle,
        m_face_rgb_state_handle,
        m_edge_rgb_state_handle);
    op.split().set_new_attribute_strategy(
        m_uv_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::Mean);
    op.split().set_new_attribute_strategy(
        m_face_rgb_state_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    op.split().set_new_attribute_strategy(
        m_edge_rgb_state_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    SECTION("GG")
    {
        wmtk::simplex::Simplex middle_edge =
            wmtk::simplex::Simplex(PrimitiveType::Edge, m.edge_tuple_between_v1_v2(0, 1, 0));
        auto mods = op(middle_edge);
        REQUIRE(!mods.empty());
        REQUIRE(mods.size() == 1);
        // all faces should be (red, 0)
        for (auto& f : m.get_all(PrimitiveType::Triangle)) {
            REQUIRE(m_face_rgb_state_accessor.vector_attribute(f) == Eigen::Vector2<int64_t>(1, 0));
        }
        Tuple split_return = mods.front().tuple();
        // the edge should be (green, 1)
        REQUIRE(
            m_edge_rgb_state_accessor.vector_attribute(split_return) ==
            Eigen::Vector2<int64_t>(0, 1));
        Tuple other_new_edge = m.switch_tuples(
            split_return,
            {PrimitiveType::Edge, PrimitiveType::Triangle, PrimitiveType::Edge});
        // the other edge should be (green, 1)
        REQUIRE(
            m_edge_rgb_state_accessor.vector_attribute(other_new_edge) ==
            Eigen::Vector2<int64_t>(0, 1));

        Tuple rib_edge0 = m.switch_tuple(split_return, {PrimitiveType::Edge});
        Tuple rib_edge1 =
            m.switch_tuples(split_return, {PrimitiveType::Triangle, PrimitiveType::Edge});
        // the rib edge should be (red, 0)
        REQUIRE(
            m_edge_rgb_state_accessor.vector_attribute(rib_edge0) == Eigen::Vector2<int64_t>(1, 0));
        REQUIRE(
            m_edge_rgb_state_accessor.vector_attribute(rib_edge1) == Eigen::Vector2<int64_t>(1, 0));

        Tuple ear_edge0 =
            m.switch_tuples(split_return, {PrimitiveType::Vertex, PrimitiveType::Edge});
        Tuple ear_edge1 = m.switch_tuples(
            rib_edge0,
            {PrimitiveType::Triangle, PrimitiveType::Vertex, PrimitiveType::Edge});
        Tuple ear_edge2 = m.switch_tuples(
            split_return,
            {PrimitiveType::Triangle, PrimitiveType::Vertex, PrimitiveType::Edge});
        Tuple ear_edge3 = m.switch_tuples(
            rib_edge1,
            {PrimitiveType::Triangle, PrimitiveType::Vertex, PrimitiveType::Edge});

        // the ear edges should be (green, 0)
        REQUIRE(
            m_edge_rgb_state_accessor.vector_attribute(ear_edge0) == Eigen::Vector2<int64_t>(0, 0));
        REQUIRE(
            m_edge_rgb_state_accessor.vector_attribute(ear_edge1) == Eigen::Vector2<int64_t>(0, 0));
        REQUIRE(
            m_edge_rgb_state_accessor.vector_attribute(ear_edge2) == Eigen::Vector2<int64_t>(0, 0));
        REQUIRE(
            m_edge_rgb_state_accessor.vector_attribute(ear_edge3) == Eigen::Vector2<int64_t>(0, 0));
    }
    SECTION("RR")
    {
        Tuple bnd_edge0 = m.edge_tuple_between_v1_v2(0, 2, 0);
        auto mods = op(wmtk::simplex::Simplex(PrimitiveType::Edge, bnd_edge0));
        REQUIRE(!mods.empty());
        REQUIRE(mods.size() == 1);
        REQUIRE(
            m_face_rgb_state_accessor.vector_attribute(mods.front().tuple()) ==
            Eigen::Vector2<int64_t>(1, 0));
        REQUIRE(
            m_face_rgb_state_accessor.vector_attribute(m.switch_tuples(
                mods.front().tuple(),
                {PrimitiveType::Edge, PrimitiveType::Triangle})) == Eigen::Vector2<int64_t>(1, 0));
        Tuple bnd_edge1 = m.edge_tuple_between_v1_v2(1, 3, 1);
        mods = op(wmtk::simplex::Simplex(PrimitiveType::Edge, bnd_edge1));
        REQUIRE(!mods.empty());
        REQUIRE(mods.size() == 1);
        REQUIRE(
            m_face_rgb_state_accessor.vector_attribute(mods.front().tuple()) ==
            Eigen::Vector2<int64_t>(1, 0));
        REQUIRE(
            m_face_rgb_state_accessor.vector_attribute(m.switch_tuples(
                mods.front().tuple(),
                {PrimitiveType::Edge, PrimitiveType::Triangle})) == Eigen::Vector2<int64_t>(1, 0));
        Tuple edge = m.edge_tuple_from_vids(0, 1);
        // the input face is red
        REQUIRE(m_face_rgb_state_accessor.vector_attribute(edge) == Eigen::Vector2<int64_t>(1, 0));
        mods = op(simplex::Simplex(PrimitiveType::Edge, edge));
        REQUIRE(!mods.empty());
        REQUIRE(mods.size() == 1);
        Tuple split_return = mods.front().tuple();
        // the edge should be (green, 1)
        REQUIRE(
            m_edge_rgb_state_accessor.vector_attribute(split_return) ==
            Eigen::Vector2<int64_t>(0, 1));
        // the other new edge should also be (green, 1)
        Tuple other_new_edge = m.switch_tuples(
            split_return,
            {PrimitiveType::Edge, PrimitiveType::Triangle, PrimitiveType::Edge});
        REQUIRE(
            m_edge_rgb_state_accessor.vector_attribute(other_new_edge) ==
            Eigen::Vector2<int64_t>(0, 1));
        // rib edges should be (green,1)
        Tuple rib_edge0 = m.switch_tuple(split_return, {PrimitiveType::Edge});

        REQUIRE(
            m_edge_rgb_state_accessor.vector_attribute(rib_edge0) == Eigen::Vector2<int64_t>(0, 1));
        // the ear edge that is red has face (blue, 0)
        // the ear edge that is green has face (green, 1)
        Tuple ear_edge0 =
            m.switch_tuples(split_return, {PrimitiveType::Vertex, PrimitiveType::Edge});
        Tuple ear_edge1 = m.switch_tuples(
            rib_edge0,
            {PrimitiveType::Triangle, PrimitiveType::Vertex, PrimitiveType::Edge});
        REQUIRE(
            ((m_edge_rgb_state_accessor.vector_attribute(ear_edge0) ==
              Eigen::Vector2<int64_t>(1, 0)) ||
             (m_edge_rgb_state_accessor.vector_attribute(ear_edge1) ==
              Eigen::Vector2<int64_t>(1, 0))));
        Tuple blue_ear, green_ear;
        if (m_edge_rgb_state_accessor.vector_attribute(ear_edge0) ==
            Eigen::Vector2<int64_t>(1, 0)) {
            blue_ear = ear_edge0;
            green_ear = ear_edge1;
        } else if (
            m_edge_rgb_state_accessor.vector_attribute(ear_edge1) ==
            Eigen::Vector2<int64_t>(1, 0)) {
            blue_ear = ear_edge1;
            green_ear = ear_edge0;
        } else {
            REQUIRE(false);
        }

        REQUIRE(
            m_face_rgb_state_accessor.vector_attribute(blue_ear) == Eigen::Vector2<int64_t>(2, 0));
        REQUIRE(
            m_face_rgb_state_accessor.vector_attribute(green_ear) == Eigen::Vector2<int64_t>(0, 1));
    }
    SECTION("RG")
    {
        Tuple bnd_edge0 = m.edge_tuple_between_v1_v2(0, 2, 0);
        auto mods = op(wmtk::simplex::Simplex(PrimitiveType::Edge, bnd_edge0));
        REQUIRE(!mods.empty());
        REQUIRE(mods.size() == 1);
        REQUIRE(
            m_face_rgb_state_accessor.vector_attribute(mods.front().tuple()) ==
            Eigen::Vector2<int64_t>(1, 0));
        REQUIRE(
            m_face_rgb_state_accessor.vector_attribute(m.switch_tuples(
                mods.front().tuple(),
                {PrimitiveType::Edge, PrimitiveType::Triangle})) == Eigen::Vector2<int64_t>(1, 0));
        Tuple edge = m.edge_tuple_from_vids(0, 1);
        // the input has one side green, one side red
        // same_side_red is true is the face of the edge tuple is red
        bool same_side_red = false;

        if (m_face_rgb_state_accessor.vector_attribute(edge) == Eigen::Vector2<int64_t>(1, 0)) {
            REQUIRE(
                m_face_rgb_state_accessor.vector_attribute(
                    m.switch_tuple(edge, PrimitiveType::Triangle)) ==
                Eigen::Vector2<int64_t>(0, 0));
            same_side_red = true;
        } else if (
            m_face_rgb_state_accessor.vector_attribute(edge) == Eigen::Vector2<int64_t>(0, 0)) {
            REQUIRE(
                m_face_rgb_state_accessor.vector_attribute(
                    m.switch_tuple(edge, PrimitiveType::Triangle)) ==
                Eigen::Vector2<int64_t>(1, 0));
        } else {
            REQUIRE(false);
        }

        mods = op(simplex::Simplex(PrimitiveType::Edge, edge));
        REQUIRE(!mods.empty());
        REQUIRE(mods.size() == 1);
        Tuple split_return = mods.front().tuple();
        // the edge should be (green, 1)
        REQUIRE(
            m_edge_rgb_state_accessor.vector_attribute(split_return) ==
            Eigen::Vector2<int64_t>(0, 1));
        // the other new edge should also be (green, 1)
        Tuple other_new_edge = m.switch_tuples(
            split_return,
            {PrimitiveType::Edge, PrimitiveType::Triangle, PrimitiveType::Edge});
        REQUIRE(
            m_edge_rgb_state_accessor.vector_attribute(other_new_edge) ==
            Eigen::Vector2<int64_t>(0, 1));
        // if same_side_red is true, then this side of facs are (blue, 0) or (green, 1) given the
        // ear edge colors
        // If the ear edge is (red, 0) then the face is (blue, 0)
        // If the ear edge is (green, 1) then the face is (green, 1)
        Tuple ear_edge0 =
            m.switch_tuples(split_return, {PrimitiveType::Vertex, PrimitiveType::Edge});
        Tuple rib_edge0 = m.switch_tuple(split_return, {PrimitiveType::Edge});
        Tuple ear_edge1 = m.switch_tuples(
            rib_edge0,
            {PrimitiveType::Triangle, PrimitiveType::Vertex, PrimitiveType::Edge});
        Tuple ear_edge2 = m.switch_tuples(
            split_return,
            {PrimitiveType::Triangle, PrimitiveType::Vertex, PrimitiveType::Edge});
        Tuple rib_edge1 =
            m.switch_tuples(split_return, {PrimitiveType::Triangle, PrimitiveType::Edge});
        Tuple ear_edge3 = m.switch_tuples(
            rib_edge1,
            {PrimitiveType::Triangle, PrimitiveType::Vertex, PrimitiveType::Edge});
        Tuple red_side_rib, other_side_rib;
        Tuple red_ear, green_ear, other_green_ear0, other_green_ear1;

        if (same_side_red) {
            // rib edge of this side should be (green, 1)
            other_side_rib = rib_edge0;
            // rib edge of the other side should be (red, 0)
            red_side_rib = rib_edge1;
            // other side should be green ears after split
            other_green_ear0 = ear_edge2;
            other_green_ear1 = ear_edge3;
            // ear0, ear1 one should be (green,1), the other should be (red,0)
            // and the face should accordingly be (green,1) or (blue,0)
            if (m_edge_rgb_state_accessor.vector_attribute(ear_edge0) ==
                Eigen::Vector2<int64_t>(0, 1)) {
                green_ear = ear_edge0;
                red_ear = ear_edge1;

            } else if (
                m_edge_rgb_state_accessor.vector_attribute(ear_edge0) ==
                Eigen::Vector2<int64_t>(1, 0)) {
                green_ear = ear_edge1;
                red_ear = ear_edge0;

            } else {
                REQUIRE(false);
            }
        } else {
            // rib edge of this this should be (red, 0)
            red_side_rib = rib_edge0;
            // rib edge of the other side should be (green, 1)
            other_side_rib = rib_edge1;
            // this side should be green ears after split
            other_green_ear0 = ear_edge0;
            other_green_ear1 = ear_edge1;
            // ear2, ear3 one should be (green,1), the other should be (red,0)
            // and the face should accordingly be (green,1) or (blue,0)
            if (m_edge_rgb_state_accessor.vector_attribute(ear_edge2) ==
                Eigen::Vector2<int64_t>(0, 1)) {
                green_ear = ear_edge2;
                red_ear = ear_edge3;

            } else if (
                m_edge_rgb_state_accessor.vector_attribute(ear_edge2) ==
                Eigen::Vector2<int64_t>(1, 0)) {
                green_ear = ear_edge3;
                red_ear = ear_edge2;

            } else {
                REQUIRE(false);
            }
        }

        REQUIRE(
            m_edge_rgb_state_accessor.vector_attribute(other_side_rib) ==
            Eigen::Vector2<int64_t>(0, 1));

        REQUIRE(
            m_edge_rgb_state_accessor.vector_attribute(red_side_rib) ==
            Eigen::Vector2<int64_t>(1, 0));
        REQUIRE(
            m_edge_rgb_state_accessor.vector_attribute(red_ear) == Eigen::Vector2<int64_t>(1, 0));
        REQUIRE(
            m_face_rgb_state_accessor.vector_attribute(red_ear) == Eigen::Vector2<int64_t>(2, 0));

        REQUIRE(
            m_face_rgb_state_accessor.vector_attribute(green_ear) == Eigen::Vector2<int64_t>(0, 1));
        REQUIRE(
            m_edge_rgb_state_accessor.vector_attribute(green_ear) == Eigen::Vector2<int64_t>(0, 1));

        REQUIRE(
            m_edge_rgb_state_accessor.vector_attribute(other_green_ear0) ==
            Eigen::Vector2<int64_t>(0, 0));
        REQUIRE(
            m_edge_rgb_state_accessor.vector_attribute(other_green_ear1) ==
            Eigen::Vector2<int64_t>(0, 0));
        REQUIRE(
            m_face_rgb_state_accessor.vector_attribute(other_green_ear0) ==
            Eigen::Vector2<int64_t>(1, 0));
        REQUIRE(
            m_face_rgb_state_accessor.vector_attribute(other_green_ear1) ==
            Eigen::Vector2<int64_t>(1, 0));
    }
    SECTION("split_invariant")
    {
        op.add_invariant(std::make_shared<wmtk::RGBSplitInvariant>(
            m,
            m_face_rgb_state_handle.as<int64_t>(),
            m_edge_rgb_state_handle.as<int64_t>()));

        auto mods = op(simplex::Simplex(PrimitiveType::Edge, m.edge_tuple_between_v1_v2(0, 1, 0)));
        REQUIRE(!mods.empty());
        REQUIRE(mods.size() == 1);
        Tuple red_edge = m.switch_tuple(mods.front().tuple(), PrimitiveType::Edge);
        mods = op(simplex::Simplex(PrimitiveType::Edge, red_edge));
        REQUIRE(mods.empty());
    }
}
std::shared_ptr<wmtk::operations::Operation> rgb_split(
    wmtk::Mesh& m,
    wmtk::attribute::MeshAttributeHandle& m_uv_handle,
    wmtk::attribute::MeshAttributeHandle& m_face_rgb_state_handle,
    wmtk::attribute::MeshAttributeHandle& m_edge_rgb_state_handle,
    wmtk::attribute::MeshAttributeHandle& m_edge_todo_handle)
{
    std::shared_ptr<wmtk::operations::composite::RGBSplit> op_split =
        std::make_shared<wmtk::operations::composite::RGBSplit>(
            m,
            m_uv_handle,
            m_uv_handle,
            m_face_rgb_state_handle,
            m_edge_rgb_state_handle);
    op_split->split().set_new_attribute_strategy(
        m_uv_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::Mean);
    op_split->split().set_new_attribute_strategy(
        m_face_rgb_state_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    op_split->split().set_new_attribute_strategy(
        m_edge_rgb_state_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    op_split->split().set_new_attribute_strategy(
        m_edge_todo_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    op_split->add_invariant(std::make_shared<wmtk::RGBSplitInvariant>(
        m,
        m_face_rgb_state_handle.as<int64_t>(),
        m_edge_rgb_state_handle.as<int64_t>()));
    op_split->add_invariant(
        std::make_shared<wmtk::TodoInvariant>(m, m_edge_todo_handle.as<int64_t>(), 1));
    return op_split;
}

std::shared_ptr<wmtk::operations::Operation> rgb_swap(
    wmtk::Mesh& m,
    wmtk::attribute::MeshAttributeHandle& m_uv_handle,
    wmtk::attribute::MeshAttributeHandle& m_face_rgb_state_handle,
    wmtk::attribute::MeshAttributeHandle& m_edge_rgb_state_handle,
    wmtk::attribute::MeshAttributeHandle& m_edge_todo_handle)
{
    std::shared_ptr<wmtk::operations::composite::RGBSwap> op_swap =
        std::make_shared<wmtk::operations::composite::RGBSwap>(
            m,
            m_face_rgb_state_handle,
            m_edge_rgb_state_handle,
            m_edge_todo_handle);

    op_swap->swap().split().set_new_attribute_strategy(
        m_uv_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::Mean);
    op_swap->swap().collapse().set_new_attribute_strategy(
        m_uv_handle,
        wmtk::operations::CollapseBasicStrategy::CopyOther);
    op_swap->swap().split().set_new_attribute_strategy(
        m_face_rgb_state_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    op_swap->swap().collapse().set_new_attribute_strategy(
        m_face_rgb_state_handle,
        wmtk::operations::CollapseBasicStrategy::None);
    op_swap->swap().split().set_new_attribute_strategy(
        m_edge_rgb_state_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    op_swap->swap().collapse().set_new_attribute_strategy(
        m_edge_rgb_state_handle,
        wmtk::operations::CollapseBasicStrategy::None);

    op_swap->swap().split().set_new_attribute_strategy(
        m_edge_todo_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    op_swap->swap().collapse().set_new_attribute_strategy(
        m_edge_todo_handle,
        wmtk::operations::CollapseBasicStrategy::None);

    op_swap->add_invariant(std::make_shared<wmtk::RGBSwapInvariant>(
        m,
        m_face_rgb_state_handle.as<int64_t>(),
        m_edge_rgb_state_handle.as<int64_t>()));
    return op_swap;
}

TEST_CASE("rgb_swap")
{
    tests::DEBUG_TriMesh m = tests::single_equilateral_triangle(2);

    wmtk::attribute::MeshAttributeHandle m_uv_handle =
        m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    wmtk::attribute::MeshAttributeHandle m_face_rgb_state_handle =
        m.register_attribute<int64_t>("face_rgb_state", PrimitiveType::Triangle, 2, true, 0);
    wmtk::attribute::MeshAttributeHandle m_edge_rgb_state_handle =
        m.register_attribute<int64_t>("edge_rgb_state", PrimitiveType::Edge, 2, true, 0);
    wmtk::attribute::MeshAttributeHandle m_edge_todo_handle =
        m.register_attribute<int64_t>("edge_todo", PrimitiveType::Edge, 1, true, 0);
    wmtk::attribute::Accessor<int64_t> m_face_rgb_state_accessor =
        m.create_accessor(m_face_rgb_state_handle.as<int64_t>());
    wmtk::attribute::Accessor<int64_t> m_edge_rgb_state_accessor =
        m.create_accessor(m_edge_rgb_state_handle.as<int64_t>());
    wmtk::attribute::Accessor<double> m_uv_accessor = m.create_accessor(m_uv_handle.as<double>());
    wmtk::attribute::Accessor<int64_t> m_edge_todo_accessor =
        m.create_accessor(m_edge_todo_handle.as<int64_t>());

    auto op_split = rgb_split(
        m,
        m_uv_handle,
        m_face_rgb_state_handle,
        m_edge_rgb_state_handle,
        m_edge_todo_handle);

    auto op_swap = rgb_swap(
        m,
        m_uv_handle,
        m_face_rgb_state_handle,
        m_edge_rgb_state_handle,
        m_edge_todo_handle);


    //  create blue_face/ red_edge/ blue_face configuration
    Tuple edge = m.edge_tuple_from_vids(1, 2);
    // tag this edge
    m_edge_todo_accessor.scalar_attribute(edge) = 1;
    auto mods = (*op_split)(simplex::Simplex(PrimitiveType::Edge, edge));
    REQUIRE(!mods.empty());
    REQUIRE(mods.size() == 1);
    Tuple split_return = mods.front().tuple();
    Tuple rib_edge = m.switch_tuple(split_return, PrimitiveType::Edge);
    REQUIRE(
        m_edge_rgb_state_accessor.vector_attribute(split_return) == Eigen::Vector2<int64_t>(0, 1));
    REQUIRE(m_edge_rgb_state_accessor.vector_attribute(rib_edge) == Eigen::Vector2<int64_t>(1, 0));
    // and both the return edge and the rib should have tag 0
    REQUIRE(m_edge_todo_accessor.scalar_attribute(split_return) == 0);
    REQUIRE(m_edge_todo_accessor.scalar_attribute(rib_edge) == 0);
    // the other new edge should also has tag 0
    REQUIRE(
        m_edge_todo_accessor.scalar_attribute(
            m.switch_tuples(rib_edge, {PrimitiveType::Triangle, PrimitiveType::Edge})) == 0);

    Tuple ear0 = m.switch_tuples(split_return, {PrimitiveType::Vertex, PrimitiveType::Edge});
    // set this ear0 to have tag 1
    m_edge_todo_accessor.scalar_attribute(ear0) = 1;
    mods = (*op_split)(simplex::Simplex(PrimitiveType::Edge, ear0));
    REQUIRE(!mods.empty());
    REQUIRE(mods.size() == 1);
    REQUIRE(m_edge_todo_accessor.scalar_attribute(mods.front().tuple()) == 0);

    Tuple ear1 = m.switch_tuples(
        mods.front().tuple(),
        {PrimitiveType::Vertex, PrimitiveType::Edge, PrimitiveType::Triangle, PrimitiveType::Edge});
    // set this ear1 to have tag 1
    m_edge_todo_accessor.scalar_attribute(ear1) = 1;
    mods = (*op_split)(simplex::Simplex(PrimitiveType::Edge, ear1));
    REQUIRE(!mods.empty());
    REQUIRE(mods.size() == 1);
    REQUIRE(m_edge_todo_accessor.scalar_attribute(mods.front().tuple()) == 0);
    // at this point there should be only one red edge of (red, 0)
    // on the two sides of this red edge are face of (blue, 0)
    //  all the other edges are (green, 1)
    bool red_edge = false;
    Tuple red_edge_tuple;
    for (auto& edge : m.get_all(PrimitiveType::Edge)) {
        if (m_edge_rgb_state_accessor.vector_attribute(edge) == Eigen::Vector2<int64_t>(1, 0)) {
            REQUIRE(!red_edge);
            red_edge = true;
            red_edge_tuple = edge;
            REQUIRE(
                m_face_rgb_state_accessor.vector_attribute(edge) == Eigen::Vector2<int64_t>(2, 0));
            REQUIRE(
                m_face_rgb_state_accessor.vector_attribute(
                    m.switch_tuple(edge, PrimitiveType::Triangle)) ==
                Eigen::Vector2<int64_t>(2, 0));

        } else {
            REQUIRE(
                m_edge_rgb_state_accessor.vector_attribute(edge) == Eigen::Vector2<int64_t>(0, 1));
        }
    }
    // test swap invariant
    mods = (*op_swap)(simplex::Simplex(PrimitiveType::Edge, mods.front().tuple()));
    // this is not a valid swap
    REQUIRE(mods.empty());

    // tag the swap edge and now we do a valid swap
    m_edge_todo_accessor.scalar_attribute(red_edge_tuple) = 1;
    mods = (*op_swap)(simplex::Simplex(PrimitiveType::Edge, red_edge_tuple));

    REQUIRE(!mods.empty());
    REQUIRE(mods.size() == 1);
    // swap return edge tuple should be (green, 1)
    Tuple swap_return = mods.front().tuple();
    REQUIRE(
        m_edge_rgb_state_accessor.vector_attribute(swap_return) == Eigen::Vector2<int64_t>(0, 1));
    // and the return edge tuple tag should be 1
    // REQUIRE(m_edge_todo_accessor.scalar_attribute(swap_return) == 1);

    // all the triangles shoul dbe (green, 1)
    for (auto& f : m.get_all(PrimitiveType::Triangle)) {
        REQUIRE(m_face_rgb_state_accessor.vector_attribute(f) == Eigen::Vector2<int64_t>(0, 1));
    }

    // test swap invariant
    mods = (*op_swap)(simplex::Simplex(PrimitiveType::Edge, swap_return));
    REQUIRE(mods.empty());
}

TEST_CASE("recursive_rgb_split")
{
    const std::filesystem::path mesh_path =
        data_dir / "adaptive_tessellation_test/upsample_square.msh";

    std::shared_ptr<Mesh> m_ptr = read_mesh(mesh_path, true);
    wmtk::attribute::MeshAttributeHandle m_uv_handle =
        m_ptr->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    wmtk::attribute::MeshAttributeHandle m_face_rgb_state_handle =
        m_ptr->register_attribute<int64_t>("face_rgb_state", PrimitiveType::Triangle, 2, true, 0);
    wmtk::attribute::MeshAttributeHandle m_edge_rgb_state_handle =
        m_ptr->register_attribute<int64_t>("edge_rgb_state", PrimitiveType::Edge, 2, true, 0);

    // we want to split every edge
    wmtk::attribute::MeshAttributeHandle m_edge_todo_handle =
        m_ptr->register_attribute<int64_t>("edge_todo", PrimitiveType::Edge, 1, true, 0);
    wmtk::attribute::Accessor<int64_t> m_face_rgb_state_accessor =
        m_ptr->create_accessor(m_face_rgb_state_handle.as<int64_t>());
    wmtk::attribute::Accessor<int64_t> m_edge_rgb_state_accessor =
        m_ptr->create_accessor(m_edge_rgb_state_handle.as<int64_t>());
    wmtk::attribute::Accessor<double> m_uv_accessor =
        m_ptr->create_accessor(m_uv_handle.as<double>());
    wmtk::attribute::Accessor<int64_t> m_edge_todo_accessor =
        m_ptr->create_accessor(m_edge_todo_handle.as<int64_t>());
    auto op_split = rgb_split(
        *m_ptr,
        m_uv_handle,
        m_face_rgb_state_handle,
        m_edge_rgb_state_handle,
        m_edge_todo_handle);

    auto op_swap = rgb_swap(
        *m_ptr,
        m_uv_handle,
        m_face_rgb_state_handle,
        m_edge_rgb_state_handle,
        m_edge_todo_handle);
    wmtk::ATScheduler scheduler;
    // tag all edges to be 1
    for (auto& e : m_ptr->get_all(PrimitiveType::Edge)) {
        m_edge_todo_accessor.scalar_attribute(e) = 1;
    }


    while (true) {
        const auto stats = scheduler.run_operation_on_all(*op_split);
        if (stats.number_of_successful_operations() == 0) {
            break;
        }
    }

    for (auto& e : m_ptr->get_all(PrimitiveType::Edge)) {
        REQUIRE(m_edge_todo_accessor.scalar_attribute(e) == 0);
    }
    for (auto& f : m_ptr->get_all(PrimitiveType::Triangle)) {
        if (m_face_rgb_state_accessor.vector_attribute(f)[0] == 2) {
            REQUIRE(
                ((m_edge_rgb_state_accessor.vector_attribute(f)[0] == 1) ||
                 (m_edge_rgb_state_accessor.vector_attribute(
                      m_ptr->switch_tuple(f, PrimitiveType::Edge))[0] == 1) ||
                 (m_edge_rgb_state_accessor.vector_attribute(
                      m_ptr->switch_tuples(f, {PrimitiveType::Vertex, PrimitiveType::Edge}))[0] ==
                  1)));
        }
    }

    // a lambda function that checks the (red, l) edge case for split edge
    auto check_red_edge = [&](Tuple edge) {
        // case 1: the edge is a (red, l)
        //  it can be ajcent to (red,l) or (blue,l) faces. Onlyt the red face can have (green,l)
        //  edge
        // Tag the red face's (green, l) edges as todo
        REQUIRE(m_edge_rgb_state_accessor.vector_attribute(edge)[0] == 1);
        Tuple f0 = edge;
        Tuple f1 = m_ptr->switch_tuple(edge, PrimitiveType::Triangle);
        auto f0_color_level = m_face_rgb_state_accessor.vector_attribute(f0);
        auto f1_color_level = m_face_rgb_state_accessor.vector_attribute(f1);

        REQUIRE(((f0_color_level[0] == 1) || (f1_color_level[0] == 2)));
        REQUIRE(((f1_color_level[0] == 1) || (f1_color_level[0] == 2)));
        Tuple red_face = f0_color_level[0] == 1 ? f0 : f1;
        Tuple blue_face = f0_color_level[0] == 2 ? f0 : f1;
        // the red face has two green ear edges
        // one is the same level as the edge and the face, the other is l+1
        Tuple red_ear0 = m_ptr->switch_tuple(red_face, PrimitiveType::Edge);
        Tuple red_ear1 =
            m_ptr->switch_tuples(red_face, {PrimitiveType::Vertex, PrimitiveType::Edge});
        REQUIRE(m_edge_rgb_state_accessor.vector_attribute(red_ear0)[0] == 0);
        REQUIRE(m_edge_rgb_state_accessor.vector_attribute(red_ear1)[0] == 0);
        REQUIRE(
            (m_edge_rgb_state_accessor.vector_attribute(red_ear0)[1] ==
                 m_edge_rgb_state_accessor.vector_attribute(edge)[1] ||
             m_edge_rgb_state_accessor.vector_attribute(red_ear0)[1] ==
                 m_edge_rgb_state_accessor.vector_attribute(edge)[1] + 1));
        REQUIRE(
            (m_edge_rgb_state_accessor.vector_attribute(red_ear1)[1] ==
                 m_edge_rgb_state_accessor.vector_attribute(edge)[1] ||
             m_edge_rgb_state_accessor.vector_attribute(red_ear1)[1] ==
                 m_edge_rgb_state_accessor.vector_attribute(edge)[1] + 1));

        // blue face has two green ears
        Tuple blue_ear0 = m_ptr->switch_tuple(blue_face, PrimitiveType::Edge);
        Tuple blue_ear1 =
            m_ptr->switch_tuples(blue_face, {PrimitiveType::Vertex, PrimitiveType::Edge});
        REQUIRE(m_edge_rgb_state_accessor.vector_attribute(blue_ear0)[0] == 0);
        REQUIRE(m_edge_rgb_state_accessor.vector_attribute(blue_ear1)[0] == 0);
        REQUIRE(
            (m_edge_rgb_state_accessor.vector_attribute(blue_ear0)[1] ==
             m_edge_rgb_state_accessor.vector_attribute(edge)[1] + 1));
        REQUIRE(
            (m_edge_rgb_state_accessor.vector_attribute(blue_ear1)[1] ==
             m_edge_rgb_state_accessor.vector_attribute(edge)[1] + 1));
    };


    // check green edge
    auto check_green_edge = [&](Tuple edge) {
        // case 2: the edge is a (green, l)
        REQUIRE(m_edge_rgb_state_accessor.vector_attribute(edge)[0] == 0);
        Tuple f0 = edge;
        auto edge_color_level = m_edge_rgb_state_accessor.vector_attribute(edge);
        auto f0_color_level = m_face_rgb_state_accessor.vector_attribute(f0);
        // the face of the edge can be a green face, a blue face, or a red face

        if (f0_color_level[0] == 0) {
            // green face:
            // the other face must be blue face or red face.
            Tuple other_face = m_ptr->switch_tuple(edge, PrimitiveType::Triangle);
            REQUIRE(m_face_rgb_state_accessor.vector_attribute(other_face)[0] != 0);
            if (m_face_rgb_state_accessor.vector_attribute(other_face)[0] == 1) {
                // red face
                // it must be a red, l-1 face
                REQUIRE(
                    m_face_rgb_state_accessor.vector_attribute(other_face)[1] ==
                    edge_color_level[1] - 1);
            } else if (m_face_rgb_state_accessor.vector_attribute(other_face)[0] == 2) {
                // blue face
                // it must be a blue, l-1 face
                REQUIRE(
                    m_face_rgb_state_accessor.vector_attribute(other_face)[1] ==
                    edge_color_level[1] - 1);
            } else {
                REQUIRE(false);
            }
        }
        if (f0_color_level[0] == 1) {
            // red face:
            // it must be a red, l-1 face
            REQUIRE(m_face_rgb_state_accessor.vector_attribute(f0)[1] == edge_color_level[1] - 1);
            // among the two ear edges must exist a green, l-1
            Tuple ear0 = m_ptr->switch_tuple(edge, PrimitiveType::Edge);
            Tuple ear1 = m_ptr->switch_tuples(edge, {PrimitiveType::Vertex, PrimitiveType::Edge});
            REQUIRE(
                (m_edge_rgb_state_accessor.vector_attribute(ear0)[0] == 0 ||
                 m_edge_rgb_state_accessor.vector_attribute(ear1)[0] == 0));
            REQUIRE(
                (m_edge_rgb_state_accessor.vector_attribute(ear0)[1] == edge_color_level[1] - 1 ||
                 m_edge_rgb_state_accessor.vector_attribute(ear1)[1] == edge_color_level[1] - 1));
        }
        if (f0_color_level[0] == 2) {
            // blue face:
            // it must be a blue, l-1 face
            REQUIRE(m_face_rgb_state_accessor.vector_attribute(f0)[1] == edge_color_level[1] - 1);
            // among the two ear edges must exist a red, l-1
            Tuple ear0 = m_ptr->switch_tuple(edge, PrimitiveType::Edge);
            Tuple ear1 = m_ptr->switch_tuples(edge, {PrimitiveType::Vertex, PrimitiveType::Edge});
            REQUIRE(
                (m_edge_rgb_state_accessor.vector_attribute(ear0)[0] == 1 ||
                 m_edge_rgb_state_accessor.vector_attribute(ear1)[0] == 1));
            REQUIRE(
                (m_edge_rgb_state_accessor.vector_attribute(ear0)[1] == edge_color_level[1] - 1 ||
                 m_edge_rgb_state_accessor.vector_attribute(ear1)[1] == edge_color_level[1] - 1));

            // get the red edge
            Tuple red_edge = m_edge_rgb_state_accessor.vector_attribute(ear0)[0] == 1 ? ear0 : ear1;
            if (m_ptr->is_boundary(simplex::Simplex(PrimitiveType::Edge, red_edge))) return;
            // the adjacent face to the red_edge is either blue or red
            Tuple other_face_red_edge = m_ptr->switch_tuple(red_edge, PrimitiveType::Triangle);
            REQUIRE(m_face_rgb_state_accessor.vector_attribute(other_face_red_edge)[0] != 0);
        }
    };
    wmtk::RGBSplitInvariant rgb_split_invariant(
        *m_ptr,
        m_face_rgb_state_handle.as<int64_t>(),
        m_edge_rgb_state_handle.as<int64_t>());


    for (auto& e : m_ptr->get_all(PrimitiveType::Edge)) {
        if (m_edge_rgb_state_accessor.vector_attribute(e)[0] == 1) {
            // case 1: the edge is a (red, l)
            //  it can be ajcent to (red,l) or (blue,l) faces. Onlyt the red face can have (green,l)
            //  edge
            REQUIRE(!rgb_split_invariant.before(simplex::Simplex(PrimitiveType::Edge, e)));
            check_red_edge(e);
        } else if (
            m_edge_rgb_state_accessor.vector_attribute(e)[0] == 0 &&
            !rgb_split_invariant.before(simplex::Simplex(PrimitiveType::Edge, e))) {
            check_green_edge(e);
            if (m_ptr->is_boundary(simplex::Simplex(PrimitiveType::Edge, e))) continue;

            check_green_edge(m_ptr->switch_tuple(e, PrimitiveType::Triangle));
            break;
        }
    }

    for (auto& e : m_ptr->get_all(PrimitiveType::Edge)) {
        if (m_edge_rgb_state_accessor.vector_attribute(e)[0] == 1) {
            REQUIRE(!rgb_split_invariant.before(simplex::Simplex(PrimitiveType::Edge, e)));
            // tag this edge
            m_edge_todo_accessor.scalar_attribute(e) = 1;
            wmtk::components::operations::utils::tag_secondary_split_edges(
                m_ptr,
                m_face_rgb_state_accessor,
                m_edge_rgb_state_accessor,
                m_edge_todo_accessor,
                e);
            scheduler.rgb_recursive_split_swap(
                m_ptr,
                m_face_rgb_state_handle,
                m_edge_rgb_state_handle,
                m_edge_todo_accessor,
                *op_split,
                *op_swap);
            for (auto& inner_e : m_ptr->get_all(PrimitiveType::Edge)) {
                if (m_edge_rgb_state_accessor.vector_attribute(inner_e)[0] == 1) {
                    REQUIRE(m_edge_todo_accessor.scalar_attribute(inner_e) == 0);
                }
            }
            break;
        }
    }
    for (auto& e : m_ptr->get_all(PrimitiveType::Edge)) {
        if (m_edge_rgb_state_accessor.vector_attribute(e)[0] == 0 &&
            !rgb_split_invariant.before(simplex::Simplex(PrimitiveType::Edge, e))) {
            REQUIRE(!rgb_split_invariant.before(simplex::Simplex(PrimitiveType::Edge, e)));
            // tag this edge
            m_edge_todo_accessor.scalar_attribute(e) = 1;
            wmtk::components::operations::utils::tag_secondary_split_edges(
                m_ptr,
                m_face_rgb_state_accessor,
                m_edge_rgb_state_accessor,
                m_edge_todo_accessor,
                e);
            scheduler.rgb_recursive_split_swap(
                m_ptr,
                m_face_rgb_state_handle,
                m_edge_rgb_state_handle,
                m_edge_todo_accessor,
                *op_split,
                *op_swap);
            for (auto& inner_e : m_ptr->get_all(PrimitiveType::Edge)) {
                if (m_edge_rgb_state_accessor.vector_attribute(inner_e)[0] == 1) {
                    REQUIRE(m_edge_todo_accessor.scalar_attribute(inner_e) == 0);
                }
            }
            break;
        }
    }
}

TEST_CASE("max_dist")
{
    std::array<std::shared_ptr<image::Image>, 3> images = {
        {std::make_shared<image::Image>(5, 5),
         std::make_shared<image::Image>(5, 5),
         std::make_shared<image::Image>(5, 5)}};

    auto u_func = [](const double& u, [[maybe_unused]] const double& v) -> double { return u; };
    auto v_func = []([[maybe_unused]] const double& u, const double& v) -> double { return v; };
    auto height_function = [](const double& u, [[maybe_unused]] const double& v) -> double {
        return exp(-(pow(u - 0.5, 2) + pow(v - 0.5, 2)) / (2 * 0.1 * 0.1));
    };
    images[0]->set(u_func);
    images[1]->set(v_func);
    images[2]->set(height_function);

    std::shared_ptr<function::utils::ThreeChannelPositionMapEvaluator> m_evaluator_ptr =
        std::make_shared<wmtk::components::function::utils::ThreeChannelPositionMapEvaluator>(
            images,
            image::SAMPLING_METHOD::Bilinear);
    Eigen::AlignedBox2d bbox;
    Eigen::Vector2d a(0.2, 0.2);
    Eigen::Vector2d b(0.9, 0.2);
    Eigen::Vector2d c(0.5, 0.9);

    bbox.extend(a);
    bbox.extend(b);
    bbox.extend(c);
    auto MD = wmtk::components::function::utils::MaxDistanceToLimit(*m_evaluator_ptr);
    SECTION("intersections")
    {
        std::vector<double> intersects0 = MD._debug_grid_line_intersections(a, b);
        REQUIRE(intersects0.size() == 5);
        std::vector<double> intersects1 = MD._debug_grid_line_intersections(c, a);
        REQUIRE(intersects1.size() == 6);

        std::vector<double> intersects2 = MD._debug_grid_line_intersections(c, b);
        REQUIRE(intersects2.size() == 6);

        for (auto& inter : intersects0) {
            REQUIRE(inter >= 0);
            REQUIRE(inter <= 1);
            auto uv = a * (1 - inter) + b * inter;
        }
        for (auto& inter : intersects1) {
            REQUIRE(inter >= 0);
            REQUIRE(inter <= 1);
            auto uv = a * (1 - inter) + b * inter;
        }
        for (auto& inter : intersects2) {
            REQUIRE(inter >= 0);
            REQUIRE(inter <= 1);
            auto uv = a * (1 - inter) + b * inter;
        }
    }

    SECTION("pixel_inside")
    {
        Eigen::Matrix3d position_Col_Major = Eigen::Matrix3d::Zero();
        position_Col_Major.col(0) = Vector3d(0., 0., 0.);
        position_Col_Major.col(1) = Vector3d(1., 0., 0.);
        position_Col_Major.col(2) = Vector3d(.5, 1., 0.);
        double max = MD._debug_max_disatance_pixel_corners_inside_triangle(
            a,
            b,
            c,
            position_Col_Major,
            bbox);
        REQUIRE(max > 1);
    }

    SECTION("load_square_mesh")
    {
        const std::filesystem::path mesh_path =
            "/mnt/ssd2/yunfan/AT_2024/data/shifted_square_upsample_2.msh";

        std::shared_ptr<Mesh> mesh_ptr = read_mesh(mesh_path, true);
        wmtk::attribute::MeshAttributeHandle m_uv_handle =
            mesh_ptr->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
        auto accessor = mesh_ptr->create_accessor(m_uv_handle.as<double>());
        // iterate over every face
        for (auto& f : mesh_ptr->get_all(PrimitiveType::Triangle)) {
            Eigen::Vector2d uv0 = accessor.vector_attribute(f);
            Eigen::Vector2d uv1 =
                accessor.vector_attribute(mesh_ptr->switch_tuple(f, PrimitiveType::Vertex));
            Eigen::Vector2d uv2 = accessor.vector_attribute(
                mesh_ptr->switch_tuples(f, {PrimitiveType::Edge, PrimitiveType::Vertex}));

            Eigen::Vector3d p0 = m_evaluator_ptr->uv_to_position<double>(uv0);
            Eigen::Vector3d p1 = m_evaluator_ptr->uv_to_position<double>(uv1);
            Eigen::Vector3d p2 = m_evaluator_ptr->uv_to_position<double>(uv2);

            double max_dist = MD.distance(uv0, uv1, uv2);

            double max_sampled_dist = 0.;
            for (int i = 0; i < 10000; i++) {
                auto coefficients = wmtk::tests::random_barycentric_coefficients();

                // Access the elements of the tuple using std::get
                double a = std::get<0>(coefficients);
                double b = std::get<1>(coefficients);
                double c = std::get<2>(coefficients);

                Eigen::Vector2d uv = a * uv0 + b * uv1 + c * uv2;
                Eigen::Vector3d p = m_evaluator_ptr->uv_to_position<double>(uv);
                Eigen::Vector3d bary_p = a * p0 + b * p1 + c * p2;
                double dist = (p - bary_p).norm();
                max_sampled_dist = std::max(dist, max_sampled_dist);
            }
            wmtk::logger().info(
                "max_dist: {}, max_sampled_dist: {} should be within 1e-4",
                max_dist,
                max_sampled_dist * wmtk::utils::triangle_unsigned_2d_area(uv0, uv1, uv2));
        }
    }
}

TEST_CASE("downsample_image")
{
    wmtk::components::image::Image img;
    img.load("image_path");
    img = img.down_sample();
    auto downsampled = img.down_sample();
    downsampled.save("output_path");
}

} // namespace wmtk::components
