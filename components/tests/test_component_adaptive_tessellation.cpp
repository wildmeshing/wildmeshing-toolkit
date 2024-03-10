#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/components/adaptive_tessellation/adaptive_tessellation.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/DistanceEnergy.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/PositionMapAMIPS.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/AnalyticalFunctionTriangleQuadrature.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/TextureIntegral.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/ThreeChannelPositionMapEvaluator.hpp>
#include <wmtk/components/adaptive_tessellation/image/Sampling.hpp>
#include <wmtk/components/adaptive_tessellation/operations/internal/ATEnergies.cpp>
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
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/triangle_areas.hpp>

#include <chrono>
#include <polysolve/Utils.hpp>
#include <wmtk/utils/Logger.hpp>

#include <finitediff.hpp>

#include <random>

#include <tools/DEBUG_TriMesh.hpp>
#include <tools/TriMesh_examples.hpp>
#include <wmtk/components/adaptive_tessellation/operations/RGBSplit.hpp>

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
TEST_CASE("distance_energy_correctness")
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
         std::make_shared<image::ProceduralFunction>(image::ProceduralFunctionType::Terrain)

        }};
    std::shared_ptr<function::utils::ThreeChannelPositionMapEvaluator> m_evaluator_ptr =
        std::make_shared<wmtk::components::function::utils::ThreeChannelPositionMapEvaluator>(
            funcs,
            image::SAMPLING_METHOD::Analytical);
    Eigen::Vector2d t0_uv0(0.109375, 0.578125), t0_uv1(0.109375, 0.453125), t0_uv2(0.125, 0.625);
    Eigen::Vector2d t1_uv0(0.5, 0.75), t1_uv1(0.375, 0.625), t1_uv2(0.625, 0.625);
    std::shared_ptr<wmtk::components::function::utils::AnalyticalFunctionTriangleQuadrature>
        analytical_function_distance_error = std::make_shared<
            wmtk::components::function::utils::AnalyticalFunctionTriangleQuadrature>(
            *m_evaluator_ptr);
    analytical_function_distance_error->m_debug = true;
    auto error_t0 =
        analytical_function_distance_error->get_error_one_triangle_exact(t0_uv0, t0_uv1, t0_uv2);
    auto area_t0 = wmtk::utils::triangle_unsigned_2d_area(t0_uv0, t0_uv1, t0_uv2);
    auto error_t1 =
        analytical_function_distance_error->get_error_one_triangle_exact(t1_uv0, t1_uv1, t1_uv2);
    auto area_t1 = wmtk::utils::triangle_unsigned_2d_area(t1_uv0, t1_uv1, t1_uv2);
    // logger().set_level(spdlog::level::debug);
    logger().debug("error_t0: {}", error_t0);
    logger().debug("area_t0: {}", area_t0);
    logger().debug("error_t1: {}", error_t1);
    logger().debug("area_t1: {}", area_t1);
    REQUIRE(pow(error_t0 - area_t0, 2) < 1e-5);
    REQUIRE(pow(error_t1 - area_t1, 2) < 1e-5);
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
        m.register_attribute<int64_t>("face_rgb_state", PrimitiveType::Triangle, 2, true);
    wmtk::attribute::MeshAttributeHandle m_edge_rgb_state_handle =
        m.register_attribute<int64_t>("edge_rgb_state", PrimitiveType::Edge, 2, true);
    wmtk::attribute::Accessor<int64_t> m_face_rgb_state_accessor =
        m.create_accessor(m_face_rgb_state_handle.as<int64_t>());
    wmtk::attribute::Accessor<int64_t> m_edge_rgb_state_accessor =
        m.create_accessor(m_edge_rgb_state_handle.as<int64_t>());
    wmtk::attribute::Accessor<double> m_uv_accessor = m.create_accessor(m_uv_handle.as<double>());

    // all faces set to green at level 0
    for (auto& f : m.get_all(PrimitiveType::Triangle)) {
        m_face_rgb_state_accessor.vector_attribute(f) = Eigen::Vector2<int64_t>(0, 0);
    }
    // all edges set to green at level 0
    for (auto& e : m.get_all(PrimitiveType::Edge)) {
        m_edge_rgb_state_accessor.vector_attribute(e) = Eigen::Vector2<int64_t>(0, 0);
    }
    SECTION("GG")
    {
        wmtk::operations::composite::RGBSplit op(
            m,
            m_face_rgb_state_handle,
            m_edge_rgb_state_handle);
        wmtk::simplex::Simplex middle_edge =
            wmtk::simplex::Simplex(PrimitiveType::Edge, m.edge_tuple_between_v1_v2(0, 1, 0));
        auto mods = op(middle_edge);
        REQUIRE(!mods.empty());
        // all faces should be (red, 0)
    }
    SECTION("RR") {}
    SECTION("RG") {}
}
} // namespace wmtk::components
