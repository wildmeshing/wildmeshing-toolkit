#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/components/adaptive_tessellation/adaptive_tessellation.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/DistanceEnergy.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/AnalyticalFunctionTriangleQuadrature.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/TextureIntegral.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/ThreeChannelPositionMapEvaluator.hpp>
#include <wmtk/components/adaptive_tessellation/image/Sampling.hpp>
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

#include <chrono>
#include <polysolve/Utils.hpp>
#include <wmtk/utils/Logger.hpp>

#include <finitediff.hpp>

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

TEST_CASE("3damips_autodiff_performance_correctness")
{
    std::shared_ptr<Mesh> mesh_ptr = read_mesh(
        "/home/yunfan/wildmeshing-toolkit/data/adaptive_tessellation_test/"
        "subdivided_unit_square.msh",
        true);
    wmtk::attribute::MeshAttributeHandle m_uv_handle =
        mesh_ptr->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto accessor = mesh_ptr->create_accessor(m_uv_handle.as<double>());
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
    std::shared_ptr<wmtk::components::function::utils::AnalyticalFunctionTriangleQuadrature>
        m_integral_ptr = std::make_shared<
            wmtk::components::function::utils::AnalyticalFunctionTriangleQuadrature>(
            *m_evaluator_ptr);
    std::shared_ptr<wmtk::function::TriangleAMIPS> m_amips_energy =
        std::make_shared<wmtk::function::TriangleAMIPS>(*mesh_ptr, m_uv_handle);
    std::shared_ptr<wmtk::function::DistanceEnergy> m_distance_energy =
        std::make_shared<wmtk::function::DistanceEnergy>(*mesh_ptr, m_uv_handle, m_integral_ptr);
    // iterate over all the triangles

    SECTION("autodiff_gradient", "[performance]")
    {
        int cnt = 0;
        const auto start = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < 1000; ++i) {
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

        for (int i = 0; i < 1000; ++i) {
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
        for (int i = 0; i < 1000; ++i) {
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
        for (int i = 0; i < 1000; ++i) {
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
    SECTION("finitediff_vs_autodiff", "[correctness]")
    {
        // logger().set_level(spdlog::level::debug);
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
            DScalar amips = wmtk::function::utils::amips(p0, p1, p2);
            auto autodiff_grad = amips.getGradient();
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

            auto autodiff_hess = amips.getHessian();
            wmtk::logger().debug("autodiff hess: {}", autodiff_hess);
            Eigen::MatrixXd finitediff_hess;
            fd::finite_hessian(uv0, double_amips, finitediff_hess, fd::AccuracyOrder::FOURTH);
            wmtk::logger().debug("finitediff hess: {}", finitediff_hess);
            REQUIRE(fd::compare_hessian(autodiff_hess, finitediff_hess, 1));
        }
    }
}


} // namespace wmtk::components