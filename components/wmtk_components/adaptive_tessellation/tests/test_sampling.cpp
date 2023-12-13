#include <catch2/catch_test_macros.hpp>
#include <random>
#include <wmtk/components/adaptive_tessellation/image/Image.hpp>
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/function/utils/AutoDiffUtils.hpp>
#include <wmtk/function/utils/PositionMapEvaluator.hpp>
#include <wmtk/utils/Logger.hpp>

using namespace wmtk;
using DScalar = DScalar2<double, Eigen::Matrix<double, -1, 1>, Eigen::Matrix<double, -1, -1>>;
namespace wmtk::components::adaptive_tessellation::image {
TEST_CASE("exr saving and loading")
{
    Image image(100, 100);
    auto height_function = [](const double& u, [[maybe_unused]] const double& v) -> double {
        return sin(2 * M_PI * u) * cos(2 * M_PI * v);
    };
    image.set(height_function);
    image.save("sinucosv.exr");
    Image image2(100, 100);
    image2.load("sinucosv.exr", WrappingMode::MIRROR_REPEAT, WrappingMode::MIRROR_REPEAT);
    std::mt19937 gen;
    std::uniform_real_distribution<float> dist_sample(0.1f, 0.9f);
    for (int i = 0; i < 10; i++) {
        auto p1 = dist_sample(gen);
        auto p2 = dist_sample(gen);
        REQUIRE(pow(image.get(p1, p2) - height_function(p1, p2), 2) < 1e-5);
        REQUIRE(pow(image2.get(p1, p2) - height_function(p1, p2), 2) < 1e-5);
        REQUIRE(pow(image2.get(p1, p2) - image.get(p1, p2), 2) < 1e-5);
    }
}

TEST_CASE("uv_to_pos mapping functor")
{
    Image image(100, 100);
    auto height_function = [](const double& u, [[maybe_unused]] const double& v) -> double {
        return sin(2 * M_PI * u) * cos(2 * M_PI * v);
    };
    image.set(height_function);
    function::utils::PositionMapEvaluator evaluator(image);
    std::mt19937 gen;
    std::uniform_real_distribution<float> dist_sample(0.1f, 0.9f);
    for (int i = 0; i < 10; i++) {
        auto p1 = dist_sample(gen);
        auto p2 = dist_sample(gen);
        auto pos = evaluator.uv_to_pos(Vector2<double>(p1, p2));
        REQUIRE(pow(pos.z() - height_function(p1, p2), 2) < 1e-5);
    }
}

TEST_CASE("sampling gradient")
{
    Image image(100, 100);
    auto height_function = [](const double& u, [[maybe_unused]] const double& v) -> double {
        return sin(2 * M_PI * u) * cos(2 * M_PI * v);
    };
    auto height_gradient = [](const double& u,
                              [[maybe_unused]] const double& v) -> Vector2<double> {
        return Vector2<double>(
            2 * M_PI * cos(2 * M_PI * u) * cos(2 * M_PI * v),
            -2 * M_PI * sin(2 * M_PI * u) * sin(2 * M_PI * v));
    };

    image.set(height_function);
    function::utils::PositionMapEvaluator evaluator(image);
    std::mt19937 gen;
    std::uniform_real_distribution<float> dist_sample(0.1f, 0.9f);
    auto scope = wmtk::function::utils::AutoDiffRAII(2);
    for (int i = 0; i < 10; i++) {
        auto p1 = dist_sample(gen);
        auto p2 = dist_sample(gen);
        auto uv = Vector2<double>(p1, p2);
        Vector2<DScalar> uvD = wmtk::function::utils::as_DScalar<DScalar>(uv);
        DScalar pos = evaluator.uv_to_height<DScalar>(uvD);
        REQUIRE((pos.getGradient() - height_gradient(p1, p2)).squaredNorm() < 1e-5);
    }
}
} // namespace wmtk::components::adaptive_tessellation::image