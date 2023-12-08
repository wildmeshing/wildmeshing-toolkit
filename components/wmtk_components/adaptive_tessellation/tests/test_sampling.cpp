#include <catch2/catch_test_macros.hpp>
#include <random>
#include <wmtk/components/adaptive_tessellation/image/Image.hpp>
using namespace wmtk;
namespace wmtk::components::adaptive_tessellation::image {
TEST_CASE("exr saving and loading")
{
    Image image(10, 10);
    auto displacement_double = [](const double& u, [[maybe_unused]] const double& v) -> double {
        return 10 * u;
    };
    image.set(displacement_double);
    image.save("tryout.exr");
    Image image2(10, 10);
    image2.load("tryout.exr", WrappingMode::MIRROR_REPEAT, WrappingMode::MIRROR_REPEAT);
    std::mt19937 gen;
    std::uniform_real_distribution<float> dist_sample(0.1f, 0.9f);
    for (int i = 0; i < 10; i++) {
        auto p1 = dist_sample(gen);
        auto p2 = dist_sample(gen);
        REQUIRE(abs(image2.get(p1, p2) - image.get(p1, p2)) < 1e-4);
    }
}
} // namespace wmtk::components::adaptive_tessellation::image