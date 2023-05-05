#include <wmtk/utils/Image.h>
#include <wmtk/utils/MipMap.h>
#include <catch2/catch.hpp>
using namespace wmtk;
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

TEST_CASE("hdr saving and loading")
{
    auto displacement = [](const double& u, const double& v) -> float {
        return (u * u * 10 * v * v * v);
    };
    Image image2(1024, 1024);
    image2.set(displacement);
    image2.save("test_hdr.hdr");

    Image image3(1024, 1024);
    image3.load("test_hdr.hdr", WrappingMode::MIRROR_REPEAT, WrappingMode::MIRROR_REPEAT);
    std::mt19937 rand_generator;
    std::uniform_real_distribution<double> rand_dist(0.15, 0.85);
    for (int j = 0; j < 10; j++) {
        Eigen::Vector2d rand_p;
        rand_p = Eigen::Vector2d(rand_dist(rand_generator), rand_dist(rand_generator));
        CAPTURE(rand_p, image3.get(rand_p.x(), rand_p.y()), displacement(rand_p.x(), rand_p.y()));
        REQUIRE(
            pow(static_cast<double>(image3.get(rand_p.x(), rand_p.y())) -
                    static_cast<double>(displacement(rand_p.x(), rand_p.y())),
                2) < 1e-2);
    }
}

TEST_CASE("mipmap")
{
    auto displacement_double = [&](const double& u, [[maybe_unused]] const double& v) -> float {
        return 10 * u;
    };
    Image image(1024, 1024);
    image.set(displacement_double);
    image.save("drlin.exr");

    MipMap mipmap(image);
    REQUIRE(mipmap.level() == 11);
    for (int i = 0; i < mipmap.level(); i++) {
        auto tmp_image = mipmap.get_image(i);
        REQUIRE(tmp_image.width() == tmp_image.height());
        REQUIRE(tmp_image.width() == pow(2, (10 - i)));
    }
}