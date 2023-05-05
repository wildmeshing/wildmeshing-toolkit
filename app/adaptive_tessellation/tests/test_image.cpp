#include <wmtk/utils/Displacement.h>
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

TEST_CASE("sampler debug")
{
    Image image(4096, 4096);
    image.load(
        "/mnt/ssd2/yunfan/adaptive_tessellation/textures/3d_mesh/ninja/"
        "position_normal_highres/ninja_position_g.exr",
        WrappingMode::MIRROR_REPEAT,
        WrappingMode::MIRROR_REPEAT);
    SamplingBicubic sampler(image);
    wmtk::logger().info(sampler.sample(0.5666910000000002, 0.032134));
    wmtk::logger().info(sampler.sample(0.768782, 0.8556950000000002));
    wmtk::logger().info(sampler.sample(0.7714219999999999, 0.8738730000000001));
    wmtk::logger().info(sampler.sample(0.537205, 0.034348000000000004));
    // u 0.768782 v 0.8556950000000002
    // u 0.7714219999999999 v 0.8738730000000001
    // u 0.537205 v 0.034348000000000004
}

TEST_CASE("downsize quickrun")
{
    wmtk::split_and_save_3channels("/home/yunfan/material_0_world_space_normals.exr");

    // MipMap mipmap(image);
    // auto image_100 = mipmap.get_image(4);
    // image_100.save(
    //     "/mnt/ssd2/yunfan/adaptive_tessellation/textures/3d_mesh/ninja/ninja_hieght_128.exr");
    // // Image image2(100, 100);
    // image2.load(
    //     "/home/yunfan/ninja_position.exr",
    //     WrappingMode::MIRROR_REPEAT,
    //     WrappingMode::MIRROR_REPEAT);
    // MipMap mipmap2(image2);
    // auto image_100_2 = mipmap2.get_image(4);
    // image_100_2.save(
    //     "/mnt/ssd2/yunfan/adaptive_tessellation/textures/ninja_position_100_height.exr");
}
