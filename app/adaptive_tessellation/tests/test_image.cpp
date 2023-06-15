#include <igl/read_triangle_mesh.h>
#include <wmtk/image/Image.h>
#include <wmtk/image/MipMap.h>
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

// NOTE: unused
TEST_CASE("hdr saving and loading", "[.]")
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

namespace {

std::pair<double, Eigen::RowVector3d> compute_mesh_normalization(const Eigen::MatrixXd& V)
{
    const Eigen::RowVector3d box_min = V.colwise().minCoeff();
    const Eigen::RowVector3d box_max = V.colwise().maxCoeff();
    const Eigen::RowVector3d scene_extent = box_max - box_min;
    const double max_comp = scene_extent.maxCoeff();
    Eigen::RowVector3d scene_offset = -box_min;
    scene_offset -= (scene_extent.array() - max_comp).matrix() * 0.5;
    return {max_comp, scene_offset};
}

} // namespace

TEST_CASE("combined displaced map")
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::read_triangle_mesh(WMTK_DATA_DIR "/hemisphere.obj", V, F);
    auto [scale, offset] = compute_mesh_normalization(V);

    std::array<wmtk::Image, 3> displaced_images = wmtk::combine_position_normal_texture(
        scale,
        offset,
        WMTK_DATA_DIR "/images/hemisphere_512_position.exr",
        WMTK_DATA_DIR "/images/hemisphere_512_normal-world-space.exr",
        WMTK_DATA_DIR "/images/riveted_castle_iron_door_512_height.exr",
        0,
        1);

    std::array<wmtk::Image, 3> displaced_from_precomputed =
        wmtk::load_rgb_image(WMTK_DATA_DIR "/images/hemisphere_512_displaced.exr");
    REQUIRE(displaced_from_precomputed[0].width() == displaced_images[0].width());
    REQUIRE(displaced_from_precomputed[0].height() == displaced_images[0].height());
    REQUIRE(displaced_from_precomputed[1].width() == displaced_images[1].width());
    REQUIRE(displaced_from_precomputed[1].height() == displaced_images[1].height());
    REQUIRE(displaced_from_precomputed[2].width() == displaced_images[2].width());
    REQUIRE(displaced_from_precomputed[2].height() == displaced_images[2].height());
    for (int i = 0; i < displaced_from_precomputed[0].height(); i++) {
        for (int j = 0; j < displaced_from_precomputed[0].width(); j++) {
            REQUIRE_THAT(
                displaced_from_precomputed[0].get_pixel(i, j) * scale - offset[0],
                Catch::Matchers::WithinRel(displaced_images[0].get_pixel(i, j), (float)1e-2));
            REQUIRE_THAT(
                displaced_from_precomputed[1].get_pixel(i, j) * scale - offset[1],
                Catch::Matchers::WithinRel(displaced_images[1].get_pixel(i, j), (float)1e-2));
            REQUIRE_THAT(
                displaced_from_precomputed[2].get_pixel(i, j) * scale - offset[2],
                Catch::Matchers::WithinRel(displaced_images[2].get_pixel(i, j), (float)1e-2));
        }
    }
}

TEST_CASE("downsample image")
{
    Image image(1024, 1024);
    image.load(
        "/home/yunfan/riveted_castle_iron_door_2048_height.exr",
        WrappingMode::MIRROR_REPEAT,
        WrappingMode::MIRROR_REPEAT);
    auto low_res = image.down_sample();
    low_res.save("/home/yunfan/riveted_castle_iron_door_1024_height.exr");
}
