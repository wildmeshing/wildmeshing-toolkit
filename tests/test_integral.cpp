#include <wmtk/image/TextureIntegral.h>
#include <wmtk/image/helpers.h>
#include <wmtk/utils/Displacement.h>
#include <wmtk/utils/load_image_exr.h>

#include <lagrange/IndexedAttribute.h>
#include <lagrange/attribute_names.h>
#include <lagrange/io/load_mesh.h>
#include <lagrange/io/save_mesh.h>
#include <lagrange/utils/assert.h>
#include <lagrange/utils/invalid.h>
#include <lagrange/views.h>

#include <spdlog/spdlog.h>
#include <catch2/catch.hpp>

#include <limits>
#include <random>

namespace {

using MeshType = lagrange::SurfaceMesh32d;
using Scalar = double;
using Index = uint32_t;

wmtk::Image buffer_to_image(const std::vector<float>& buffer, int w, int h)
{
    wmtk::Image image(w, h);
    for (int i = 0, k = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            image.set(h - i - 1, j, buffer[k++]);
        }
    }
    return image;
}

std::array<wmtk::Image, 3> load_rgb_image(const std::filesystem::path& path)
{
    auto [w, h, index_red, index_green, index_blue, buffer_r, buffer_g, buffer_b] =
        wmtk::load_image_exr_split_3channels(path);

    CAPTURE(index_red, index_green, index_blue);
    REQUIRE(index_red == 2);
    REQUIRE(index_green == 1);
    REQUIRE(index_blue == 0);

    return {
        buffer_to_image(buffer_r, w, h),
        buffer_to_image(buffer_g, w, h),
        buffer_to_image(buffer_b, w, h),
    };
}

std::vector<std::array<float, 6>> load_uv_triangles(const std::filesystem::path& path)
{
    auto mesh = lagrange::io::load_mesh<lagrange::SurfaceMesh32d>(path);
    auto& uv_attr = mesh.get_indexed_attribute<double>(lagrange::AttributeName::texcoord);
    auto uv_vertices = matrix_view(uv_attr.values());
    auto uv_facets = reshaped_view(uv_attr.indices(), 3);

    MeshType uv_mesh(2);
    uv_mesh.add_vertices(uv_vertices.rows());
    uv_mesh.add_triangles(uv_facets.rows());
    vertex_ref(uv_mesh) = uv_vertices;
    facet_ref(uv_mesh) = uv_facets;
    std::vector<std::array<float, 6>> uv_triangles(uv_mesh.get_num_facets());
    for (Index i = 0; i < uv_mesh.get_num_facets(); i++) {
        for (Index j = 0; j < 3; j++) {
            uv_triangles[i][2 * j + 0] = uv_vertices(uv_facets(i, j), 0);
            uv_triangles[i][2 * j + 1] = uv_vertices(uv_facets(i, j), 1);
        }
    }
    return uv_triangles;
}

std::pair<double, Eigen::RowVector3d> compute_mesh_normalization(const MeshType& mesh)
{
    auto vertices = vertex_view(mesh);
    const Eigen::RowVector3d box_min = vertices.colwise().minCoeff();
    const Eigen::RowVector3d box_max = vertices.colwise().maxCoeff();
    const Eigen::RowVector3d scene_extent = box_max - box_min;
    const double max_comp = scene_extent.maxCoeff();
    Eigen::RowVector3d scene_offset = -box_min;
    scene_offset -= (scene_extent.array() - max_comp).matrix() * 0.5;
    return {max_comp, scene_offset};
}

void test_integral_reference(
    const MeshType& mesh,
    std::array<wmtk::Image, 6> position_normal_images,
    wmtk::Image height,
    std::array<wmtk::Image, 3> displaced)
{
    auto& uv_attr = mesh.get_indexed_attribute<double>(lagrange::AttributeName::texcoord);
    auto uv_vertices = matrix_view(uv_attr.values());
    auto uv_facets = reshaped_view(uv_attr.indices(), 3);

    MeshType uv_mesh(2);
    uv_mesh.add_vertices(uv_vertices.rows());
    uv_mesh.add_triangles(uv_facets.rows());
    vertex_ref(uv_mesh) = uv_vertices;
    facet_ref(uv_mesh) = uv_facets;
    std::vector<std::array<float, 6>> uv_triangles(uv_mesh.get_num_facets());
    for (Index i = 0; i < uv_mesh.get_num_facets(); i++) {
        for (Index j = 0; j < 3; j++) {
            uv_triangles[i][2 * j + 0] = uv_vertices(uv_facets(i, j), 0);
            uv_triangles[i][2 * j + 1] = uv_vertices(uv_facets(i, j), 1);
        }
    }

    // Test with new engine
    std::vector<float> computed_errors(uv_mesh.get_num_facets());
    wmtk::TextureIntegral integral(std::move(displaced));
    integral.get_error_per_triangle(uv_triangles, computed_errors);

    // Test with old engine
    auto [scale, offset] = compute_mesh_normalization(mesh);
    wmtk::DisplacementMesh displacement(
        height,
        position_normal_images,
        wmtk::SAMPLING_MODE::BICUBIC,
        scale,
        offset);

// Test random uv positions
#if 0
    std::mt19937 gen;
    std::uniform_real_distribution<float> dist(0.f, 1.f);
    for (size_t k = 0; k < 100; ++k) {
        const float u = dist(gen);
        const float v = dist(gen);
        const auto expected = displacement.get(u, v);
        const auto computed = integral.get_value(u, v);
        for (size_t d = 0; d < 3; ++d) {
            REQUIRE_THAT(
                computed[d] * scale - offset[d],
                Catch::Matchers::WithinRel(expected[d], 1e-2));
        }
    }
#endif

    // Test per-triangle errors
    std::vector<double> expected(mesh.get_num_facets());
    tbb::parallel_for(size_t(0), size_t(mesh.get_num_facets()), [&](size_t f) {
        Eigen::Matrix<double, 3, 2, Eigen::RowMajor> triangle;
        triangle.row(0) = uv_vertices.row(uv_facets(f, 0));
        triangle.row(1) = uv_vertices.row(uv_facets(f, 1));
        triangle.row(2) = uv_vertices.row(uv_facets(f, 2));
        expected[f] = displacement.get_error_per_triangle(triangle);
    });

    for (int f = 0; f < mesh.get_num_facets(); ++f) {
        wmtk::logger().debug(
            "error: {}",
            std::abs(expected[f] - computed_errors[f] * scale * scale));
        REQUIRE_THAT(
            computed_errors[f] * scale * scale,
            Catch::Matchers::WithinRel(expected[f], 1e-2));
    }
    wmtk::logger().info("done with integral test");
}

void test_sampling(std::array<wmtk::Image, 3> displaced)
{
    // Check interpolation at pixel centers
    const int w = displaced[0].width();
    const int h = displaced[0].height();
    for (int x = 0; x < w; ++x) {
        for (int y = 0; y < h; ++y) {
            const float u = (static_cast<float>(x) + 0.5) / static_cast<float>(w);
            const float v = (static_cast<float>(y) + 0.5) / static_cast<float>(h);
            const auto value_nearest = wmtk::internal::sample_nearest(displaced, u, v);
            const auto value_bilinear = wmtk::internal::sample_bilinear(displaced, u, v);
            const auto value_bicubic = wmtk::internal::sample_bicubic(displaced, u, v);
            for (size_t i = 0; i < 3; ++i) {
                const auto value_pixel = displaced[i].get_raw_image()(y, x);
                CAPTURE(x, y, u, v);
                REQUIRE_THAT(value_nearest[i], Catch::Matchers::WithinRel(value_pixel, 1e-5f));
                REQUIRE_THAT(value_bilinear[i], Catch::Matchers::WithinRel(value_pixel, 1e-5f));
                REQUIRE_THAT(value_bicubic[i], Catch::Matchers::WithinRel(value_pixel, 1e-5f));
            }
        }
    }

    // Check nearest interpolation around pixel centers
    std::mt19937 gen;
    std::uniform_real_distribution<float> dist(-0.45f, 0.45f);
    for (int x = 0; x < w; ++x) {
        for (int y = 0; y < h; ++y) {
            for (size_t k = 0; k < 4; ++k) {
                const float u = (static_cast<float>(x) + 0.5 + dist(gen)) / static_cast<float>(w);
                const float v = (static_cast<float>(y) + 0.5 + dist(gen)) / static_cast<float>(h);
                const auto value_nearest = wmtk::internal::sample_nearest(displaced, u, v);
                for (size_t i = 0; i < 3; ++i) {
                    const auto value_pixel = displaced[i].get_raw_image()(y, x);
                    CAPTURE(x, y, u, v);
                    REQUIRE_THAT(value_nearest[i], Catch::Matchers::WithinRel(value_pixel, 1e-5f));
                }
            }
        }
    }

    // Check bilinear interpolation at midpoints between pixels
    for (int x = 0; x + 1 < w; ++x) {
        for (int y = 0; y + 1 < h; ++y) {
            const float u00 = (static_cast<float>(x) + 0.5) / static_cast<float>(w);
            const float u05 = (static_cast<float>(x) + 1) / static_cast<float>(w);
            const float v00 = (static_cast<float>(y) + 0.5) / static_cast<float>(h);
            const float v05 = (static_cast<float>(y) + 1) / static_cast<float>(h);
            const auto q01 = wmtk::internal::sample_bilinear(displaced, u00, v05);
            const auto q10 = wmtk::internal::sample_bilinear(displaced, u05, v00);
            const auto q11 = wmtk::internal::sample_bilinear(displaced, u05, v05);
            for (size_t i = 0; i < 3; ++i) {
                const auto p00 = displaced[i].get_raw_image()(y, x);
                const auto p10 = displaced[i].get_raw_image()(y, x + 1);
                const auto p01 = displaced[i].get_raw_image()(y + 1, x);
                const auto p11 = displaced[i].get_raw_image()(y + 1, x + 1);
                CAPTURE(x, y);
                REQUIRE_THAT(q01[i], Catch::Matchers::WithinRel(0.5f * (p00 + p01), 1e-5f));
                REQUIRE_THAT(q10[i], Catch::Matchers::WithinRel(0.5f * (p00 + p10), 1e-5f));
                REQUIRE_THAT(
                    q11[i],
                    Catch::Matchers::WithinRel(0.25f * (p00 + p01 + p10 + p11), 1e-4f));
            }
        }
    }
}

} // namespace

TEST_CASE("Texture Integral Reference", "[utils][integral]")
{
    std::string displaced_positions = WMTK_DATA_DIR "/images/hemisphere_512_displaced.exr";
    std::string position_path = WMTK_DATA_DIR "/images/hemisphere_512_position.exr";
    std::string normal_path = WMTK_DATA_DIR "/images/hemisphere_512_normal-world-space.exr";
    std::string height_path = WMTK_DATA_DIR "/images/riveted_castle_iron_door_512_height.exr";
    wmtk::Image height;
    height.load(height_path, WrappingMode::CLAMP_TO_EDGE, WrappingMode::CLAMP_TO_EDGE);
    std::array<wmtk::Image, 6> position_normal_images;
    {
        auto pos = load_rgb_image(position_path);
        auto nrm = load_rgb_image(normal_path);
        for (int i = 0; i < 3; i++) {
            position_normal_images[i] = pos[i];
            position_normal_images[i + 3] = nrm[i];
        }
    }
    test_integral_reference(
        lagrange::io::load_mesh<lagrange::SurfaceMesh32d>(WMTK_DATA_DIR "/hemisphere.obj"),
        std::move(position_normal_images),
        std::move(height),
        load_rgb_image(displaced_positions));
}

TEST_CASE("Texture Integral Sampling", "[utils][integral]")
{
    std::string displaced_positions = WMTK_DATA_DIR "/images/hemisphere_512_displaced.exr";
    auto displacement_image = load_rgb_image(displaced_positions);
    test_sampling(displacement_image);
}

TEST_CASE("Morton Z-Order", "[utils][integral]")
{
    std::string displaced_positions = WMTK_DATA_DIR "/images/hemisphere_512_displaced.exr";

    auto displacement_image_linear = load_rgb_image(displaced_positions);
    auto displacement_image_zorder = wmtk::internal::convert_image_to_morton_z_order(displacement_image_linear);

    auto planes = displacement_image_linear.size();
    auto width = displacement_image_linear[0].width();
    auto height = displacement_image_linear[0].height();
    for (int k = 0; k < planes; ++k)
    {
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                auto zorder = wmtk::internal::fetch_texels_zorder(displacement_image_zorder, x, y);
                auto linear = wmtk::internal::fetch_texels(displacement_image_linear, x, y);
                REQUIRE(zorder == linear);
            }
        }
    }
}

TEST_CASE("Texture Integral Adaptive", "[utils][integral]")
{
    std::string displaced_positions = WMTK_DATA_DIR "/images/hemisphere_512_displaced.exr";
    auto uv_triangles = load_uv_triangles(WMTK_DATA_DIR "/hemisphere.obj");
    auto mesh = lagrange::io::load_mesh<lagrange::SurfaceMesh32d>(WMTK_DATA_DIR "/hemisphere.obj");

    wmtk::TextureIntegral integral(load_rgb_image(displaced_positions));

    std::vector<float> errors_exact(uv_triangles.size());
    integral.set_sampling_method(wmtk::TextureIntegral::SamplingMethod::Bilinear);
    integral.set_integration_method(wmtk::TextureIntegral::IntegrationMethod::Exact);
    integral.get_error_per_triangle(uv_triangles, errors_exact);

    std::vector<float> errors_adaptive(uv_triangles.size());
    integral.set_integration_method(wmtk::TextureIntegral::IntegrationMethod::Adaptive);
    integral.get_error_per_triangle(uv_triangles, errors_adaptive);

    if (0) {
        // Uncomment to save file for inspection
        mesh.create_attribute<float>(
            "error_exact",
            lagrange::AttributeElement::Facet,
            1,
            lagrange::AttributeUsage::Scalar,
            errors_exact);

        mesh.create_attribute<float>(
            "error_adaptive",
            lagrange::AttributeElement::Facet,
            1,
            lagrange::AttributeUsage::Scalar,
            errors_adaptive);

        lagrange::io::save_mesh("mesh_with_errors.msh", mesh);
    }

    for (size_t f = 0; f < uv_triangles.size(); ++f) {
        CAPTURE(f);
        CHECK_THAT(errors_adaptive[f], Catch::Matchers::WithinRel(errors_exact[f], 1.5e-1f));
    }
}

TEST_CASE("Texture Integral Benchmark", "[utils][!benchmark]")
{
    // spdlog::set_level(spdlog::level::off);

    std::string displaced_positions = WMTK_DATA_DIR "/images/hemisphere_512_displaced.exr";
    // std::string displaced_positions =
    //     "/Users/jedumas/cloud/tessellation/sandbox/benchmark/hemisphere_4096_displaced.exr";
    auto uv_triangles = load_uv_triangles(WMTK_DATA_DIR "/hemisphere.obj");

    std::vector<float> computed_errors(uv_triangles.size());

    auto displacement_image_linear = load_rgb_image(displaced_positions);
    wmtk::TextureIntegral integral(displacement_image_linear);
    
    BENCHMARK("Bicubic + Exact")
    {
        integral.set_sampling_method(wmtk::TextureIntegral::SamplingMethod::Bicubic);
        integral.set_integration_method(wmtk::TextureIntegral::IntegrationMethod::Exact);
        integral.get_error_per_triangle(uv_triangles, computed_errors);
        return std::accumulate(computed_errors.begin(), computed_errors.end(), 0.f);
    };
    BENCHMARK("Bicubic + Adaptive")
    {
        integral.set_sampling_method(wmtk::TextureIntegral::SamplingMethod::Bicubic);
        integral.set_integration_method(wmtk::TextureIntegral::IntegrationMethod::Adaptive);
        integral.get_error_per_triangle(uv_triangles, computed_errors);
        return std::accumulate(computed_errors.begin(), computed_errors.end(), 0.f);
    };
    BENCHMARK("Nearest + Exact")
    {
        integral.set_sampling_method(wmtk::TextureIntegral::SamplingMethod::Nearest);
        integral.set_integration_method(wmtk::TextureIntegral::IntegrationMethod::Exact);
        integral.get_error_per_triangle(uv_triangles, computed_errors);
        return std::accumulate(computed_errors.begin(), computed_errors.end(), 0.f);
    };
    BENCHMARK("Nearest + Adaptive")
    {
        integral.set_sampling_method(wmtk::TextureIntegral::SamplingMethod::Nearest);
        integral.set_integration_method(wmtk::TextureIntegral::IntegrationMethod::Adaptive);
        integral.get_error_per_triangle(uv_triangles, computed_errors);
        return std::accumulate(computed_errors.begin(), computed_errors.end(), 0.f);
    };
    BENCHMARK("Bilinear + Exact")
    {
        integral.set_sampling_method(wmtk::TextureIntegral::SamplingMethod::Bilinear);
        integral.set_integration_method(wmtk::TextureIntegral::IntegrationMethod::Exact);
        integral.get_error_per_triangle(uv_triangles, computed_errors);
        return std::accumulate(computed_errors.begin(), computed_errors.end(), 0.f);
    };
    BENCHMARK("Bilinear + Adaptive")
    {
        integral.set_sampling_method(wmtk::TextureIntegral::SamplingMethod::Bilinear);
        integral.set_integration_method(wmtk::TextureIntegral::IntegrationMethod::Adaptive);
        integral.get_error_per_triangle(uv_triangles, computed_errors);
        return std::accumulate(computed_errors.begin(), computed_errors.end(), 0.f);
    };
}
