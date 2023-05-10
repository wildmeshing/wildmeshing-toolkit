#include <wmtk/image/TextureIntegral.h>
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

void test_integral(
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

} // namespace

TEST_CASE("Texture Integral", "[utils][integral]")
{
    std::string displaced_positions = WMT_DATA_DIR "/images/hemisphere_512_displaced.exr";
    std::string position_path = WMT_DATA_DIR "/images/hemisphere_512_position.exr";
    std::string normal_path = WMT_DATA_DIR "/images/hemisphere_512_normal-world-space.exr";
    std::string height_path = WMT_DATA_DIR "/images/riveted_castle_iron_door_512_height.exr";
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
    test_integral(
        lagrange::io::load_mesh<lagrange::SurfaceMesh32d>(WMT_DATA_DIR "/hemisphere.obj"),
        std::move(position_normal_images),
        std::move(height),
        load_rgb_image(displaced_positions));
}

TEST_CASE("Texture Integral Benchmark", "[utils][!benchmark]")
{
    spdlog::set_level(spdlog::level::off);

    std::string displaced_positions = WMT_DATA_DIR "/images/hemisphere_512_displaced.exr";

    auto mesh = lagrange::io::load_mesh<lagrange::SurfaceMesh32d>(WMT_DATA_DIR "/hemisphere.obj");
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
    wmtk::TextureIntegral integral(load_rgb_image(displaced_positions));

    BENCHMARK("Bicubic")
    {
        integral.get_error_per_triangle(uv_triangles, computed_errors, 0);
        float total = 0;
        for (size_t i = 0; i < computed_errors.size(); ++i) {
            total += computed_errors[i];
        }
        return total;
    };
    BENCHMARK("Nearest")
    {
        integral.get_error_per_triangle(uv_triangles, computed_errors, 1);
        float total = 0;
        for (size_t i = 0; i < computed_errors.size(); ++i) {
            total += computed_errors[i];
        }
        return total;
    };
    BENCHMARK("Bilinear")
    {
        integral.get_error_per_triangle(uv_triangles, computed_errors, 2);
        float total = 0;
        for (size_t i = 0; i < computed_errors.size(); ++i) {
            total += computed_errors[i];
        }
        return total;
    };
}
