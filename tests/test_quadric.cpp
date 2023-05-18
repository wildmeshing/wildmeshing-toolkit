#include <wmtk/image/QuadricIntegral.h>
#include <wmtk/image/helpers.h>
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

// Simple midpoint subdivision that preserves UV attributes
MeshType midpoint_subdivision(MeshType mesh)
{
    mesh.initialize_edges();
    const auto nv = mesh.get_num_vertices();
    const auto ne = mesh.get_num_edges();
    const auto nf = mesh.get_num_facets();

    MeshType new_mesh(mesh.get_dimension());
    new_mesh.add_vertices(nv + ne);
    new_mesh.add_triangles(nf * 4);

    auto vertices = vertex_ref(new_mesh);
    auto facets = facet_ref(new_mesh);
    vertices.topRows(nv) = vertex_view(mesh);
    for (Index e = 0; e < ne; ++e) {
        auto v = mesh.get_edge_vertices(e);
        vertices.row(nv + e) = Scalar(0.5) * (vertices.row(v[0]) + vertices.row(v[1]));
    }
    for (Index f = 0; f < mesh.get_num_facets(); ++f) {
        const Index v0 = mesh.get_facet_vertex(f, 0);
        const Index v1 = mesh.get_facet_vertex(f, 1);
        const Index v2 = mesh.get_facet_vertex(f, 2);
        const Index e0 = nv + mesh.get_edge(f, 0);
        const Index e1 = nv + mesh.get_edge(f, 1);
        const Index e2 = nv + mesh.get_edge(f, 2);
        facets.row(4 * f + 0) << v0, e0, e2;
        facets.row(4 * f + 1) << v1, e1, e0;
        facets.row(4 * f + 2) << v2, e2, e1;
        facets.row(4 * f + 3) << e0, e1, e2;
    }

    if (mesh.has_attribute(lagrange::AttributeName::texcoord)) {
        lagrange::SurfaceMesh32d uv_mesh(2);
        {
            auto& uv_attr = mesh.get_indexed_attribute<double>(lagrange::AttributeName::texcoord);
            auto uv_vertices = matrix_view(uv_attr.values());
            auto uv_facets = reshaped_view(uv_attr.indices(), 3);
            uv_mesh.add_vertices(uv_vertices.rows());
            uv_mesh.add_triangles(uv_facets.rows());
            vertex_ref(uv_mesh) = uv_vertices;
            facet_ref(uv_mesh) = uv_facets;
        }
        uv_mesh = midpoint_subdivision(uv_mesh);
        new_mesh.create_attribute<double>(
            lagrange::AttributeName::texcoord,
            lagrange::AttributeElement::Indexed,
            2,
            lagrange::AttributeUsage::UV);
        auto& uv_attr = new_mesh.ref_indexed_attribute<double>(lagrange::AttributeName::texcoord);
        uv_attr.values().resize_elements(uv_mesh.get_num_vertices());
        auto uv_vertices = matrix_ref(uv_attr.values());
        auto uv_facets = reshaped_ref(uv_attr.indices(), 3);
        uv_vertices = vertex_view(uv_mesh);
        uv_facets = facet_view(uv_mesh);
    }

    return new_mesh;
}

MeshType displace_mesh(MeshType mesh, const std::array<wmtk::Image, 3>& positions)
{
    // auto [scale, offset] = compute_mesh_normalization(mesh);
    auto& uv_attr = mesh.get_indexed_attribute<double>(lagrange::AttributeName::texcoord);
    auto uv_vertices = matrix_view(uv_attr.values());
    auto uv_facets = reshaped_view(uv_attr.indices(), 3);
    auto facets = facet_view(mesh);
    Eigen::MatrixXd V(mesh.get_num_vertices(), 3);
    Eigen::VectorXd denom(mesh.get_num_vertices());
    V.setZero();
    denom.setZero();
    for (int f = 0; f < static_cast<int>(mesh.get_num_facets()); ++f) {
        for (int lv = 0; lv < 3; ++lv) {
            const float u = uv_vertices(uv_facets(f, lv), 0);
            const float v = uv_vertices(uv_facets(f, lv), 1);
            V.row(facets(f, lv)) +=
                wmtk::internal::sample_bilinear(positions, u, v).transpose().cast<double>();
            denom(facets(f, lv)) += 1;
        }
    }
    V.array().colwise() /= denom.array();
    vertex_ref(mesh) = V;
    return mesh;
}

MeshType advect_vertices(
    MeshType mesh,
    const std::array<wmtk::Image, 3>& displaced_positions,
    wmtk::QuadricIntegral::QuadricType quadric_type)
{
    auto& uv_attr = mesh.get_indexed_attribute<double>(lagrange::AttributeName::texcoord);
    auto uv_vertices = matrix_view(uv_attr.values());
    auto uv_facets = reshaped_view(uv_attr.indices(), 3);

    // Compute per-facet quadrics using image integral
    std::vector<wmtk::Quadric<double>> per_facet_quadrics(mesh.get_num_facets());
    wmtk::QuadricIntegral integral(displaced_positions, quadric_type);

    for (int k = 0; k < 1; ++k) {
        integral.get_quadric_per_triangle(
            mesh.get_num_facets(),
            [&](int f) {
                std::array<float, 6> uv_triangle;
                uv_triangle[0] = uv_vertices(uv_facets(f, 0), 0);
                uv_triangle[1] = uv_vertices(uv_facets(f, 0), 1);
                uv_triangle[2] = uv_vertices(uv_facets(f, 1), 0);
                uv_triangle[3] = uv_vertices(uv_facets(f, 1), 1);
                uv_triangle[4] = uv_vertices(uv_facets(f, 2), 0);
                uv_triangle[5] = uv_vertices(uv_facets(f, 2), 1);
                return uv_triangle;
            },
            per_facet_quadrics);

        // Accumulate per-facet quadrics over their incident vertices
        auto facets = facet_view(mesh);
        std::vector<wmtk::Quadric<double>> per_vertex_quadrics(mesh.get_num_vertices());
        for (int f = 0; f < static_cast<int>(mesh.get_num_facets()); ++f) {
            for (int j = 0; j < 3; j++) {
                per_vertex_quadrics[facets(f, j)] += per_facet_quadrics[f];
            }
        }

        // Advect vertices to their quadric minimizer
        auto vertices = vertex_ref(mesh);
        for (int v = 0; v < static_cast<int>(mesh.get_num_vertices()); ++v) {
            vertices.row(v) = per_vertex_quadrics[v].minimizer().transpose();
        }
    }

    return mesh;
}


} // namespace

TEST_CASE("Quadric Integral Advection", "[utils][quadric]")
{
    std::string displaced_positions = WMTK_DATA_DIR "/images/hemisphere_512_displaced.exr";
    auto mesh = lagrange::io::load_mesh<lagrange::SurfaceMesh32d>(WMTK_DATA_DIR "/hemisphere.obj");
    auto positions = load_rgb_image(displaced_positions);

    for (size_t k = 0; k < 1; ++k) {
        mesh = midpoint_subdivision(mesh);
    }

    using QuadricType = wmtk::QuadricIntegral::QuadricType;
    lagrange::io::save_mesh("mesh_displaced.obj", displace_mesh(mesh, positions));
    lagrange::io::save_mesh(
        "mesh_point_quadric.obj",
        advect_vertices(mesh, positions, QuadricType::Point));
    lagrange::io::save_mesh(
        "mesh_plane_quadric.obj",
        advect_vertices(mesh, positions, QuadricType::Plane));
    lagrange::io::save_mesh(
        "mesh_triangle_quadric.obj",
        advect_vertices(mesh, positions, QuadricType::Triangle));
    wmtk::logger().info("done");

    {
        auto& uv_attr = mesh.get_indexed_attribute<double>(lagrange::AttributeName::texcoord);
        auto uv_vertices = matrix_view(uv_attr.values());
        auto uv_facets = reshaped_view(uv_attr.indices(), 3);
        lagrange::SurfaceMesh32d uv_mesh(2);
        uv_mesh.add_vertices(uv_vertices.rows());
        uv_mesh.add_triangles(uv_facets.rows());
        vertex_ref(uv_mesh) = uv_vertices;
        facet_ref(uv_mesh) = uv_facets;
        lagrange::io::save_mesh("mesh_uv.obj", uv_mesh);
    }
}
