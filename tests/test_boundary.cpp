
#include <wmtk/utils/BoundaryParametrization.h>

#include <lagrange/IndexedAttribute.h>
#include <lagrange/attribute_names.h>
#include <lagrange/io/load_mesh.h>
#include <lagrange/io/save_mesh.h>
#include <lagrange/utils/assert.h>
#include <lagrange/utils/invalid.h>
#include <lagrange/views.h>

#include <catch2/catch.hpp>

#include <limits>

namespace {

using MeshType = lagrange::SurfaceMesh32d;
using Scalar = double;
using Index = uint32_t;

std::pair<Eigen::MatrixXi, Eigen::MatrixXi> compute_seam_edges(MeshType mesh)
{
    std::vector<std::array<Index, 4>> seams;
    mesh.initialize_edges();
    auto& uv_attr = mesh.get_indexed_attribute<Scalar>(lagrange::AttributeName::texcoord);
    auto facets = facet_view(mesh);
    auto uv_facets = reshaped_view(uv_attr.indices(), 3);
    std::vector<Index> corners_around_edge;
    for (Index e = 0; e < mesh.get_num_edges(); ++e) {
        corners_around_edge.clear();
        mesh.foreach_corner_around_edge(e, [&](Index c) { corners_around_edge.push_back(c); });
        if (corners_around_edge.size() <= 1) {
            // Boundary edge, skip
            continue;
        }
        if (corners_around_edge.size() > 2) {
            throw std::runtime_error("Non-manifold edges are not supported");
        }
        // Interior edge with exactly two incident facets
        Index ci = corners_around_edge.front();
        Index cj = corners_around_edge.back();
        Index fi = mesh.get_corner_facet(ci);
        Index fj = mesh.get_corner_facet(cj);
        Index lvi0 = ci - mesh.get_facet_corner_begin(fi);
        Index lvj0 = cj - mesh.get_facet_corner_begin(fj);
        Index lvi1 = (lvi0 + 1) % 3;
        Index lvj1 = (lvj0 + 1) % 3;
        la_debug_assert(facets(fi, lvi0) == facets(fj, lvj1));
        la_debug_assert(facets(fi, lvi1) == facets(fj, lvj0));
        if (uv_facets(fi, lvi0) != uv_facets(fj, lvj1) ||
            uv_facets(fi, lvi1) != uv_facets(fj, lvj0)) {
            // Edge e is a seam! Do something...
            seams.push_back({
                uv_facets(fi, lvi0),
                uv_facets(fi, lvi1),
                uv_facets(fj, lvj1),
                uv_facets(fj, lvj0),
            });
        }
    }

    Eigen::MatrixXi E0(seams.size(), 2);
    Eigen::MatrixXi E1(seams.size(), 2);
    for (size_t i = 0; i < seams.size(); ++i) {
        auto [a, b, c, d] = seams[i];
        E0.row(i) << a, b;
        E1.row(i) << c, d;
    }

    return std::make_pair(E0, E1);
}

class BoundaryTester : public wmtk::Boundary
{
public:
    static void test_boundary_parameterization(const MeshType& mesh, int expected_num_curves)
    {
        auto& uv_attr = mesh.get_indexed_attribute<double>(lagrange::AttributeName::texcoord);
        auto uv_vertices = matrix_view(uv_attr.values());
        auto uv_facets = reshaped_view(uv_attr.indices(), 3);

        MeshType uv_mesh(2);
        uv_mesh.add_vertices(uv_vertices.rows());
        uv_mesh.add_triangles(uv_facets.rows());
        vertex_ref(uv_mesh) = uv_vertices;
        facet_ref(uv_mesh) = uv_facets;
        // lagrange::io::save_mesh("uv_mesh.obj", uv_mesh);

        auto [E0, E1] = compute_seam_edges(mesh);

        BoundaryTester boundary;
        boundary.construct_boundaries(uv_vertices, uv_facets.cast<int>(), E0, E1);

        auto curve_to_mesh = [&](int curve_id, bool parametrized = false) {
            MeshType seams;
            const auto& positions = boundary.positions(curve_id);
            REQUIRE(positions.size() >= 2);
            seams.add_vertices(positions.size(), [&](Index v, lagrange::span<Scalar> p) {
                if (parametrized) {
                    auto parent_id = boundary.parent_curve(curve_id);
                    auto result = boundary.t_to_uv(curve_id, boundary.arclengths(parent_id)[v]);
                    p[0] = result.x();
                    p[1] = result.y();
                    p[2] = 0;
                } else {
                    p[0] = positions[v].x();
                    p[1] = positions[v].y();
                    p[2] = 0;
                }
            });
            seams.add_triangles(positions.size() - 1, [&](Index f, lagrange::span<Index> t) {
                t[0] = f;
                t[1] = f + 1;
                t[2] = f + 1;
            });
            return seams;
        };

        wmtk::logger().info("Num curves: {}", boundary.num_curves());
        REQUIRE(boundary.num_curves() == expected_num_curves);

        for (int curve_id = 0; curve_id < static_cast<int>(boundary.num_curves()); ++curve_id) {
            auto parent_id = boundary.parent_curve(curve_id);
            auto curve_mesh = curve_to_mesh(curve_id);
            auto param_mesh = curve_to_mesh(curve_id, true);
            auto parent_mesh = curve_to_mesh(parent_id);
            // lagrange::io::save_mesh(fmt::format("seam_{}.obj", curve_id), curve_mesh);
            // lagrange::io::save_mesh(fmt::format("seam_{}_t.obj", curve_id), param_mesh);
            // lagrange::io::save_mesh(fmt::format("seam_{}_parent.obj", curve_id), parent_mesh);

            for (size_t i = 0; i < boundary.positions(curve_id).size(); ++i) {
                auto expected = boundary.positions(curve_id)[i];
                auto result = boundary.t_to_uv(curve_id, boundary.arclengths(parent_id)[i]);
                for (size_t k = 0; k < expected.size(); ++k) {
                    CAPTURE(curve_id, i, k);
                    REQUIRE_THAT(
                        result[k],
                        Catch::Matchers::WithinRel(expected[k], 1e-5) ||
                            Catch::Matchers::WithinAbs(expected[k], 1e-5));
                }
            }
            if (boundary.is_periodic(curve_id)) {
                auto expected = boundary.positions(curve_id).front();
                auto result = boundary.t_to_uv(curve_id, boundary.arclengths(parent_id).back());
                for (size_t k = 0; k < expected.size(); ++k) {
                    REQUIRE_THAT(
                        result[k],
                        Catch::Matchers::WithinRel(expected[k], 1e-5) ||
                            Catch::Matchers::WithinAbs(expected[k], 1e-5));
                }
            }
        }
    }
};

} // namespace

TEST_CASE("Boundary Parameterization", "[utils][boundary]")
{
    // SECTION("creature")
    // {
    //     BoundaryTester::test_boundary_parameterization(
    //         lagrange::io::load_mesh<lagrange::SurfaceMesh32d>("creature_clean.obj"),
    //         186);
    // }
    SECTION("blub")
    {
        BoundaryTester::test_boundary_parameterization(
            lagrange::io::load_mesh<lagrange::SurfaceMesh32d>(WMTK_DATA_DIR "/blub.obj"),
            18);
    }
    SECTION("blub_open")
    {
        BoundaryTester::test_boundary_parameterization(
            lagrange::io::load_mesh<lagrange::SurfaceMesh32d>(WMTK_DATA_DIR "/blub_open.obj"),
            15);
    }
    SECTION("hemisphere")
    {
        BoundaryTester::test_boundary_parameterization(
            lagrange::io::load_mesh<lagrange::SurfaceMesh32d>(WMTK_DATA_DIR "/hemisphere.obj"),
            39);
    }
    SECTION("hemisphere_splited")
    {
        BoundaryTester::test_boundary_parameterization(
            lagrange::io::load_mesh<lagrange::SurfaceMesh32d>(WMTK_DATA_DIR
                                                              "/hemisphere_splited.obj"),
            39);
    }
    SECTION("pyramid")
    {
        BoundaryTester::test_boundary_parameterization(
            lagrange::io::load_mesh<lagrange::SurfaceMesh32d>(WMTK_DATA_DIR "/seamPyramid.obj"),
            3);
    }
}
