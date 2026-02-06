#include "Delaunay.hpp"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <Delaunay_psm.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <igl/predicates/predicates.h>


#include <cassert>
#include <mutex>

namespace {
bool is_inverted(
    const wmtk::delaunay::Point3D& p0,
    const wmtk::delaunay::Point3D& p1,
    const wmtk::delaunay::Point3D& p2,
    const wmtk::delaunay::Point3D& p3)
{
    const Eigen::Vector3d v0(p0[0], p0[1], p0[2]);
    const Eigen::Vector3d v1(p1[0], p1[1], p1[2]);
    const Eigen::Vector3d v2(p2[0], p2[1], p2[2]);
    const Eigen::Vector3d v3(p3[0], p3[1], p3[2]);

    igl::predicates::exactinit();
    auto res = igl::predicates::orient3d(v0, v1, v2, v3);
    int result;
    if (res == igl::predicates::Orientation::POSITIVE)
        result = 1;
    else if (res == igl::predicates::Orientation::NEGATIVE)
        result = -1;
    else
        result = 0;

    if (result < 0) // neg result == pos tet (tet origin from geogram delaunay)
        return false;
    return true;
}

bool is_inverted(
    const wmtk::delaunay::Point2D& p0,
    const wmtk::delaunay::Point2D& p1,
    const wmtk::delaunay::Point2D& p2)
{
    const Eigen::Vector2d v0(p0[0], p0[1]);
    const Eigen::Vector2d v1(p1[0], p1[1]);
    const Eigen::Vector2d v2(p2[0], p2[1]);

    igl::predicates::exactinit();
    auto res = igl::predicates::orient2d(v0, v1, v2);
    int result;
    if (res == igl::predicates::Orientation::POSITIVE)
        result = 1;
    else if (res == igl::predicates::Orientation::NEGATIVE)
        result = -1;
    else
        result = 0;

    if (result < 0) // neg result == pos tet (tet origin from geogram delaunay)
        return true;
    return false;
}
} // namespace

namespace wmtk::delaunay {

auto delaunay3D(const std::vector<Point3D>& points)
    -> std::pair<std::vector<Point3D>, std::vector<Tetrahedron>>
{
    static std::once_flag once_flag;
    std::call_once(once_flag, []() { GEO::initialize(); });

    GEO::Delaunay_var engine = GEO::Delaunay::create(3, "BDEL");
    assert(engine);

    // Some settings
    engine->set_reorder(true); // making this true fixed a bug for whatever reason
    engine->set_stores_cicl(false); // Incident tetrahedral list.
    engine->set_stores_neighbors(false); // Vertex neighbors.
    engine->set_refine(false);
    engine->set_keeps_infinite(false);

    // Run!
    geo_assert(points.size() > 0);
    engine->set_vertices((GEO::index_t)points.size(), points.front().data());

    // Extract output.
    const size_t num_vertices = engine->nb_vertices();
    const size_t num_tets = engine->nb_cells();
    std::vector<Point3D> vertices(num_vertices);
    std::vector<Tetrahedron> tets(num_tets);

    for (GEO::index_t i = 0; i < num_vertices; i++) {
        for (size_t j = 0; j < 3; j++) {
            vertices[i][j] = engine->vertex_ptr(i)[j];
        }
    }

    for (GEO::index_t i = 0; i < num_tets; i++) {
        for (GEO::index_t j = 0; j < 4; j++) {
            tets[i][j] = engine->cell_vertex(i, j);
        }
    }

    // sort tets
    for (auto& tri : tets) {
        std::sort(tri.begin(), tri.end());
    }
    std::sort(tets.begin(), tets.end());

    for (auto& tri : tets) {
        const Point3D p0 = points[tri[0]];
        const Point3D p1 = points[tri[1]];
        const Point3D p2 = points[tri[2]];
        const Point3D p3 = points[tri[3]];
        if (is_inverted(p0, p1, p2, p3)) {
            // std::cout << "Inverted tet found" << std::endl;
            std::swap(tri[2], tri[3]); // invert tet
        }
    }

    return {vertices, tets};
}

auto delaunay2D(const std::vector<Point2D>& points)
    -> std::pair<std::vector<Point2D>, std::vector<Triangle>>
{
    static std::once_flag once_flag;
    std::call_once(once_flag, []() { GEO::initialize(); });

    GEO::Delaunay_var engine = GEO::Delaunay::create(2, "BDEL2d");
    assert(engine);

    // Some settings
    engine->set_reorder(true);
    engine->set_stores_cicl(false); // Incident tetrahedral list.
    engine->set_stores_neighbors(false); // Vertex neighbors.
    engine->set_refine(false);
    engine->set_keeps_infinite(false);

    // Run!
    geo_assert(points.size() > 0);
    engine->set_vertices((GEO::index_t)points.size(), points.front().data());

    // Extract output.
    const size_t num_vertices = engine->nb_vertices();
    const size_t num_triangles = engine->nb_cells();
    std::vector<Point2D> vertices(num_vertices);
    std::vector<Triangle> triangles(num_triangles);

    for (GEO::index_t i = 0; i < num_vertices; i++) {
        for (GEO::index_t j = 0; j < 2; j++) {
            vertices[i][j] = engine->vertex_ptr(i)[j];
        }
    }

    for (GEO::index_t i = 0; i < num_triangles; i++) {
        for (GEO::index_t j = 0; j < 3; j++) {
            triangles[i][j] = engine->cell_vertex(i, j);
        }
    }

    // sort tets
    for (auto& tri : triangles) {
        std::sort(tri.begin(), tri.end());
    }
    std::sort(triangles.begin(), triangles.end());

    for (auto& tri : triangles) {
        const Point2D p0 = points[tri[0]];
        const Point2D p1 = points[tri[1]];
        const Point2D p2 = points[tri[2]];
        if (is_inverted(p0, p1, p2)) {
            // std::cout << "Inverted tet found" << std::endl;
            std::swap(tri[1], tri[2]); // invert triangle
        }
    }

    return {vertices, triangles};
}

} // namespace wmtk::delaunay
