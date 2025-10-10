#include "Delaunay.hpp"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <geogram/delaunay/delaunay.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on


#include <cassert>
#include <mutex>

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
    engine->set_vertices(points.size(), points.front().data());

    // Extract output.
    const size_t num_vertices = engine->nb_vertices();
    const size_t num_tets = engine->nb_cells();
    std::vector<Point3D> vertices(num_vertices);
    std::vector<Tetrahedron> tets(num_tets);

    for (size_t i = 0; i < num_vertices; i++) {
        for (auto j = 0; j < 3; j++) vertices[i][j] = engine->vertex_ptr(i)[j];
    }

    for (size_t i = 0; i < num_tets; i++) {
        for (auto j = 0; j < 4; j++) tets[i][j] = engine->cell_vertex(i, j);
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
    engine->set_vertices(points.size(), points.front().data());

    // Extract output.
    const size_t num_vertices = engine->nb_vertices();
    const size_t num_triangles = engine->nb_cells();
    std::vector<Point2D> vertices(num_vertices);
    std::vector<Triangle> triangles(num_triangles);

    for (size_t i = 0; i < num_vertices; i++) {
        for (auto j = 0; j < 2; j++) vertices[i][j] = engine->vertex_ptr(i)[j];
    }

    for (size_t i = 0; i < num_triangles; i++) {
        for (auto j = 0; j < 3; j++) triangles[i][j] = engine->cell_vertex(i, j);
    }

    return {vertices, triangles};
}

} // namespace wmtk::delaunay
