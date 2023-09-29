#include "delaunay_2d.hpp"

#include <cassert>
#include <mutex>

#include <geogram/delaunay/delaunay.h>

namespace wmtk::components::internal {

void delaunay_2d(
    const std::vector<Eigen::Vector2d>& input_points,
    Eigen::MatrixXd& vertices,
    Eigen::MatrixXi& triangles)
{
    static std::once_flag once_flag;
    std::call_once(once_flag, []() { GEO::initialize(); });

    GEO::Delaunay_var engine = GEO::Delaunay::create(2, "BDEL2d");
    assert(engine);

    // Some settings
    engine->set_reorder(false);
    engine->set_stores_cicl(false); // Incident tetrahedral list.
    engine->set_stores_neighbors(false); // Vertex neighbors.
    engine->set_refine(false);
    engine->set_keeps_infinite(false);

    // Run!
    geo_assert(input_points.size() > 0);
    engine->set_vertices(input_points.size(), input_points.front().data());

    // Extract output.
    const size_t num_vertices = engine->nb_vertices();
    const size_t num_triangles = engine->nb_cells();

    vertices.resize(num_vertices, 2);
    triangles.resize(num_triangles, 3);

    for (size_t i = 0; i < num_vertices; ++i) {
        for (size_t j = 0; j < 2; ++j) {
            vertices(i, j) = engine->vertex_ptr(i)[j];
        }
    }

    for (size_t i = 0; i < num_triangles; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            triangles(i, j) = engine->cell_vertex(i, j);
        }
    }
}

} // namespace wmtk::components::internal