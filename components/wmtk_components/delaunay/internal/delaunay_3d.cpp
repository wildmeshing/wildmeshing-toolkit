#include "delaunay_3d.hpp"

#include <cassert>
#include <mutex>

#include <geogram/delaunay/delaunay.h>

namespace wmtk::components::internal {

void delaunay_3d(
    const std::vector<Eigen::Vector3d>& input_points,
    Eigen::MatrixXd& vertices,
    Eigen::MatrixXi& tetrahedra)
{
    static std::once_flag once_flag;
    std::call_once(once_flag, []() { GEO::initialize(); });

    GEO::Delaunay_var engine = GEO::Delaunay::create(3, "BDEL");
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
    const size_t num_tets = engine->nb_cells();

    vertices.resize(num_vertices, 3);
    tetrahedra.resize(num_tets, 4);

    for (size_t i = 0; i < num_vertices; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            vertices(i, j) = engine->vertex_ptr(i)[j];
        }
    }

    for (size_t i = 0; i < num_tets; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            tetrahedra(i, j) = engine->cell_vertex(i, j);
        }
    }
}

} // namespace wmtk::components::internal