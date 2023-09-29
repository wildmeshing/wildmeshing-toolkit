#include "delaunay_3d.hpp"

#include <cassert>
#include <mutex>

#include <geogram/delaunay/delaunay.h>

namespace wmtk::components::internal {

std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> delaunay_3d(
    const std::vector<Eigen::Vector3d>& input_points)
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
    engine->set_vertices(
        static_cast<GEO::index_t>(input_points.size()),
        input_points.front().data());

    // Extract output.
    const GEO::index_t num_vertices = engine->nb_vertices();
    const GEO::index_t num_tets = engine->nb_cells();

    Eigen::MatrixXd vertices;
    Eigen::MatrixXi tetrahedra;
    vertices.resize(num_vertices, 3);
    tetrahedra.resize(num_tets, 4);

    for (GEO::index_t i = 0; i < num_vertices; ++i) {
        vertices.row(i) = Eigen::Map<const Eigen::Vector3d>(engine->vertex_ptr(i));
    }

    for (GEO::index_t i = 0; i < num_tets; ++i) {
        for (GEO::index_t j = 0; j < 4; ++j) {
            tetrahedra(i, j) = engine->cell_vertex(i, j);
        }
    }

    return {vertices, tetrahedra};
}

} // namespace wmtk::components::internal