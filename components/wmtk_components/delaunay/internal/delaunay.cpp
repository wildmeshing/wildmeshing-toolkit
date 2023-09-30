#include "delaunay.hpp"

#include <cassert>
#include <mutex>

#include <geogram/delaunay/delaunay.h>

namespace wmtk::components::internal {

std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> delaunay(
    Eigen::Ref<const Eigen::MatrixXd> input_points)
{
    const int dimension = input_points.cols();
    GEO::initialize();

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
    engine->set_vertices(input_points.rows(), input_points.data());

    // Extract output.
    const size_t num_vertices = engine->nb_vertices();
    const size_t num_simplices = engine->nb_cells();

    std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> return_data;
    auto& [vertices, simplices] = return_data;

    vertices.resize(num_vertices, dimension);
    simplices.resize(num_simplices, dimension + 1);

    for (size_t i = 0; i < num_vertices; ++i) {
        // depending on branch prediction to make these calls cost nothing
        if (dimension == 2) {
            vertices.row(i) = Eigen::Map<const Eigen::Vector2d>(engine->vertex_ptr(i)).transpose();
        } else if (dimension == 3) {
            vertices.row(i) = Eigen::Map<const Eigen::Vector3d>(engine->vertex_ptr(i)).transpose();
        } else {
            // for generality allowing for arbitrary dimensions
            vertices.row(i) =
                Eigen::Map<const Eigen::VectorXd>(engine->vertex_ptr(i), dimension).transpose();
        }
    }

    const int dimension_p1 = dimension + 1;
    for (size_t i = 0; i < num_simplices; ++i) {
        for (size_t j = 0; j < dimension_p1; ++j) {
            simplices(i, j) = engine->cell_vertex(i, j);
        }
    }
    return return_data;
}

} // namespace wmtk::components::internal
