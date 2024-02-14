#include "delaunay_geogram.hpp"

#include <cassert>
#include <mutex>
#include <wmtk/utils/Logger.hpp>

#include <Delaunay_psm.h>

namespace wmtk::components::internal {

std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> delaunay_geogram(
    Eigen::Ref<const Eigen::MatrixXd> points)
{
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi tetrahedra;

    if (points.rows() == 0) {
        wmtk::logger().warn("Cannot run delaunay on an empty point set");
        return {vertices, tetrahedra};
    }

    int64_t dim = points.cols();

    static std::once_flag once_flag;
    std::call_once(once_flag, []() { GEO::initialize(); });

    GEO::Delaunay_var engine;
    switch (dim) {
    case 2: {
        engine = GEO::Delaunay::create(2, "BDEL2d");
        break;
    }
    case 3: {
        engine = GEO::Delaunay::create(3, "BDEL");
        break;
    }
    default: throw std::runtime_error("unexpected vector type in delaunay_geogram");
    }

    assert(engine);

    // Some settings
    engine->set_reorder(false);
    engine->set_stores_cicl(false); // Incident tetrahedral list.
    engine->set_stores_neighbors(false); // Vertex neighbors.
    engine->set_refine(false);
    engine->set_keeps_infinite(false);

    // Run!
    geo_assert(points.size() > 0);
    Eigen::MatrixXd points_transposed = points.transpose();
    engine->set_vertices(static_cast<GEO::index_t>(points.rows()), points_transposed.data());

    // Extract output.
    vertices.resize(engine->nb_vertices(), dim);
    tetrahedra.resize(engine->nb_cells(), dim + 1);

    for (GEO::index_t i = 0; i < vertices.rows(); ++i) {
        //   depending on branch prediction to make these calls cost nothing
        switch (dim) {
        case 2: {
            vertices.row(i) = Eigen::Map<const Eigen::Vector2d>(engine->vertex_ptr(i)).transpose();
            break;
        }
        case 3: {
            vertices.row(i) = Eigen::Map<const Eigen::Vector3d>(engine->vertex_ptr(i)).transpose();
            break;
        }
            // default: {
            //    // for generality allowing for arbitrary dimensions
            //    vertices.row(i) =
            //        Eigen::Map<const Eigen::VectorXd>(engine->vertex_ptr(i), dim).transpose();
            //    break;
            //}
        }
    }

    for (GEO::index_t i = 0; i < tetrahedra.rows(); ++i) {
        for (GEO::index_t j = 0; j < tetrahedra.cols(); ++j) {
            tetrahedra(i, j) = engine->cell_vertex(i, j);
        }
    }

    return {vertices, tetrahedra};
}


} // namespace wmtk::components::internal