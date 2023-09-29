#include "delaunay_geogram.hpp"

#include <cassert>
#include <mutex>
#include <wmtk/utils/Logger.hpp>

#include <geogram/delaunay/delaunay.h>

namespace wmtk::components::internal {

template <typename VectorT>
std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> delaunay_geogram(
    const std::vector<VectorT>& input_points)
{
    static_assert(
        std::is_same<VectorT, Eigen::Vector2d>() || std::is_same<VectorT, Eigen::Vector3d>());

    Eigen::MatrixXd vertices;
    Eigen::MatrixXi tetrahedra;

    if (input_points.empty()) {
        wmtk::logger().warn("Cannot run delaunay on an empty point set");
        return {vertices, tetrahedra};
    }

    long dim = VectorT().rows();
    assert(dim == 2 || dim == 3);
    for (const auto& p : input_points) {
        assert(dim == p.rows());
    }

    static std::once_flag once_flag;
    std::call_once(once_flag, []() { GEO::initialize(); });

    GEO::Delaunay_var engine;
    if constexpr (std::is_same<VectorT, Eigen::Vector2d>()) {
        engine = GEO::Delaunay::create(2, "BDEL2d");
    } else if constexpr (std::is_same<VectorT, Eigen::Vector3d>()) {
        engine = GEO::Delaunay::create(3, "BDEL");
    } else {
        throw "unexpected vector type in delaunay_geogram";
    }

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
    vertices.resize(engine->nb_vertices(), dim);
    tetrahedra.resize(engine->nb_cells(), dim + 1);

    for (GEO::index_t i = 0; i < vertices.rows(); ++i) {
        vertices.row(i) = Eigen::Map<const VectorT>(engine->vertex_ptr(i));
    }

    for (GEO::index_t i = 0; i < tetrahedra.rows(); ++i) {
        for (GEO::index_t j = 0; j < tetrahedra.cols(); ++j) {
            tetrahedra(i, j) = engine->cell_vertex(i, j);
        }
    }

    return {vertices, tetrahedra};
}

template std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> delaunay_geogram(
    const std::vector<Eigen::Vector2d>&);
template std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> delaunay_geogram(
    const std::vector<Eigen::Vector3d>&);

} // namespace wmtk::components::internal