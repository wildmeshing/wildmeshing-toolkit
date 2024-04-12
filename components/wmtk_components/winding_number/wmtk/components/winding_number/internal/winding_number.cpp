#include "winding_number.hpp"

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/utils/EigenMatrixWriter.hpp>

#include <igl/bfs_orient.h>
#include <igl/fast_winding_number.h>
#include <igl/winding_number.h>
#include <wmtk/utils/Rational.hpp>

namespace wmtk::components::internal {

Eigen::VectorXd winding_number(const Mesh& m, const TriMesh& surface)
{
    wmtk::utils::EigenMatrixWriter m_writer, surface_writer;
    m.serialize(m_writer);
    surface.serialize(surface_writer);

    Eigen::MatrixX<Rational> m_pos_rational, surface_pos_rational;
    Eigen::MatrixXd m_pos, surface_pos;
    MatrixX<int64_t> m_FV, surface_FV;

    m_writer.get_position_matrix(m_pos_rational);
    assert(m_pos_rational.cols() == 3);
    m_pos.resize(m_pos_rational.rows(), m_pos_rational.cols());
    // for (int64_t i = 0; i < m_pos_rational.rows(); ++i) {
    //     for (int64_t j = 0; j < m_pos_rational.cols(); ++j) {
    //         m_pos(i, j) = m_pos_rational(i, j).to_double();
    //     }
    // }
    m_pos = m_pos_rational.cast<double>();
    surface_writer.get_position_matrix(surface_pos_rational);
    surface_pos.resize(surface_pos_rational.rows(), surface_pos_rational.cols());
    surface_pos = surface_pos_rational.cast<double>();

    switch (m.top_simplex_type()) {
    case (PrimitiveType::Tetrahedron): {
        m_writer.get_TV_matrix(m_FV);
        break;
    }
    case (PrimitiveType::Triangle): {
        m_writer.get_FV_matrix(m_FV);
        break;
    }
    case (PrimitiveType::Edge): {
        m_writer.get_EV_matrix(m_FV);
        break;
    }
    default: throw std::runtime_error("Unsupported Mesh Type");
    }

    surface_writer.get_FV_matrix(surface_FV);

    Eigen::MatrixXd centroids;
    centroids.resize(m_FV.rows(), 3);

    for (int64_t i = 0; i < m_FV.rows(); ++i) {
        Eigen::Vector3d centroid(0, 0, 0);
        for (int64_t j = 0; j < m_FV.cols(); ++j) {
            Vector3d v_pos = m_pos.row(m_FV(i, j));
            centroid = centroid + v_pos;
        }
        centroids.row(i) = centroid / m_FV.cols();
    }

    // correct the orientation of the surface mesh
    MatrixX<int64_t> _C;
    igl::bfs_orient(surface_FV, surface_FV, _C);

    Eigen::VectorXd winding_numbers = winding_number_internal(centroids, surface_pos, surface_FV);
    return winding_numbers;
}

Eigen::VectorXd winding_number_internal(
    const Eigen::MatrixXd& query_points,
    const Eigen::MatrixXd& surface_V,
    const MatrixX<int64_t>& surface_F)
{
    Eigen::VectorXd winding_numbers;
    igl::fast_winding_number(surface_V, surface_F, query_points, winding_numbers);
    return winding_numbers;
}


} // namespace wmtk::components::internal
