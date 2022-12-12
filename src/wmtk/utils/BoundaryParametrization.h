#pragma once

#include <igl/boundary_loop.h>
#include <igl/project_to_line_segment.h>
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <array>
#include <cmath>
#include "Logger.hpp"
namespace wmtk {
class Boundary
{
public:
    std::vector<std::vector<Eigen::Vector2d>> m_boundaries;
    std::vector<std::vector<double>> m_arclengths;

public:
    /**
     * @brief construct m_boundaries which is a list of boundaries that are each a list of boundary
     * vertices. Also construct m_arclengties, which is a list of cumulative boundary lengths at
     * each vertex from the beginning vertex of each boundary.
     *
     * @param V Matrix of mesh vertices
     * @param F Matrix of mes faces
     */
    void construct_boudaries(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
    Eigen::Vector2d t_to_uv(int i, double t) const;
    double uv_to_t(const Eigen::Vector2d& v) const;
    std::pair<int, int> uv_to_ij(const Eigen::Vector2d& v, double& t) const;
};
} // namespace wmtk