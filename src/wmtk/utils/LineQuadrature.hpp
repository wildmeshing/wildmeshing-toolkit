#pragma once
#include <Eigen/Dense>
namespace wmtk {
class LineQuadrature
{
public:
    Eigen::MatrixXd points;
    Eigen::VectorXd weights;

    int size() const
    {
        assert(points.rows() == weights.size());
        return points.rows();
    }

    LineQuadrature(){};

    void get_quadrature(const int order);
};
} // namespace wmtk
