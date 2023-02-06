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

    LineQuadrature();

    void get_quadrature(const int order, LineQuadrature& quad);
    template <class T>
    std::decay_t<T> LineQuadrature::eval(
        const Eigen::Vector2d& a,
        const Eigen::Vector2d& b,
        std::function<Eigen::Vector3d> const LineQuadrature& quad)
    {
        double ret = 0.0;

        return ret;
    }
};
} // namespace wmtk
