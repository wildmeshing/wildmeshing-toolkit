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

    void get_quadrature(const int order, LineQuadrature& quad);

    template <class T, int order>
    inline std::decay_t<T> eval(
        const Eigen::Matrix<T, 1, 4> edge_verts,
        std::function<T(const T&, const T&)> image_get_z,
        LineQuadrature& quad)
    {
        get_quadrature(order, quad);
        double ret = 0.0;
        auto z1 = image_get_z(edge_verts(0, 0), edge_verts(0, 1));
        auto z2 = image_get_z(edge_verts(0, 2), edge_verts(0, 3));

        // now do 1d quadrature
        for (int i = 0; i < quad.points.rows(); i++) {
            auto tmpu =
                edge_verts(0, 0) + (edge_verts(0, 2) - edge_verts(0, 0)) * quad.points(i, 0);
            auto tmpv =
                edge_verts(0, 1) + (edge_verts(0, 3) - edge_verts(0, 1)) * quad.points(i, 0);
            auto tmpz = image_get_z(tmpu, tmpv);
            ret += quad.weights() * tmpz;
        }
        return ret;
    }
};
} // namespace wmtk
