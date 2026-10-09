#include "prism_jacobian.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace wmtk::components::prismatic_mesh {
PrismJacobianValue
prism_jacobian(const MatrixXd& vertices, const HybridCell& prism, const Vector3d& reference_point)
{
    if (prism.vertices.size() != 6) throw std::invalid_argument("Jacobian needs six prism corners");
    using Vec = Eigen::Matrix<long double, 3, 1>;
    std::array<Vec, 6> p;
    for (int i = 0; i < 6; ++i)
        p[i] = vertices.row(prism.vertices[i]).transpose().cast<long double>();
    const long double r = reference_point[0], s = reference_point[1], t = reference_point[2];
    const Vec dr = (1 - t) * (p[1] - p[0]) + t * (p[4] - p[3]);
    const Vec ds = (1 - t) * (p[2] - p[0]) + t * (p[5] - p[3]);
    const Vec dt = (1 - r - s) * (p[3] - p[0]) + r * (p[4] - p[1]) + s * (p[5] - p[2]);
    const Vec cr = ds.cross(dt), cs = dt.cross(dr), ct = dr.cross(ds);
    const std::array<long double, 6> nr = {t - 1, 1 - t, 0, -t, t, 0};
    const std::array<long double, 6> ns = {t - 1, 0, 1 - t, -t, 0, t};
    const std::array<long double, 6> nt = {r + s - 1, -r, -s, 1 - r - s, r, s};
    PrismJacobianValue result;
    result.determinant = static_cast<double>(dr.dot(cr));
    for (int i = 0; i < 6; ++i)
        result.gradients[i] = (nr[i] * cr + ns[i] * cs + nt[i] * ct).cast<double>();
    return result;
}

PrismJacobianMinimum minimum_prism_jacobian(const MatrixXd& vertices, const HybridCell& prism)
{
    PrismJacobianMinimum result;
    result.determinant = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 3; ++i) {
        Vector3d q(i == 1 ? 1 : 0, i == 2 ? 1 : 0, 0);
        const double f0 = prism_jacobian(vertices, prism, q).determinant;
        q[2] = 0.5;
        const double fm = prism_jacobian(vertices, prism, q).determinant;
        q[2] = 1;
        const double f1 = prism_jacobian(vertices, prism, q).determinant;
        if (!std::isfinite(f0) || !std::isfinite(fm) || !std::isfinite(f1)) {
            result.determinant = -std::numeric_limits<double>::infinity();
            result.edge_values[i] = result.determinant;
            result.edge_points[i] = q;
            continue;
        }
        double value = std::min(f0, f1);
        q[2] = f0 <= f1 ? 0 : 1;
        // Fit in extended precision; evaluate the stationary point through the original mapping.
        const long double a =
            2 * (static_cast<long double>(f0) + f1 - 2 * static_cast<long double>(fm));
        const long double b = static_cast<long double>(f1) - f0 - a;
        if (a > 0) {
            const double t = static_cast<double>(-b / (2 * a));
            if (t > 0 && t < 1) {
                Vector3d interior = q;
                interior[2] = t;
                const double f = prism_jacobian(vertices, prism, interior).determinant;
                if (f < value) {
                    value = f;
                    q = interior;
                }
            }
        }
        result.edge_values[i] = value;
        result.edge_points[i] = q;
        result.determinant = std::min(result.determinant, value);
    }
    return result;
}

std::vector<Vector3d> prism_jacobian_sample_points()
{
    std::vector<Vector3d> result;
    for (double t : {0., .5, 1.})
        for (const auto& p : std::array<std::array<double, 2>, 7>{
                 {{0, 0}, {1, 0}, {0, 1}, {.5, 0}, {.5, .5}, {0, .5}, {1. / 3, 1. / 3}}})
            result.emplace_back(p[0], p[1], t);
    return result;
}
} // namespace wmtk::components::prismatic_mesh
