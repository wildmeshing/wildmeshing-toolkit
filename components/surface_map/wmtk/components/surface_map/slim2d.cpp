#include "slim2d.hpp"

#include <wmtk/utils/predicates.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>

namespace wmtk::components::surface_map {

namespace {

Eigen::Matrix2d edges(const std::vector<Eigen::Vector2d>& x, const SlimElement& e)
{
    Eigen::Matrix2d X;
    X.col(0) = x[e.v[1]] - x[e.v[0]];
    X.col(1) = x[e.v[2]] - x[e.v[0]];
    return X;
}

/// Whether the element is positively oriented, exactly.
bool positive(const std::vector<Eigen::Vector2d>& x, const SlimElement& e)
{
    return utils::predicates::orient2d(x[e.v[0]], x[e.v[1]], x[e.v[2]]) ==
           utils::predicates::Orientation::POSITIVE;
}

/// The energy of the layout; infinite if an element is not positively oriented.
double energy(const std::vector<Eigen::Vector2d>& x, const std::vector<SlimElement>& elements)
{
    double E = 0;
    for (const SlimElement& e : elements) {
        if (!positive(x, e)) return std::numeric_limits<double>::infinity();
        if (e.weight == 0) continue;
        const Eigen::Matrix2d X = edges(x, e);
        const Eigen::Matrix2d J = X * e.rest_inv;
        const double det = J.determinant(); // |J^-1|^2 = |J|^2 / det^2 for a 2x2
        E += e.weight * J.squaredNorm() * (1 + 1 / (det * det));
    }
    return E;
}

/// The largest step along `d` before an element's edge determinant reaches zero, in floating
/// point: only where the line search starts, which checks every step exactly.
double max_step(
    const std::vector<Eigen::Vector2d>& x,
    const std::vector<Eigen::Vector2d>& d,
    const std::vector<SlimElement>& elements)
{
    double t_max = std::numeric_limits<double>::infinity();
    for (const SlimElement& e : elements) {
        const Eigen::Matrix2d X = edges(x, e), D = edges(d, e);
        // det(X + t D) = a t^2 + b t + c
        const double a = D.determinant();
        const double b =
            X(0, 0) * D(1, 1) + D(0, 0) * X(1, 1) - X(0, 1) * D(1, 0) - D(0, 1) * X(1, 0);
        const double c = X.determinant();
        double t = std::numeric_limits<double>::infinity();
        if (std::abs(a) < 1e-300) {
            if (b < 0) t = -c / b;
        } else {
            const double disc = b * b - 4 * a * c;
            if (disc >= 0) {
                const double s = std::sqrt(disc);
                for (const double r : {(-b - s) / (2 * a), (-b + s) / (2 * a)}) {
                    if (r > 0) t = std::min(t, r);
                }
            }
        }
        t_max = std::min(t_max, t);
    }
    return t_max;
}

} // namespace

SlimElement slim_element(
    const std::array<int, 3>& v,
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2,
    const double weight)
{
    SlimElement e;
    e.v = v;
    const Eigen::Vector3d e1 = p1 - p0, e2 = p2 - p0, n = e1.cross(e2);
    const double area = 0.5 * n.norm();
    if (!(area > 0) || !(e1.norm() > 0)) return e; // weight 0: kept from turning over only
    const Eigen::Vector3d u = e1.normalized(), w = n.cross(e1).normalized();
    Eigen::Matrix2d R;
    R << e1.norm(), e2.dot(u), 0, e2.dot(w);
    e.rest_inv = R.inverse();
    e.weight = weight * area;
    return e;
}

SlimElement slim_element(
    const std::array<int, 3>& v,
    const Eigen::Vector2d& q0,
    const Eigen::Vector2d& q1,
    const Eigen::Vector2d& q2,
    const double weight)
{
    SlimElement e;
    e.v = v;
    Eigen::Matrix2d R;
    R.col(0) = q1 - q0;
    R.col(1) = q2 - q0;
    if (!(R.determinant() > 0)) return e;
    e.rest_inv = R.inverse();
    e.weight = weight;
    return e;
}

double slim(
    std::vector<Eigen::Vector2d>& x,
    const std::vector<char>& fixed,
    const std::vector<SlimElement>& elements,
    const int iterations)
{
    const int n = int(x.size());
    std::vector<int> free_index(n, -1);
    int n_free = 0;
    for (int i = 0; i < n; ++i) {
        if (!fixed[i]) free_index[i] = n_free++;
    }
    double E = energy(x, elements);
    if (n_free == 0 || !std::isfinite(E)) return E;

    // vec(W X D) = (D^T kron W) vec(X), X = [x0 x1 x2] column by column.
    Eigen::Matrix<double, 3, 2> G;
    G << -1, -1, 1, 0, 0, 1;
    for (int it = 0; it < iterations; ++it) {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2 * n_free, 2 * n_free);
        Eigen::VectorXd g = Eigen::VectorXd::Zero(2 * n_free);
        for (const SlimElement& e : elements) {
            if (e.weight == 0) continue;
            const Eigen::Matrix2d J = edges(x, e) * e.rest_inv;
            // The 2x2 SVD in closed form (Blinn 1996), J = Rot(phi) diag(s) Rot(theta), with phi
            // half the sum of the angles of (E, H) and (F, G): every element is positive here, so
            // s[1] > 0, and Rot(phi + theta) -- (E, H) normalized -- is the closest rotation. U
            // only enters W = U diag(w) U^T, which does not see its sign, so phi may be off by pi.
            const double E2 = (J(0, 0) + J(1, 1)) / 2, F2 = (J(0, 0) - J(1, 1)) / 2;
            const double G2 = (J(1, 0) + J(0, 1)) / 2, H2 = (J(1, 0) - J(0, 1)) / 2;
            const double Q = std::hypot(E2, H2), R = std::hypot(F2, G2);
            const Eigen::Vector2d s(Q + R, Q - R);
            Eigen::Matrix2d U = Eigen::Matrix2d::Identity(), Rot;
            if (R > 0) { // else s[0] == s[1] and any U does
                const double c = (F2 * E2 - G2 * H2) / (Q * R), sn = (F2 * H2 + G2 * E2) / (Q * R);
                const double ch = std::sqrt(std::max(0., (1 + c) / 2));
                const double sh = std::copysign(std::sqrt(std::max(0., (1 - c) / 2)), sn);
                U << ch, -sh, sh, ch;
            }
            Rot << E2 / Q, -H2 / Q, H2 / Q, E2 / Q;
            // SLIM's proxy weights for the symmetric Dirichlet energy, sqrt(f'(s) / 2(s - 1)).
            Eigen::Vector2d w;
            for (int k = 0; k < 2; ++k) {
                const double sk = std::max(std::abs(s[k]), 1e-12);
                w[k] = std::sqrt((sk + 1) * (sk * sk + 1) / (sk * sk * sk));
            }
            const Eigen::Matrix2d W = U * w.asDiagonal() * U.transpose();
            const Eigen::Matrix<double, 3, 2> D = G * e.rest_inv;
            Eigen::Matrix<double, 4, 6> M;
            for (int r = 0; r < 2; ++r) { // columns of the result
                for (int c = 0; c < 3; ++c) { // vertices
                    M.block<2, 2>(2 * r, 2 * c) = D(c, r) * W;
                }
            }
            const Eigen::Matrix2d WR = W * Rot;
            const Eigen::Vector4d t(WR(0, 0), WR(1, 0), WR(0, 1), WR(1, 1));
            // Fixed vertices go to the right-hand side.
            Eigen::Matrix<double, 6, 1> xc;
            for (int c = 0; c < 3; ++c) xc.segment<2>(2 * c) = x[e.v[c]];
            Eigen::Vector4d rhs = t;
            for (int c = 0; c < 3; ++c) {
                if (fixed[e.v[c]]) rhs -= M.block<4, 2>(0, 2 * c) * xc.segment<2>(2 * c);
            }
            const Eigen::Matrix<double, 6, 6> MtM = e.weight * M.transpose() * M;
            const Eigen::Matrix<double, 6, 1> Mtr = e.weight * M.transpose() * rhs;
            for (int a = 0; a < 3; ++a) {
                const int fa = free_index[e.v[a]];
                if (fa < 0) continue;
                g.segment<2>(2 * fa) += Mtr.segment<2>(2 * a);
                for (int b = 0; b < 3; ++b) {
                    const int fb = free_index[e.v[b]];
                    if (fb < 0) continue;
                    H.block<2, 2>(2 * fa, 2 * fb) += MtM.block<2, 2>(2 * a, 2 * b);
                }
            }
        }
        const Eigen::VectorXd sol = H.ldlt().solve(g);
        if (!sol.allFinite()) break;
        std::vector<Eigen::Vector2d> d(n, Eigen::Vector2d::Zero());
        for (int i = 0; i < n; ++i) {
            if (free_index[i] >= 0) d[i] = sol.segment<2>(2 * free_index[i]) - x[i];
        }
        double step = std::min(1., 0.8 * max_step(x, d, elements));
        bool improved = false;
        for (int k = 0; k < 30 && step > 1e-12; ++k, step *= 0.5) {
            std::vector<Eigen::Vector2d> y = x;
            for (int i = 0; i < n; ++i) y[i] += step * d[i];
            const double Ey = energy(y, elements);
            if (Ey < E) {
                x = std::move(y);
                improved = E - Ey > 1e-4 * E;
                E = Ey;
                break;
            }
        }
        if (!improved) break;
    }
    return E;
}

} // namespace wmtk::components::surface_map
