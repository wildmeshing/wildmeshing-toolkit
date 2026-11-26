#pragma once
#include <algorithm>
#include <nlohmann/json.hpp>

#include <wmtk/utils/AMIPS.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <array>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include <wmtk/utils/Rational.hpp>

namespace wmtk::components::prismatic_mesh::quality::native {
using Json = nlohmann::json;
using V = Eigen::Vector3d;
using M = Eigen::Matrix3d;
using Rat = wmtk::Rational;
struct Cell
{
    int type = 4, region = 3;
    std::vector<size_t> v;
    double energy = 0, minimum = 0;
};
inline const std::map<int, std::vector<std::vector<int>>> faces{
    {4, {{1, 2, 3}, {0, 3, 2}, {0, 1, 3}, {0, 2, 1}}},
    {6, {{0, 2, 1}, {3, 4, 5}, {0, 1, 4, 3}, {1, 2, 5, 4}, {2, 0, 3, 5}}},
    {7, {{0, 3, 2, 1}, {0, 1, 4}, {1, 2, 4}, {2, 3, 4}, {3, 0, 4}}}};
inline M tet_reference()
{
    M w;
    w << 1, .5, .5, 0, std::sqrt(3.) / 2, std::sqrt(3.) / 6, 0, 0, std::sqrt(2. / 3);
    return w;
}
inline M prism_inverse()
{
    M w;
    w << 1, .5, 0, 0, std::sqrt(3.) / 2, 0, 0, 0, 1;
    return w.inverse();
}
inline double determinant(const std::vector<V>& p, const std::vector<size_t>& c)
{
    const V a = p[c[1]] - p[c[0]], b = p[c[2]] - p[c[0]], d = p[c[3]] - p[c[0]];
    const double det = a.dot(b.cross(d)), scale = std::max({a.norm(), b.norm(), d.norm()});
    if (std::abs(det) > 1e-12 * scale * scale * scale) return det;
    std::array<Rat, 3> ar, br, dr;
    for (int j = 0; j < 3; ++j) {
        ar[j] = Rat(p[c[1]][j]) - Rat(p[c[0]][j]);
        br[j] = Rat(p[c[2]][j]) - Rat(p[c[0]][j]);
        dr[j] = Rat(p[c[3]][j]) - Rat(p[c[0]][j]);
    }
    return (ar[0] * (br[1] * dr[2] - br[2] * dr[1]) - ar[1] * (br[0] * dr[2] - br[2] * dr[0]) +
            ar[2] * (br[0] * dr[1] - br[1] * dr[0]))
        .to_double();
}
inline double tetra_energy(const std::vector<V>& p, const std::vector<size_t>& c)
{
    std::array<double, 12> q;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 3; ++j) q[3 * i + j] = p[c[i]][j];
    return std::cbrt(wmtk::AMIPS_energy_stable_p3<Rat>(q));
}
struct Q
{
    double r, s, t, w;
    std::vector<V> g;
};
inline const std::vector<Q>& quadrature(int type, int order)
{
    static thread_local std::map<std::pair<int, int>, std::vector<Q>> cache;
    const auto key = std::make_pair(type, order);
    auto found = cache.find(key);
    if (found != cache.end()) return found->second;
    std::vector<Q> result;
    if (type == 4) {
        const M inv = tet_reference().inverse();
        Q q{0, 0, 0, 1, {}};
        for (const V v : {V(-1, -1, -1), V(1, 0, 0), V(0, 1, 0), V(0, 0, 1)})
            q.g.push_back(inv.transpose() * v);
        result.push_back(q);
    } else {
        Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(order, order);
        for (int k = 1; k < order; ++k)
            jac(k - 1, k) = jac(k, k - 1) = k / std::sqrt(4. * k * k - 1);
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(jac);
        const auto z = eig.eigenvalues();
        const Eigen::RowVectorXd w = eig.eigenvectors().row(0).array().square().matrix();
        const M inv = prism_inverse();
        for (int a = 0; a < order; ++a)
            for (int b = 0; b < order; ++b)
                for (int h = 0; h < (type == 6 ? order : 1); ++h) {
                    const double u = (z[a] + 1) / 2, v = (z[b] + 1) / 2,
                                 t = type == 6 ? (z[h] + 1) / 2 : 0;
                    Q q{type == 6 ? u : z[a],
                        type == 6 ? (1 - u) * v : z[b],
                        t,
                        type == 6 ? 2 * w[a] * w[b] * w[h] * (1 - u) : w[a] * w[b],
                        {}};
                    const auto r = q.r, s = q.s;
                    if (type == 6) {
                        for (const V g :
                             {V(-(1 - t), -(1 - t), -(1 - r - s)),
                              V(1 - t, 0, -r),
                              V(0, 1 - t, -s),
                              V(-t, -t, 1 - r - s),
                              V(t, 0, r),
                              V(0, t, s)})
                            q.g.push_back(inv.transpose() * g);
                    } else {
                        const double bb[5] = {-.25, .25, .25, -.25, 0},
                                     cc[5] = {-.25, -.25, .25, .25, 0},
                                     dd[5] = {.25, -.25, .25, -.25, 0},
                                     ee[5] = {-.25, -.25, -.25, -.25, 1};
                        for (int j = 0; j < 5; ++j)
                            q.g.emplace_back(
                                2 * (bb[j] + dd[j] * s),
                                2 * (cc[j] + dd[j] * r),
                                std::sqrt(2.) * (ee[j] + dd[j] * r * s));
                    }
                    result.push_back(q);
                }
    }
    return cache.emplace(key, std::move(result)).first->second;
}
inline M matrix(const std::vector<V>& p, const Cell& c, const Q& q)
{
    M a = M::Zero();
    for (size_t j = 0; j < c.v.size(); ++j)
        a.noalias() += (p[c.v[j]] - p[c.v[0]]) * q.g[j].transpose();
    return a;
}
// Rational fallback for nearly singular hybrid elements. Positive reference
// normalization factors can be applied after locating the exact minimum.
inline double hybrid_minimum_rational(const std::vector<V>& p, const Cell& c)
{
    using RV = std::array<Rat, 3>;
    std::vector<RV> v(c.v.size());
    for (size_t i = 0; i < v.size(); ++i)
        for (int j = 0; j < 3; ++j) v[i][j] = Rat(p[c.v[i]][j]);
    auto det = [](const RV& a, const RV& b, const RV& d) {
        return a[0] * (b[1] * d[2] - b[2] * d[1]) - a[1] * (b[0] * d[2] - b[2] * d[0]) +
               a[2] * (b[0] * d[1] - b[1] * d[0]);
    };
    Rat minimum;
    bool first = true;
    auto add = [&](const Rat& d) {
        if (first || d < minimum) {
            minimum = d;
            first = false;
        }
    };
    if (c.type == 6) {
        for (int corner = 0; corner < 3; ++corner) {
            std::array<Rat, 3> values;
            for (int sample = 0; sample < 3; ++sample) {
                const Rat t(sample * .5);
                RV a, b, d;
                for (int j = 0; j < 3; ++j) {
                    a[j] = (1 - t) * (v[1][j] - v[0][j]) + t * (v[4][j] - v[3][j]);
                    b[j] = (1 - t) * (v[2][j] - v[0][j]) + t * (v[5][j] - v[3][j]);
                    d[j] = v[corner + 3][j] - v[corner][j];
                }
                values[sample] = det(a, b, d);
            }
            add(values[0]);
            add(values[2]);
            const Rat a = 2 * (values[0] + values[2] - 2 * values[1]),
                      b = values[2] - values[0] - a;
            if (a > 0 && -b > 0 && -b < 2 * a) add(values[0] - b * b / (4 * a));
        }
        return minimum.to_double() * 2 / std::sqrt(3.);
    }
    for (int r : {-1, 1})
        for (int s : {-1, 1}) {
            RV a, b, d;
            for (int j = 0; j < 3; ++j) {
                const Rat center = (v[0][j] + v[1][j] + v[2][j] + v[3][j]) / 4;
                const Rat u = (-v[0][j] + v[1][j] + v[2][j] - v[3][j]) / 4,
                          w = (-v[0][j] - v[1][j] + v[2][j] + v[3][j]) / 4;
                const Rat cross = (v[0][j] - v[1][j] + v[2][j] - v[3][j]) / 4;
                a[j] = 2 * (u + s * cross);
                b[j] = 2 * (w + r * cross);
                d[j] = v[4][j] - center + r * s * cross;
            }
            add(det(a, b, d));
        }
    return minimum.to_double() * std::sqrt(2.);
}
inline double hybrid_minimum(const std::vector<V>& p, const Cell& c)
{
    double result = std::numeric_limits<double>::infinity();
    if (c.type == 6) {
        for (const auto rs : {std::array<double, 2>{0, 0}, {1, 0}, {0, 1}}) {
            const double r = rs[0], s = rs[1];
            double dets[3];
            for (int j = 0; j < 3; ++j) {
                const double t = j * .5;
                M a;
                a.col(0) = (1 - t) * (p[c.v[1]] - p[c.v[0]]) + t * (p[c.v[4]] - p[c.v[3]]);
                a.col(1) = (1 - t) * (p[c.v[2]] - p[c.v[0]]) + t * (p[c.v[5]] - p[c.v[3]]);
                a.col(2) = (1 - r - s) * (p[c.v[3]] - p[c.v[0]]) + r * (p[c.v[4]] - p[c.v[1]]) +
                           s * (p[c.v[5]] - p[c.v[2]]);
                dets[j] = (a * prism_inverse()).determinant();
            }
            result = std::min({result, dets[0], dets[2]});
            const double a = 2 * (dets[0] + dets[2] - 2 * dets[1]), b = dets[2] - dets[0] - a;
            if (a > 0 && -b > 0 && -b < 2 * a) result = std::min(result, dets[0] - b * b / (4 * a));
        }
    } else {
        for (double r : {-1., 1.})
            for (double s : {-1., 1.}) {
                const V p1 = p[c.v[1]] - p[c.v[0]], p2 = p[c.v[2]] - p[c.v[0]],
                        p3 = p[c.v[3]] - p[c.v[0]];
                const V a = (p1 + p2 + p3) / 4, b = (p1 + p2 - p3) / 4, d = (-p1 + p2 - p3) / 4,
                        e = (-p1 + p2 + p3) / 4;
                M m;
                m.col(0) = 2 * (b + s * d);
                m.col(1) = 2 * (e + r * d);
                m.col(2) = std::sqrt(2.) * (p[c.v[4]] - p[c.v[0]] - a + r * s * d);
                result = std::min(result, m.determinant());
            }
    }
    double scale = 0;
    for (size_t v : c.v) scale = std::max(scale, (p[v] - p[c.v[0]]).norm());
    if (!std::isfinite(result) || std::abs(result) <= 1e-10 * scale * scale * scale)
        return hybrid_minimum_rational(p, c);
    return result;
}
inline double quality(const std::vector<V>& p, Cell& c, int order = 4)
{
    c.minimum = c.type == 4 ? determinant(p, c.v) : hybrid_minimum(p, c);
    if (!(c.minimum > 0)) return c.energy = std::numeric_limits<double>::infinity();
    if (c.type == 4) return c.energy = tetra_energy(p, c.v);
    double value = 0;
    for (const auto& q : quadrature(c.type, order)) {
        const M a = matrix(p, c, q);
        const double d = a.determinant();
        if (!(d > 0)) return c.energy = std::numeric_limits<double>::infinity();
        value += q.w * a.squaredNorm() / std::pow(d, 2. / 3);
    }
    return c.energy = value;
}

struct Mesh
{
    std::vector<V> p, initial, base;
    std::vector<double> thickness, radius, sizing;
    std::vector<uint8_t> input, outer, shell;
    std::vector<uint8_t> fixed_vertices;
    std::vector<int64_t> correspondence;
    std::vector<Cell> cells, source_shell, source_input;
    std::vector<std::vector<size_t>> star;
    size_t original_n = 0;
    V lo, hi;
    double target_length = 0;

    bool fixed(size_t v) const { return v < fixed_vertices.size() && fixed_vertices[v]; }
    bool fixed(const Cell& c) const
    {
        return std::all_of(c.v.begin(), c.v.end(), [&](size_t v) { return fixed(v); });
    }

    void rebuild()
    {
        star.assign(p.size(), {});
        shell.assign(p.size(), false);
        for (size_t i = 0; i < cells.size(); ++i)
            for (size_t v : cells[i].v) {
                star[v].push_back(i);
                if (cells[i].region == 2) shell[v] = true;
            }
    }
};
} // namespace wmtk::components::prismatic_mesh::quality::native
