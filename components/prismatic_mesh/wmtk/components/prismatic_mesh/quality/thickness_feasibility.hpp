#pragma once
#include "native_mixed.hpp"
namespace wmtk::components::prismatic_mesh::quality::native {
// The displacement/reference vectors at a newly refined node are convex
// combinations of its parent's nodal vectors. Enforcing the ratio over the
// full convex hull therefore survives later conforming subdivision.
inline bool thickness_vectors_feasible(
    const std::vector<V>& reference,
    const std::vector<V>& actual,
    double margin = 1e-6,
    double lower = .8,
    double upper_bound = 1.2)
{
    using L = long double;
    using LV = Eigen::Matrix<L, 3, 1>;
    using LM = Eigen::Matrix<L, Eigen::Dynamic, Eigen::Dynamic>;
    const size_t n = reference.size();
    if (n == 0) return true;
    std::vector<LV> r(n), u(n);
    for (size_t j = 0; j < n; ++j) {
        r[j] = reference[j].cast<L>();
        u[j] = actual[j].cast<L>();
    }
    for (bool upper : {false, true}) {
        const L limit = upper ? L(upper_bound - margin) : L(lower + margin), factor = limit * limit;
        auto valid = [&](const LV& a, const LV& b) {
            const L rr = a.squaredNorm(), uu = b.squaredNorm(),
                    q = upper ? factor * rr - uu : uu - factor * rr;
            return q >= -1e-12L * rr;
        };
        LM q(n, n);
        for (size_t i = 0; i < n; ++i)
            for (size_t j = 0; j < n; ++j)
                q(i, j) = upper ? factor * r[i].dot(r[j]) - u[i].dot(u[j])
                                : u[i].dot(u[j]) - factor * r[i].dot(r[j]);
        for (size_t i = 0; i < n; ++i)
            if (!valid(r[i], u[i])) return false;
        // All two-node faces: the inequality is quadratic along an edge.
        for (size_t i = 0; i < n; ++i)
            for (size_t j = i + 1; j < n; ++j) {
                const L a = q(i, i) + q(j, j) - 2 * q(i, j), b = 2 * (q(i, j) - q(i, i));
                if (a > 0) {
                    const L t = -b / (2 * a);
                    if (t > 0 && t < 1 &&
                        !valid((1 - t) * r[i] + t * r[j], (1 - t) * u[i] + t * u[j]))
                        return false;
                }
            }
        // A quadratic reaches a minimum at a vertex or a stationary point on
        // some active simplex face. Enumerate those (at most 63 for a prism).
        // Singular stationary families attain the same value on a smaller face.
        for (size_t mask = 1; mask < (size_t(1) << n); ++mask) {
            std::vector<size_t> ids;
            for (size_t i = 0; i < n; ++i)
                if (mask & (size_t(1) << i)) ids.push_back(i);
            const size_t k = ids.size();
            if (k < 3) continue;
            L scale = 0;
            for (size_t i : ids)
                for (size_t j : ids) scale = std::max(scale, std::abs(q(i, j)));
            if (scale == 0) continue;
            LM a = LM::Zero(k + 1, k + 1);
            Eigen::Matrix<L, Eigen::Dynamic, 1> b =
                Eigen::Matrix<L, Eigen::Dynamic, 1>::Zero(k + 1);
            b[k] = 1;
            for (size_t i = 0; i < k; ++i) {
                for (size_t j = 0; j < k; ++j) a(i, j) = 2 * q(ids[i], ids[j]) / scale;
                a(i, k) = -1;
                a(k, i) = 1;
            }
            Eigen::FullPivLU<LM> lu(a);
            if (!lu.isInvertible()) continue;
            const auto x = lu.solve(b).eval();
            bool inside = true;
            L sum = 0;
            for (size_t i = 0; i < k; ++i) {
                inside &= x[i] >= -1e-12L;
                sum += std::max(L(0), x[i]);
            }
            if (!inside || sum == 0) continue;
            LV rr = LV::Zero(), uu = LV::Zero();
            for (size_t i = 0; i < k; ++i) {
                const L w = std::max(L(0), x[i]) / sum;
                rr += w * r[ids[i]];
                uu += w * u[ids[i]];
            }
            if (!valid(rr, uu)) return false;
        }
    }
    return true;
}
inline bool thickness_cell_feasible(
    const Mesh& mesh,
    const Cell& cell,
    double margin = 1e-6,
    double lower = .8,
    double upper = 1.2)
{
    std::vector<V> r, u;
    for (size_t v : cell.v) {
        if (!std::isfinite(mesh.thickness[v])) return true;
        const V a = mesh.initial[v] - mesh.base[v], b = mesh.p[v] - mesh.base[v];
        if (a.squaredNorm() == 0 && b.squaredNorm() == 0) continue;
        r.push_back(a);
        u.push_back(b);
    }
    return thickness_vectors_feasible(r, u, margin, lower, upper);
}
} // namespace wmtk::components::prismatic_mesh::quality::native
