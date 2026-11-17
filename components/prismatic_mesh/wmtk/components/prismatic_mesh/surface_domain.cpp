#include "surface_domain.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <set>

namespace wmtk::components::prismatic_mesh::internal {
namespace {
using UV = SurfaceDomain::UV;
using Triangle = SurfaceDomain::Triangle;
using Edge = std::array<size_t, 2>;
Edge edge(size_t a, size_t b)
{
    if (a > b) std::swap(a, b);
    return {a, b};
}
long double cross(const UV& a, const UV& b)
{
    return static_cast<long double>(a[0]) * b[1] - static_cast<long double>(a[1]) * b[0];
}
double first_root(long double c0, long double c1, long double c2)
{
    double root = std::numeric_limits<double>::infinity();
    const auto add = [&](long double x) {
        if (x > 0 && std::isfinite(x)) root = std::min(root, static_cast<double>(x));
    };
    if (c2 == 0) {
        if (c1 < 0) add(-c0 / c1);
    } else {
        const long double discriminant = c1 * c1 - 4 * c2 * c0;
        if (discriminant >= 0) {
            const long double q = -.5L * (c1 + std::copysign(std::sqrt(discriminant), c1));
            add(q / c2);
            if (q != 0) add(c0 / q);
        }
    }
    return root;
}
} // namespace

std::optional<SurfaceDomain> SurfaceDomain::build(
    const MatrixXd& positions,
    const std::vector<Triangle>& faces,
    const std::array<size_t, 2>& moving,
    std::string& failure)
{
    const auto fail = [&](const char* reason) -> std::optional<SurfaceDomain> {
        failure = reason;
        return std::nullopt;
    };
    if (faces.empty() || moving[0] == moving[1]) return fail("empty_or_invalid_patch");
    SurfaceDomain out;
    std::set<size_t> used;
    std::set<Triangle> unique;
    for (auto f : faces) {
        std::sort(f.begin(), f.end());
        if (f[0] == f[1] || f[1] == f[2] || !unique.insert(f).second)
            return fail("invalid_or_duplicate_triangle");
        used.insert(f.begin(), f.end());
    }
    if (!used.count(moving[0]) || !used.count(moving[1])) return fail("missing_endpoint");
    out.vertices.assign(used.begin(), used.end());
    for (size_t j = 0; j < out.vertices.size(); ++j) out.local[out.vertices[j]] = j;
    out.ends = {out.local.at(moving[0]), out.local.at(moving[1])};
    std::map<Edge, std::vector<size_t>> edges;
    std::vector<Triangle> cells;
    std::vector<std::set<size_t>> neighbors(used.size());
    for (const auto& face : faces) {
        Triangle t;
        for (size_t j = 0; j < 3; ++j) t[j] = out.local.at(face[j]);
        const size_t id = cells.size();
        cells.push_back(t);
        for (size_t j = 0; j < 3; ++j) {
            const size_t a = t[j], b = t[(j + 1) % 3];
            auto& star = edges[edge(a, b)];
            star.push_back(id);
            if (star.size() > 2) return fail("nonmanifold_edge");
            neighbors[a].insert(b);
            neighbors[b].insert(a);
        }
    }
    if (!edges.count(edge(out.ends[0], out.ends[1]))) return fail("edge_not_on_offset_surface");
    if (static_cast<int64_t>(used.size()) - static_cast<int64_t>(edges.size()) +
            static_cast<int64_t>(cells.size()) !=
        1)
        return fail("patch_is_not_a_disk");
    std::map<size_t, std::vector<size_t>> boundary_neighbors;
    for (const auto& [e, star] : edges)
        if (star.size() == 1) {
            boundary_neighbors[e[0]].push_back(e[1]);
            boundary_neighbors[e[1]].push_back(e[0]);
        }
    if (boundary_neighbors.size() < 3) return fail("missing_disk_boundary");
    for (const auto& [v, star] : boundary_neighbors)
        if (star.size() != 2) return fail("nonmanifold_boundary");
    const size_t start = boundary_neighbors.begin()->first;
    size_t previous = std::numeric_limits<size_t>::max(), current = start;
    do {
        out.boundary.push_back(current);
        const auto& pair = boundary_neighbors.at(current);
        const size_t next = pair[0] == previous ? pair[1] : pair[0];
        previous = current;
        current = next;
        if (out.boundary.size() > boundary_neighbors.size()) return fail("invalid_boundary_cycle");
    } while (current != start);
    if (out.boundary.size() != boundary_neighbors.size()) return fail("multiple_boundary_loops");

    // Orient all source triangles consistently through their shared edges.
    std::vector<bool> seen(cells.size(), false);
    std::queue<size_t> queue;
    queue.push(0);
    seen[0] = true;
    size_t visited = 0;
    while (!queue.empty()) {
        const size_t i = queue.front();
        queue.pop();
        ++visited;
        for (size_t j = 0; j < 3; ++j) {
            const size_t a = cells[i][j], b = cells[i][(j + 1) % 3];
            for (size_t k : edges.at(edge(a, b))) {
                if (k == i) continue;
                bool same = false;
                for (size_t l = 0; l < 3; ++l)
                    same |= cells[k][l] == a && cells[k][(l + 1) % 3] == b;
                if (seen[k]) {
                    if (same) return fail("inconsistent_surface_orientation");
                } else {
                    if (same) std::swap(cells[k][1], cells[k][2]);
                    seen[k] = true;
                    queue.push(k);
                }
            }
        }
    }
    if (visited != cells.size()) return fail("disconnected_patch");

    out.points.resize(used.size(), 3);
    out.parameters = MatrixXd::Zero(used.size(), 2);
    for (size_t j = 0; j < out.vertices.size(); ++j) {
        if (out.vertices[j] >= static_cast<size_t>(positions.rows())) return fail("invalid_vertex");
        out.points.row(j) = positions.row(out.vertices[j]);
    }
    if (!out.points.allFinite()) return fail("nonfinite_position");
    std::map<size_t, size_t> interior;
    for (size_t j = 0; j < used.size(); ++j)
        if (!boundary_neighbors.count(j)) interior[j] = interior.size();
    const double pi = std::acos(-1.);
    for (size_t j = 0; j < out.boundary.size(); ++j) {
        const double angle = 2 * pi * j / out.boundary.size();
        out.parameters.row(out.boundary[j]) << std::cos(angle), std::sin(angle);
    }
    if (!interior.empty()) {
        MatrixXd laplacian = MatrixXd::Zero(interior.size(), interior.size());
        MatrixXd rhs = MatrixXd::Zero(interior.size(), 2);
        for (const auto& [v, row] : interior) {
            laplacian(row, row) = neighbors[v].size();
            for (size_t w : neighbors[v])
                if (const auto col = interior.find(w); col != interior.end())
                    laplacian(row, col->second) -= 1;
                else
                    rhs.row(row) += out.parameters.row(w);
        }
        Eigen::LDLT<MatrixXd> solver(laplacian);
        if (solver.info() != Eigen::Success || !solver.isPositive()) return fail("singular_chart");
        const MatrixXd solution = solver.solve(rhs);
        if (!solution.allFinite()) return fail("nonfinite_chart");
        for (const auto& [v, row] : interior) out.parameters.row(v) = solution.row(row);
    }
    const auto at = [&](size_t v) -> UV { return out.parameters.row(v); };
    if (cross(at(cells[0][1]) - at(cells[0][0]), at(cells[0][2]) - at(cells[0][0])) < 0)
        for (auto& t : cells) std::swap(t[1], t[2]);
    for (const auto& t : cells) {
        Eigen::Matrix2d uv;
        uv.col(0) = at(t[1]) - at(t[0]);
        uv.col(1) = at(t[2]) - at(t[0]);
        if (!(uv.determinant() > 1e-12)) return fail("folded_or_degenerate_chart");
        Eigen::Matrix<double, 3, 2> xyz;
        xyz.col(0) = out.points.row(t[1]) - out.points.row(t[0]);
        xyz.col(1) = out.points.row(t[2]) - out.points.row(t[0]);
        if (!(xyz.col(0).cross(xyz.col(1)).stableNorm() > 0))
            return fail("degenerate_source_triangle");
        const Eigen::Matrix2d inverse = uv.inverse();
        out.triangles.push_back({t, inverse, xyz * inverse});
    }
    failure.clear();
    return out;
}

SurfaceDomain::UV SurfaceDomain::coordinate(size_t v) const
{
    return parameters.row(local.at(v));
}

Eigen::Vector3d SurfaceDomain::barycentric(const ChartTriangle& t, const UV& uv) const
{
    const UV z = t.inverse * (uv - UV(parameters.row(t.vertices[0])));
    return {1 - z.sum(), z[0], z[1]};
}

std::optional<Vector3d> SurfaceDomain::position(const UV& uv) const
{
    if (!uv.allFinite()) return std::nullopt;
    for (size_t j = 0; j < vertices.size(); ++j)
        if (uv == UV(parameters.row(j))) return Vector3d(points.row(j));
    for (const auto& t : triangles) {
        auto b = barycentric(t, uv);
        if (b.minCoeff() < -1e-12) continue;
        b = b.cwiseMax(0.);
        b /= b.sum();
        return Vector3d(points.row(t.vertices[0])) +
               b[1] * Vector3d(points.row(t.vertices[1]) - points.row(t.vertices[0])) +
               b[2] * Vector3d(points.row(t.vertices[2]) - points.row(t.vertices[0]));
    }
    return std::nullopt;
}

SurfaceDomain::UV SurfaceDomain::descent(const UV& uv, const Vector3d& gradient) const
{
    UV best = UV::Zero();
    double score = 0;
    for (const auto& t : triangles) {
        const auto b = barycentric(t, uv);
        if (b.minCoeff() < -1e-12) continue;
        const UV g = t.jacobian.transpose() * gradient;
        const std::array<UV, 3> db{
            -t.inverse.row(0).transpose() - t.inverse.row(1).transpose(),
            t.inverse.row(0).transpose(),
            t.inverse.row(1).transpose()};
        std::vector<UV> candidates{-g};
        for (size_t j = 0; j < 3; ++j)
            if (b[j] <= 1e-10) {
                const UV tangent{-db[j][1], db[j][0]};
                candidates.push_back(tangent);
                candidates.push_back(-tangent);
            }
        for (auto d : candidates) {
            const double norm = d.stableNorm();
            if (!(norm > 0)) continue;
            d /= norm;
            bool feasible = true;
            for (size_t j = 0; j < 3; ++j)
                if (b[j] <= 1e-10 && db[j].dot(d) < -1e-12 * db[j].norm()) feasible = false;
            const double improvement = -g.dot(d);
            if (feasible && improvement > score) {
                score = improvement;
                best = improvement * d;
            }
        }
    }
    return best;
}

double SurfaceDomain::step_limit(const std::array<UV, 2>& direction) const
{
    const auto at = [&](size_t v) -> UV { return parameters.row(v); };
    const auto delta = [&](size_t v) -> UV {
        if (v == ends[0]) return direction[0];
        if (v == ends[1]) return direction[1];
        return UV::Zero();
    };
    double limit = std::numeric_limits<double>::infinity();
    for (size_t end = 0; end < 2; ++end)
        for (size_t j = 0; j < boundary.size(); ++j) {
            const UV a = at(boundary[j]), b = at(boundary[(j + 1) % boundary.size()]);
            const long double slope = cross(b - a, direction[end]);
            if (slope < 0)
                limit = std::min(
                    limit,
                    std::max(0., static_cast<double>(cross(b - a, at(ends[end]) - a) / -slope)));
        }
    for (const auto& t : triangles) {
        const auto& v = t.vertices;
        const UV a = at(v[1]) - at(v[0]), b = at(v[2]) - at(v[0]);
        const UV da = delta(v[1]) - delta(v[0]), db = delta(v[2]) - delta(v[0]);
        limit =
            std::min(limit, first_root(cross(a, b), cross(da, b) + cross(a, db), cross(da, db)));
    }
    return limit;
}

bool SurfaceDomain::valid(const std::array<UV, 2>& updated) const
{
    const auto at = [&](size_t v) -> UV {
        if (v == ends[0]) return updated[0];
        if (v == ends[1]) return updated[1];
        return parameters.row(v);
    };
    for (const auto& t : triangles)
        if (!(cross(at(t.vertices[1]) - at(t.vertices[0]), at(t.vertices[2]) - at(t.vertices[0])) >
              1e-12))
            return false;
    return position(updated[0]).has_value() && position(updated[1]).has_value();
}
} // namespace wmtk::components::prismatic_mesh::internal
