#include "BoundaryParametrization.h"

#include <lagrange/utils/assert.h>

#include <map>
#include <unordered_map>
#include <vector>

namespace wmtk {

namespace {

struct Edge
{
    Index v1;
    Index v2;

    Edge(Index x, Index y)
        : v1(std::min(x, y))
        , v2(std::max(x, y))
    {}

    auto key() const { return std::make_pair(v1, v2); }

    bool operator<(const Edge& e) const { return key() < e.key(); }
};

struct CurveNetwork
{
    using Curve = std::vector<int>;

    std::vector<Curve> curves;
    std::vector<bool> is_closed;
};

CurveNetwork split_loops(
    const std::vector<std::vector<int>>& loops,
    const std::map<Edge, int> edge_to_seam)
{
    CurveNetwork result;

    // Color each loop edge based on seam info
    std::unordered_map<int, int> seam_to_loop;
    std::vector<std::vector<int>> edge_colors(loops.size());
    for (int loop_id = 0; loop_id < static_cast<int>(loops.size()); ++loop_id) {
        const auto& loop = loops[loop_id];
        auto& colors = edge_colors[loop_id];
        colors.resize(loop.size());
        for (size_t i = 0; i < loop.size(); ++i) {
            const int v0 = loop[i];
            const int v1 = loop[(i + 1) % loop.size()];
            Edge e(v0, v1);
            auto it = edge_to_seam.find(e);
            if (it != edge_to_seam.end()) {
                const int seam_id = *it;
                auto [lt, _] seam_to_loop.try_emplace(seam_id, loop_id);
                colors[i] = lt->second;
            } else {
                colors[i] = -1;
            }
        }
    }

    // Split each loop based on continuous edge colors
    for (int loop_id = 0; loop_id < static_cast<int>(loops.size()); ++loop_id) {
        const auto& loop = loops[loop_id];
        const auto& colors = edge_colors[loop_id];
        if (loop.empty()) {
            continue;
        }
        auto it = std::adjacent_find(colors.begin(), colors.end(), std::not_equal_to<int>);
        if (it == colors.end()) {
            // 1st case: every edge is the same color. Keep loop intact.
            result.curves.push_back(loop);
            result.is_closed.push_back(true);
        } else {
            // 2nd case: split each chunk of edges sharing the same color

            {
                // First we rotate the vertex ids to make sure the same color doesn't run
                // periodically across the loop indices.
                std::ptrdiff_t m = std::distance(colors.begin(), it) + 1;
                std::rotate(loop.begin(), loop.begin() + m, loop.end());
                std::rotate(colors.begin(), colors.begin() + m, colors.end());
            }

            // Now we can split safely each continuous chunk...
            it = std::adjacent_find(colors.begin(), colors.end(), std::not_equal_to<int>);
            while (it != colors.end()) {
                std::ptrdiff_t m = std::distance(colors.begin(), it) + 1;
                Curve path(loop.begin(), loop.begin() + m);
                loop.erase(loop.begin(), loop.begin() + m);
                colors.erase(colors.begin(), colors.begin() + m);
                result.curves.emplace_back(std::move(path));
                result.is_closed.emplace_back(false);
                it = std::adjacent_find(colors.begin(), colors.end(), std::not_equal_to<int>);
            }
        }
    }

    return result;
}

Boundary::ParameterizedCurves parameterize_curves(
    const CurveNetwork& input,
    const std::map<Edge, int> edge_to_seam)
{
    Boundary::ParameterizedCurves result;

    std::unordered_map<int, int> seam_to_curve;

    auto check_orientation = [&](const auto& curve, int parent_id) {
        Edge e0(curve[0], curve[1]);
        const int s0 = edge_to_seam.at(e0);
        Edge e1(input.curves[parent_id][0], input.curves[parent_id][1]);
        const int s1 = edge_to_seam.at(e1);
        return s0 == s1;
    };

    for (int curve_id = 0; curve_id < static_cast<int>(input.curves.size()); ++curve_id) {
        auto curve = input.curves[curve_id];

        // First pass on the curve to find the parent curve id
        int parent_id = curve_id;
        for (size_t i = 0; i < curve.size(); ++i) {
            const int v0 = curve[i];
            const int v1 = curve[(i + 1) % curve.size()];

            // Find mirror curve_id
            Edge e(v0, v1);
            auto it = edge_to_seam.find(e);
            if (it != edge_to_seam.end()) {
                const int seam_id = *it;
                auto [ct, inserted] = seam_to_curve.try_emplace(seam_id, curve_id);
                if (!inserted) {
                    la_runtime_assert(parent_id == curve_id || parent_id == ct->second);
                    parent_id = ct->second;
                }
            }
        }

        // Check if we need to flip the current curve
        if (parent_id != curve_id) {
            la_runtime_assert(curve.size(), input.curves[parent_id].size());
            if (check_orientation(curve, parent_id)) {
                std::reverse(curve.begin(), curve.end());
                la_runtime_assert(check_orientation(curve, parent_id));
            }
        }

        // Second pass to parameterize the current curve
        std::vector<Eigen::Vector2d> positions;
        std::vector<double> arclengths;
        double len = 0.; // cumulative length till current vertex
        arclengths.emplace_back(len);
        for (size_t i = 0; i < curve.size(); ++i) {
            const int v0 = curve[i];
            const int v1 = curve[(i + 1) % curve.size()];
            auto p0 = V.row(v0).leftCols(2);
            auto p1 = V.row(v1).leftCols(2);
            positions.emplace_back(p0);
            len += (p1 - p0).stableNorm();
            arclengths.emplace_back(len);
        }

        assert(arclengths.size() == positions.size() + 1);
        result.positions.emplace_back(std::move(positions));
        result.arclengths.emplace_back(std::move(arclengths));
        result.periodic.emplace(input.is_closed[curve_id]);
        result.parent_curve.emplace(parent_id);
    }

    // Sanity check: When two curves are paired, and one of them is "open", we make sure we use the
    // open curve as the parent...
    for (int curve_id = 0; curve_id < static_cast<int>(input.curves.size()); ++curve_id) {
        int parent_id = result.parent_curve[curve_id];
        if (parent_id != curve_id && result.periodic[parent_id] && !result.periodic[curve_id]) {
            logger().warn("Found an open boundary polyline paired with a periodic parent polyline. "
                          "Swapping them.");
            result.parent_curve[parent_id] = curve_id;
            result.parent_curve[curve_id] = curve_id;
        }
    }
}

} // namespace

void Boundary::construct_boundaries(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const Eigen::MatrixXi& E0,
    const Eigen::MatrixXi& E1)
{
    std::vector<std::vector<int>> paths;
    igl::boundary_loop(F, paths);

    std::map<Edge, int> edge_to_seam;
    for (int i = 0; i < E0.rows(); ++i) {
        Edge e0(E0(i, 0), E0(i, 1));
        Edge e1(E1(i, 0), E1(i, 1));
        for (auto e : {e0, e1}) {
            auto [it, inserted] = edge_to_seam.try_emplace(e, i);
            if (!inserted) {
                throw std::runtime_error("Non-manifold seam edge detected");
            }
        }
    }

    m_curves = parameterize_curves(split_loops(paths, edge_to_seam), edge_to_seam);
}

ParameterizedSegment Boundary::t_to_segment(int curve_id, double t) const
{
    ParameterizedSegment result;

    const auto& arclength = m_curves.arclengths[m_curves.parent_curve[curve_id]];
    while (t < 0) t += arclength.back();
    t = std::fmod(t, arclength.back());
    assert(t < arclength.back());

    auto it = std::prev(std::upper_bound(arclength.begin(), arclength.end(), t));
    auto a = std::distance(arclength.begin(), it);
    assert((a + 1) < arclength.size());

    result.t0 = *it;
    const auto& boundary = m_curves.positions[curve_id];
    assert(a < boundary.size());
    result.A = boundary[a];
    result.B = boundary[(a + 1) % boundary.size()];
    result.tlen = arclength[a + 1] - arclength[a];
    return result;
}

double Boundary::uv_to_t(const Eigen::Vector2d& v) const
{
    double ret_t = 0.;
    Eigen::RowVector2d P;
    P = v.transpose();
    Eigen::RowVector<double, 1> tmp_t, tmp_d;
    double d = std::numeric_limits<double>::infinity();
    for (auto i = 0; i < m_curves.positions.size(); i++) {
        for (auto j = 0; j < m_curves.positions[i].size(); j++) {
            Eigen::RowVector2d A = m_curves.positions[i][j];
            Eigen::RowVector2d B = m_curves.positions[i][(j + 1) % m_curves.positions[i].size()];
            igl::project_to_line_segment(P, A, B, tmp_t, tmp_d);
            if (tmp_d(0) < d) {
                d = tmp_d(0);
                ret_t = m_curves.arclengths[i][j] +
                        std::abs(
                            tmp_t(0) *
                            (m_curves.arclengths[i][(j + 1) % m_curves.arclengths[i].size()] -
                             m_curves.arclengths[i][j]));
            }
        }
    }
    return ret_t;
}

std::pair<int, int> Boundary::uv_to_ij(const Eigen::Vector2d& v, double& t) const
{
    std::pair<int, int> ij;
    t = 0.;
    Eigen::RowVector2d P;
    P = v.transpose();
    Eigen::RowVector<double, 1> tmp_t, tmp_d;
    double d = std::numeric_limits<double>::infinity();
    for (auto i = 0; i < m_curves.positions.size(); i++) {
        for (auto j = 0; j < m_curves.positions[i].size(); j++) {
            Eigen::RowVector2d A = m_curves.positions[i][j];
            Eigen::RowVector2d B = m_curves.positions[i][(j + 1) % m_curves.positions[i].size()];
            igl::project_to_line_segment(P, A, B, tmp_t, tmp_d);
            if (tmp_d(0) < d) {
                d = tmp_d(0);
                t = m_curves.arclengths[i][j] +
                    std::abs(
                        tmp_t(0) *
                        (m_curves.arclengths[i][(j + 1) % m_curves.arclengths[i].size()] -
                         m_curves.arclengths[i][j]));
                ij.first = i;
                ij.second = j;
            }
        }
    }
    std::cout << m_curves.arclengths[ij.first].back() << std::endl;
    assert(t <= m_curves.arclengths[ij.first].back());
    return ij;
}

} // namespace wmtk
