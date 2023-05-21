#include "BoundaryParametrization.h"

#include <igl/boundary_loop.h>
#include <igl/project_to_line_segment.h>
#include <lagrange/utils/DisjointSets.h>
#include <lagrange/utils/assert.h>

#include <fstream>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace wmtk {

namespace {

struct Edge
{
    int v1;
    int v2;

    Edge(int x, int y)
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


// A cone vertex is a vertex that lies at the intersection of more than two seams, or at the
// endpoint of a seam.
std::vector<bool> compute_cone_vertices(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& E0,
    const Eigen::MatrixXi& E1)
{
    int num_vertices = V.rows();
    int num_seams = E0.rows();
    std::vector<bool> is_cone_vertex(num_vertices, false);

    lagrange::DisjointSets<int> vertex_colors(num_vertices);
    for (int e = 0; e < num_seams; ++e) {
        vertex_colors.merge(E0(e, 0), E1(e, 0));
        vertex_colors.merge(E0(e, 1), E1(e, 1));
    }

    std::vector<std::unordered_set<int>> color_to_seams(num_vertices);
    for (int e = 0; e < num_seams; ++e) {
        for (int v : {E0(e, 0), E0(e, 1), E1(e, 0), E1(e, 1)}) {
            color_to_seams[vertex_colors.find(v)].insert(e);
        }
    }

    for (int e = 0; e < num_seams; ++e) {
        for (int v : {E0(e, 0), E0(e, 1), E1(e, 0), E1(e, 1)}) {
            if (color_to_seams[vertex_colors.find(v)].size() != 2) {
                is_cone_vertex[v] = true;
            }
        }
    }

    return is_cone_vertex;
}

CurveNetwork split_loops(
    const Eigen::MatrixXd& vertices,
    const std::vector<std::vector<int>>& loops,
    const std::map<Edge, int> edge_to_seam,
    const std::vector<bool>& is_cone_vertex)
{
    CurveNetwork result;

    logger().debug("Num loops: {}", loops.size());

    // Map seam to incident loop ids
    std::unordered_multimap<int, int> seam_to_loop;
    for (int loop_id = 0; loop_id < static_cast<int>(loops.size()); ++loop_id) {
        const auto& loop = loops[loop_id];
        for (size_t i = 0; i < loop.size(); ++i) {
            const int v0 = loop[i];
            const int v1 = loop[(i + 1) % loop.size()];
            Edge e(v0, v1);
            auto it = edge_to_seam.find(e);
            if (it != edge_to_seam.end()) {
                const int seam_id = it->second;
                seam_to_loop.emplace(seam_id, loop_id);
                la_runtime_assert(seam_to_loop.count(seam_id) <= 2, "Non-manifold seam detected!");
            }
        }
    }

    // Color each loop edge according using the following strategy:
    // 1. If the edge is a seam edge, use a unique color based on the (loop_id0, loop_id1) on either
    //    side of the seam.
    // 2. If the edge is a boundary edge, use a unique color based on the loop_id
    int num_colors = 0;
    using LoopPair = Edge;
    std::map<LoopPair, int> loop_pair_to_color;
    std::vector<std::vector<int>> edge_colors(loops.size());

    auto color_from_edge = [&](int v0, int v1, int default_color) {
        Edge e(v0, v1);
        auto it = edge_to_seam.find(e);
        if (it != edge_to_seam.end()) {
            auto range = seam_to_loop.equal_range(it->second);
            la_runtime_assert(std::distance(range.first, range.second) == 2);
            LoopPair key(range.first->second, std::next(range.first)->second);
            auto [ct, inserted] = loop_pair_to_color.try_emplace(key, num_colors);
            if (inserted) {
                ++num_colors;
            }
            return ct->second;
        } else {
            return default_color;
        }
    };

    for (int loop_id = 0; loop_id < static_cast<int>(loops.size()); ++loop_id) {
        const auto& loop = loops[loop_id];
        auto& colors = edge_colors[loop_id];
        colors.resize(loop.size());
        const int current_color = num_colors++;
        for (size_t i = 0; i < loop.size(); ++i) {
            const int v0 = loop[i];
            const int v1 = loop[(i + 1) % loop.size()];
            colors[i] = color_from_edge(v0, v1, current_color);
        }
    }

    // If a loop has cone vertices, update edge colors on each side of the cone vertices
    for (int loop_id = 0; loop_id < static_cast<int>(loops.size()); ++loop_id) {
        const auto& loop = loops[loop_id];
        auto& colors = edge_colors[loop_id];
        bool has_cone_vertices = false;
        int max_color = 0;
        for (size_t i = 0; i < loop.size(); ++i) {
            const int v0 = loop[i];
            max_color = std::max(max_color, colors[i]);
            if (is_cone_vertex[v0]) {
                has_cone_vertices = true;
            }
        }
        if (!has_cone_vertices) {
            continue;
        }
        int loop_num_colors = max_color + 1;
        const size_t n = loop.size();
        for (size_t i = 0; i < n; ++i) {
            const int v0 = loop[i];
            if (is_cone_vertex[v0]) {
                size_t j = i + 1;
                for (; j < n; ++j) {
                    const int v1 = loop[j];
                    if (is_cone_vertex[v1]) {
                        break;
                    }
                }
                if (j == n) {
                    break;
                }
                la_debug_assert(is_cone_vertex[loop[j]]);
                for (size_t k = i; k < j; ++k) {
                    colors[k] += loop_num_colors;
                }
                loop_num_colors += max_color + 1;
            }
        }
    }

    // Split each loop based on continuous edge colors
    for (int loop_id = 0; loop_id < static_cast<int>(loops.size()); ++loop_id) {
        auto loop = loops[loop_id];
        auto& colors = edge_colors[loop_id];
        if (loop.empty()) {
            continue;
        }
        auto it = std::adjacent_find(colors.begin(), colors.end(), std::not_equal_to<>());
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

            // Duplicate the first point to the back of the loop, so that we can split the loop.
            // Don't duplicate the color, we have one color per edge, so now that the loop is not
            // periodic, we have loop.size() == colors.size() + 1.
            loop.push_back(loop.front());

            // Now we can split safely each continuous chunk...
            it = std::adjacent_find(colors.begin(), colors.end(), std::not_equal_to<>());
            while (it != colors.end()) {
                std::ptrdiff_t m = std::distance(colors.begin(), it) + 1;
                la_debug_assert(m < colors.size());
                CurveNetwork::Curve path(loop.begin(), loop.begin() + m + 1);
                loop.erase(loop.begin(), loop.begin() + m);
                colors.erase(colors.begin(), colors.begin() + m);
                result.curves.emplace_back(std::move(path));
                result.is_closed.emplace_back(false);
                it = std::adjacent_find(colors.begin(), colors.end(), std::not_equal_to<>());
            }
            result.curves.emplace_back(std::move(loop));
            result.is_closed.emplace_back(false);
        }
    }

    return result;
}

// Compute parent curves for each curve in the input network.
std::vector<int> compute_parent_curves(
    const CurveNetwork& input,
    const std::map<Edge, int>& edge_to_seam)
{
    std::vector<int> parent_curve(input.curves.size(), -1);
    std::unordered_map<int, int> seam_to_curve;

    for (int curve_id = 0; curve_id < static_cast<int>(input.curves.size()); ++curve_id) {
        const auto& curve = input.curves[curve_id];
        la_debug_assert(curve.size() > 1);

        // First pass on the curve to find the parent curve id
        int parent_id = curve_id;
        for (size_t i = 0; i < curve.size(); ++i) {
            if (!input.is_closed[curve_id] && i == curve.size() - 1) continue;
            const int v0 = curve[i];
            const int v1 = curve[(i + 1) % curve.size()];

            // Find mirror curve_id
            Edge e(v0, v1);
            auto it = edge_to_seam.find(e);
            if (it != edge_to_seam.end()) {
                const int seam_id = it->second;
                auto [ct, inserted] = seam_to_curve.try_emplace(seam_id, curve_id);
                if (!inserted) {
                    la_runtime_assert(parent_id == curve_id || parent_id == ct->second);
                    parent_id = ct->second;
                }
            }
        }

        // Check if we need to flip the current curve
        if (parent_id != curve_id) {
            la_runtime_assert(
                curve.size() + (input.is_closed[curve_id] ? 1 : 0) ==
                    input.curves[parent_id].size() + (input.is_closed[parent_id] ? 1 : 0),
                fmt::format(
                    "Current curve #{} has {} vertices, while parent curve #{} has {} vertices",
                    curve_id,
                    curve.size(),
                    parent_id,
                    input.curves[parent_id].size()));
        }

        parent_curve[curve_id] = parent_id;
    }

    return parent_curve;
}

// When a curve and its parent do have the same periodicity, we:
// 1. Use the open curve as the parent
// 2. Rotate the child curve vertices to match its parent
// 3. Make the child curve not periodic by duplicating the first vertex at the end
CurveNetwork fix_curve_orientation_and_periodicity(
    CurveNetwork input,
    const std::vector<int>& parent_curve,
    const std::map<Edge, int>& edge_to_seam)
{
    auto curve_edge_to_seam = [&](int curve_id, int i) {
        int n = static_cast<int>(input.curves[curve_id].size());
        i = (i + n) % n;
        int j = (i + 1) % n;
        Edge e(input.curves[curve_id][i], input.curves[curve_id][j]);
        return edge_to_seam.at(e);
    };

    auto check_orientation = [&](int curve_id, int parent_id) {
        for (int i = 0; i + (input.is_closed[curve_id] ? 0 : 1) <
                        static_cast<int>(input.curves[curve_id].size());
             ++i) {
            if (curve_edge_to_seam(curve_id, i) != curve_edge_to_seam(parent_id, i)) {
                return false;
            }
        }
        return true;
    };

    for (int curve_id = 0; curve_id < static_cast<int>(input.curves.size()); ++curve_id) {
        la_debug_assert(input.curves[curve_id].size() > 1);
        int parent_id = parent_curve[curve_id];

        if (curve_id != parent_id) {
            // If curves have different periodicity, use the open curve as the parent
            if (!input.is_closed[curve_id] && input.is_closed[parent_id]) {
                std::swap(input.is_closed[curve_id], input.is_closed[parent_id]);
                std::swap(input.curves[curve_id], input.curves[parent_id]);
            }

            if (input.is_closed[curve_id]) {
                // If curve is closed, we need to rotate it (and maybe flip it) to match the parent
                // curve
                la_runtime_assert(
                    input.curves[parent_id].size() > 2,
                    "A closed curve cannot be matched to an open single-segment curve");

                auto find_matching_edge = [&](int s) {
                    for (int i = 0; i < input.curves[curve_id].size(); ++i) {
                        if (curve_edge_to_seam(curve_id, i) == s) {
                            return i;
                        }
                    }
                    throw std::runtime_error("Could not find matching edge");
                };

                int s0 = curve_edge_to_seam(parent_id, 0);
                int s1 = curve_edge_to_seam(parent_id, 1);

                int i0 = find_matching_edge(s0);
                if (s1 == curve_edge_to_seam(curve_id, i0 - 1)) {
                    // Need to reverse the current curve
                    std::reverse(input.curves[curve_id].begin(), input.curves[curve_id].end());
                    i0 = find_matching_edge(s0);
                }
                la_runtime_assert(s1 == curve_edge_to_seam(curve_id, i0 + 1));
                // Now rotate the current curve to match the parent curve
                std::rotate(
                    input.curves[curve_id].begin(),
                    input.curves[curve_id].begin() + i0,
                    input.curves[curve_id].end());
            } else {
                // If the curve is open, we only need to check if we need to flip it.
                if (!check_orientation(curve_id, parent_id)) {
                    std::reverse(input.curves[curve_id].begin(), input.curves[curve_id].end());
                }
            }

            // If the curve is closed but not its parent, make current curve open by duplicating the
            // first vertex at the end
            if (input.is_closed[curve_id] && !input.is_closed[parent_id]) {
                input.curves[curve_id].push_back(input.curves[curve_id][0]);
                input.is_closed[curve_id] = false;
            }

            la_runtime_assert(check_orientation(curve_id, parent_id));
        }
    }

    return input;
}

Boundary::ParameterizedCurves parameterize_curves(
    CurveNetwork input,
    const Eigen::MatrixXd& V,
    const std::map<Edge, int>& edge_to_seam)
{
    Boundary::ParameterizedCurves result;
    result.parent_curve = compute_parent_curves(input, edge_to_seam);
    input =
        fix_curve_orientation_and_periodicity(std::move(input), result.parent_curve, edge_to_seam);

    for (int curve_id = 0; curve_id < static_cast<int>(input.curves.size()); ++curve_id) {
        auto curve = input.curves[curve_id];
        la_debug_assert(curve.size() > 1);

        // Second pass to parameterize the current curve
        std::vector<Eigen::Vector2d> positions;
        std::vector<double> arclengths;
        double len = 0.; // cumulative length till current vertex
        arclengths.emplace_back(len);
        for (size_t i = 0; i < curve.size(); ++i) {
            const int v0 = curve[i];
            auto p0 = V.row(v0).leftCols(2);
            positions.emplace_back(p0);
            if (!input.is_closed[curve_id] && i == curve.size() - 1) continue;
            const int v1 = curve[(i + 1) % curve.size()];
            auto p1 = V.row(v1).leftCols(2);
            len += (p1 - p0).stableNorm();
            arclengths.emplace_back(len);
        }

        assert(arclengths.size() == positions.size() + (input.is_closed[curve_id] ? 1 : 0));
        result.positions.emplace_back(std::move(positions));
        result.arclengths.emplace_back(std::move(arclengths));
        result.periodic.emplace_back(input.is_closed[curve_id]);
    }

    return result;
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

    m_curves = parameterize_curves(
        split_loops(V, paths, edge_to_seam, compute_cone_vertices(V, E0, E1)),
        V,
        edge_to_seam);
}

Boundary::ParameterizedSegment Boundary::t_to_segment(int curve_id, double t) const
{
    ParameterizedSegment result;

    const auto& arclength = m_curves.arclengths[m_curves.parent_curve[curve_id]];
    if (is_periodic(curve_id)) {
        while (t < 0) {
            t += arclength.back();
        }
        t = std::fmod(t, arclength.back());
    } else {
        t = std::clamp(t, 0., arclength.back());
    }
    assert(t <= arclength.back());

    auto it = std::prev(std::upper_bound(arclength.begin(), arclength.end(), t));
    if (!is_periodic(curve_id) && std::next(it) == arclength.end()) {
        --it;
    }
    auto a = std::distance(arclength.begin(), it);
    assert(a < arclength.size());

    result.t0 = *it;
    const auto& boundary = m_curves.positions[curve_id];
    assert(a < boundary.size());
    result.A = boundary[a];
    result.B =
        m_curves.periodic[curve_id] ? boundary[(a + 1) % boundary.size()] : boundary[(a + 1)];
    result.tlen = arclength[a + 1] - arclength[a];
    return result;
}

std::pair<int, double> Boundary::uv_to_t(const Eigen::Vector2d& v) const
{
    int ret_i = -1;
    double ret_t = 0.;
    Eigen::RowVector2d P;
    P = v.transpose();
    Eigen::RowVector<double, 1> tmp_t, tmp_d;
    double d = std::numeric_limits<double>::infinity();
    for (auto i = 0; i < m_curves.positions.size(); i++) {
        for (auto j = 0; j < m_curves.positions[i].size(); j++) {
            if (!m_curves.periodic[i] && j == m_curves.positions[i].size() - 1) continue;
            size_t j2 = m_curves.periodic[i] ? (j + 1) % m_curves.positions[i].size() : j + 1;
            Eigen::RowVector2d A = m_curves.positions[i][j];
            Eigen::RowVector2d B = m_curves.positions[i][j2];
            igl::project_to_line_segment(P, A, B, tmp_t, tmp_d);
            if (tmp_d(0) < d) {
                d = tmp_d(0);
                int k = m_curves.parent_curve[i];
                double len = m_curves.arclengths[k][j + 1] - m_curves.arclengths[k][j];
                ret_i = i;
                ret_t = m_curves.arclengths[k][j] + tmp_t(0) * len;
            }
        }
    }
    return std::make_pair(ret_i, ret_t);
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
            if (!m_curves.periodic[i] && j == m_curves.positions[i].size() - 1) continue;
            size_t j2 = m_curves.periodic[i] ? (j + 1) % m_curves.positions[i].size() : j + 1;
            Eigen::RowVector2d A = m_curves.positions[i][j];
            Eigen::RowVector2d B = m_curves.positions[i][j2];
            igl::project_to_line_segment(P, A, B, tmp_t, tmp_d);
            if (tmp_d(0) < d) {
                d = tmp_d(0);
                int k = m_curves.parent_curve[i];
                double len = m_curves.arclengths[k][j + 1] - m_curves.arclengths[k][j];
                t = m_curves.arclengths[k][j] + tmp_t(0) * len;
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
