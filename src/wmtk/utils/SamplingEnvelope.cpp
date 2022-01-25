#include "SamplingEnvelope.hpp"

namespace wmtk {
SamplingEnvelope::SamplingEnvelope(double sampling_dist, double box_tolerance)
    : m_sampling_dist(sampling_dist)
    , m_tol(box_tolerance)
{}

void SamplingEnvelope::init(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, double eps)
{
    m_V = V;
    m_F = F;

    m_bvh.init(m_V, m_F, m_tol);
    m_eps = eps;
}

bool SamplingEnvelope::is_outside(const std::array<Eigen::Vector3d, 3>& tri) const
{
    Eigen::Vector3d min = tri[0];
    Eigen::Vector3d max = tri[0];

    for (int i = 1; i < 3; ++i) {
        for (int d = 0; d < 3; ++d) {
            min[d] = std::min(min[d], tri[i][d]);
            max[d] = std::max(max[d], tri[i][d]);
        }
    }

    std::vector<unsigned int> candidates;
    m_bvh.intersect_box(min, max, candidates);

    if (candidates.empty()) return true;

    std::vector<Eigen::Vector3d> samples;
    sample_tri(tri, samples);

    for (const auto i : candidates) {
        const std::array<Eigen::Vector3d, 3> t_i = {
            {m_V.row(m_F(i, 0)), m_V.row(m_F(i, 1)), m_V.row(m_F(i, 2))}};

        for (const auto& s : samples) {
            if (point_tri_distance(t_i, s) > m_eps) return true;
        }
    }

    return false;
}

void SamplingEnvelope::sample_tri(
    const std::array<Eigen::Vector3d, 3>& tri,
    std::vector<Eigen::Vector3d> samples) const
{
    double sq_dist = std::numeric_limits<double>::max();
    const double sqrt3_2 = std::sqrt(3) / 2;

    std::array<double, 3> ls;
    for (int i = 0; i < 3; i++) {
        ls[i] = (tri[i] - tri[(i + 1) % 3]).squaredNorm();
    }

    auto min_max = std::minmax_element(ls.begin(), ls.end());
    int min_i = min_max.first - ls.begin();
    int max_i = min_max.second - ls.begin();

    double N = sqrt(ls[max_i]) / m_sampling_dist;
    if (N <= 1) {
        for (int i = 0; i < 3; i++) samples.push_back(tri[i]);

        return;
    }

    if (N == int(N)) N -= 1;

    const auto v0 = tri[max_i];
    const auto v1 = tri[(max_i + 1) % 3];
    const auto v2 = tri[(max_i + 2) % 3];
    const auto n_v0v1 = (v1 - v0).normalized();

    for (int n = 0; n <= N; ++n) {
        samples.push_back(v0 + n_v0v1 * m_sampling_dist * n);
    }
    samples.push_back(v1);

    const double h = ((v2 - v0).dot(v1 - v0) * (v1 - v0) / ls[max_i] + v0 - v2).norm();
    const int M = h / (sqrt3_2 * m_sampling_dist);
    if (M < 1) {
        samples.push_back(v2);
        return;
    }

    const auto n_v0v2 = (v2 - v0).normalized();
    const auto n_v1v2 = (v2 - v1).normalized();

    const double sin_v0 = (((v2 - v0).cross(v1 - v0)) / (v0 - v2).norm() / (v0 - v1).norm()).norm();
    const double tan_v0 = (((v2 - v0).cross(v1 - v0)) / ((v2 - v0).dot(v1 - v0))).norm();
    const double tan_v1 = (((v2 - v1).cross(v0 - v1)) / ((v2 - v1).dot(v0 - v1))).norm();
    const double sin_v1 = (((v2 - v1).cross(v0 - v1)) / (v1 - v2).norm() / (v0 - v1).norm()).norm();

    for (int m = 1; m <= M; m++) {
        int n = sqrt3_2 / tan_v0 * m + 0.5;
        int n1 = sqrt3_2 / tan_v0 * m;
        if (m % 2 == 0 && n == n1) {
            n += 1;
        }
        const Eigen::Vector3d v0_m = v0 + m * sqrt3_2 * m_sampling_dist / sin_v0 * n_v0v2;
        const Eigen::Vector3d v1_m = v1 + m * sqrt3_2 * m_sampling_dist / sin_v1 * n_v1v2;

        if ((v0_m - v1_m).norm() <= m_sampling_dist) break;

        const double delta_d = ((n + (m % 2) / 2.0) - m * sqrt3_2 / tan_v0) * m_sampling_dist;

        const Eigen::Vector3d v = v0_m + delta_d * n_v0v1;
        const int N1 = (v - v1_m).norm() / m_sampling_dist;
        for (int i = 0; i <= N1; i++) {
            samples.push_back(v + i * n_v0v1 * m_sampling_dist);
        }
    }
    samples.push_back(v2);

    // sample edges
    N = sqrt(ls[(max_i + 1) % 3]) / m_sampling_dist;
    if (N > 1) {
        if (N == int(N)) N -= 1;

        const Eigen::Vector3d n_v1v2 = (v2 - v1).normalized();

        for (int n = 1; n <= N; n++) {
            samples.push_back(v1 + n_v1v2 * m_sampling_dist * n);
        }
    }

    N = sqrt(ls[(max_i + 2) % 3]) / m_sampling_dist;
    if (N > 1) {
        if (N == int(N)) N -= 1;

        const Eigen::Vector3d n_v2v0 = (v0 - v2).normalized();
        for (int n = 1; n <= N; n++) {
            samples.push_back(v2 + n_v2v0 * m_sampling_dist * n);
        }
    }
}

double SamplingEnvelope::point_tri_distance(
    const std::array<Eigen::Vector3d, 3>& tri,
    const Eigen::Vector3d& p) const
{
    Eigen::Vector3d n = (tri[1] - tri[0]).cross(tri[2] - tri[0]);
    const double n_len = n.norm();

    if (n_len < 1e-30) return 0;

    n /= n_len;

    // Project point p onto the plane spanned by a->b and a->c.
    const double dist = p.dot(n) - tri[0].dot(n);

    // Project p onto the plane by stepping the distance from p to the plane
    // in the direction opposite the normal: proj = p - dist * n
    const Eigen::Vector3d proj = p - dist * n;

    // Find out if the projected point falls within the triangle -- see:
    // http://blackpawn.com/texts/pointinpoly/default.html

    // Compute edge vectors
    const double v0x = tri[2](0) - tri[0](0);
    const double v0y = tri[2](1) - tri[0](1);
    const double v0z = tri[2](2) - tri[0](2);
    const double v1x = tri[1](0) - tri[0](0);
    const double v1y = tri[1](1) - tri[0](1);
    const double v1z = tri[1](2) - tri[0](2);
    const double v2x = proj(0) - tri[0](0);
    const double v2y = proj(1) - tri[0](1);
    const double v2z = proj(2) - tri[0](2);

    // Compute dot products
    const double dot00 = v0x * v0x + v0y * v0y + v0z * v0z;
    const double dot01 = v0x * v1x + v0y * v1y + v0z * v1z;
    const double dot02 = v0x * v2x + v0y * v2y + v0z * v2z;
    const double dot11 = v1x * v1x + v1y * v1y + v1z * v1z;
    const double dot12 = v1x * v2x + v1y * v2y + v1z * v2z;

    // Compute barycentric coordinates (u, v) of projection point
    const double denom = (dot00 * dot11 - dot01 * dot01);
    if (fabs(denom) < 1e-30) return 0;

    const double invDenom = 1.0 / denom;
    const double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    const double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    // Check barycentric coordinates
    if ((u >= 0) && (v >= 0) && (u + v < 1)) {
        // Nearest orthogonal projection point is in triangle
        return (p - proj).norm();
    } else {
        // TODO
        return 0;
    }
}

} // namespace wmtk
