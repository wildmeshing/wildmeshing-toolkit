#include "BoundaryParametrization.h"
namespace wmtk {
void Boundary::construct_boudaries(Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    std::vector<std::vector<int>> paths;
    igl::boundary_loop(F, paths);

    for (auto p : paths) {
        std::vector<Eigen::Vector2d> boundary;
        std::vector<double> arclength;
        double len = 0.; // cumulative length till current vertex
        arclength.emplace_back(len);
        for (auto i = 0; i < p.size(); i++) {
            boundary.emplace_back(V.row(p[i]));
            len += (V.row(p[i]) - V.row(p[(i + 1) % p.size()])).norm();
            arclength.emplace_back(len);
        }
        assert(arclength.size() == boundary.size() + 1);
        m_boundaries.emplace_back(boundary);
        m_arclengties.emplace_back(arclength);
    }
}
Eigen::Vector2d Boundary::t_to_uv(int i, double t)
{
    const auto& arclength = m_arclengties[i];
    while (t < 0) t += arclength.back();
    assert(t < arclength.back());
    t = std::fmod(t, arclength.back());
    auto it = std::prev(std::upper_bound(arclength.begin(), arclength.end(), t));
    auto a = std::distance(arclength.begin(), it);
    assert((a + 1) < arclength.size());

    double r = t - *it;
    const auto& boundary = m_boundaries[i];
    assert(a < boundary.size());
    Eigen::Vector2d A = boundary[a];
    Eigen::Vector2d B = boundary[(a + 1) % boundary.size()];
    auto n = (B - A) / (arclength[a + 1] - arclength[a]);
    assert(std::pow((n.squaredNorm() - 1), 2) < 1e-8);
    return A + r * n;
}
double Boundary::uv_to_t(const Eigen::Vector2d& v)
{
    double ret_t = 0.;
    Eigen::MatrixXd P;
    P.resize(1, 2);
    P.row(0) = v.transpose();
    Eigen::VectorXd tmp_t, tmp_d;
    double d = std::numeric_limits<double>::infinity();

    for (auto i = 0; i < m_boundaries.size(); i++) {
        for (auto j = 0; j < m_boundaries[i].size(); j++) {
            Eigen::RowVectorXd A = m_boundaries[i][j];
            Eigen::RowVectorXd B = m_boundaries[i][(j + 1) % m_boundaries[i].size()];
            igl::project_to_line_segment(P, A, B, tmp_t, tmp_d);
            if (tmp_d(0) < d) {
                d = tmp_d(0);
                ret_t = tmp_t(0);
            }
        }
    }
    return ret_t;
}
} // namespace wmtk