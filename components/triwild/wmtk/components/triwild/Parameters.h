#pragma once

namespace wmtk::components::triwild {
struct Parameters
{
    double epsr = 2e-3; // relative error bound (wrt diagonal)
    double eps = -1.; // absolute error bound
    double lr = 5e-2; // target edge length (relative)
    double l = -1.;
    double l_min = -1;
    double diag_l = -1.;
    Vector2d box_min = Vector2d::Zero();
    Vector2d box_max = Vector2d::Ones();
    bool preserve_topology = false;
    std::string output_path;

    double splitting_l2 = -1.; // the lower bound length (squared) for edge split
    double collapsing_l2 =
        std::numeric_limits<double>::max(); // the upper bound length (squared) for edge collapse

    double stop_energy = 10;

    bool debug_output = false;
    bool perform_sanity_checks = false;

    double w_amips = 1e-4;
    double w_envelope = 0;

    void init(const Vector2d& min_, const Vector2d& max_)
    {
        box_min = min_;
        box_max = max_;
        diag_l = (box_max - box_min).norm();
        if (l > 0) {
            lr = l / diag_l;
        } else {
            l = lr * diag_l;
        }
        splitting_l2 = l * l * (16 / 9.);
        collapsing_l2 = l * l * (16 / 25.);

        if (eps > 0) {
            epsr = eps / diag_l;
        } else {
            eps = epsr * diag_l;
        }

        l_min = eps;
    }
    void init(
        const std::vector<Vector2d>& vertices,
        const std::vector<std::array<size_t, 3>>& faces)
    {
        Vector2d min_, max_;
        for (size_t i = 0; i < vertices.size(); i++) {
            if (i == 0) {
                min_ = vertices[i];
                max_ = vertices[i];
                continue;
            }
            for (int j = 0; j < 2; j++) {
                if (vertices[i][j] < min_[j]) {
                    min_[j] = vertices[i][j];
                }
                if (vertices[i][j] > max_[j]) {
                    max_[j] = vertices[i][j];
                }
            }
        }

        init(min_, max_);
    }
};
} // namespace wmtk::components::triwild
