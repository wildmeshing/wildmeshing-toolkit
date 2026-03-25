#pragma once
#include <wmtk/Types.hpp>

namespace wmtk::components::image_simulation {
struct Parameters
{
    // parameters set by user
    double epsr = 2e-3; // relative error bound (wrt diagonal)
    double eps = -1.; // absolute error bound
    double lr = 5e-2; // target edge length (relative)
    double l = -1.; // target edge length (absolute)
    double l_min = -1;
    bool preserve_topology = false;
    std::string output_path;

    // parameters set in `init` function based on mesh bbox
    double diag_l = -1.;
    double diag_l2 = -1.;
    double diag_l3 = -1.;
    double diag_l4 = -1.;
    VectorXd box_min;
    VectorXd box_max;
    double vol = -1; // bbox volume
    double splitting_l2 = -1.; // the lower bound length (squared) for edge split
    double collapsing_l2 =
        std::numeric_limits<double>::max(); // the upper bound length (squared) for edge collapse

    double stop_energy = 10;

    bool debug_output = false;
    bool perform_sanity_checks = false;

    bool smooth_without_envelope = false;

    // weighting terms for the optimization
    double w_amips = 1;
    double w_smooth = 1;
    double w_envelope = 1;
    double w_separate = 1;

    double dhat_rel = 2e-3;
    double dhat = -1;

    void init(const VectorXd& min_, const VectorXd& max_)
    {
        box_min = min_;
        box_max = max_;
        vol = (box_max - box_min).prod();
        diag_l = (box_max - box_min).norm();
        diag_l2 = diag_l * diag_l;
        diag_l3 = diag_l * diag_l * diag_l;
        diag_l4 = diag_l * diag_l * diag_l * diag_l;
        if (l > 0)
            lr = l / diag_l;
        else
            l = lr * diag_l;
        splitting_l2 = l * l * (16 / 9.);
        collapsing_l2 = l * l * (16 / 25.);

        if (eps > 0) {
            epsr = eps / diag_l;
        } else {
            eps = epsr * diag_l;
        }

        l_min = 0.5 * eps;

        if (dhat > 0) {
            dhat_rel = dhat / diag_l;
        } else {
            dhat = dhat_rel * diag_l;
        }
    }
    void init(
        const std::vector<Vector3d>& vertices,
        const std::vector<std::array<size_t, 3>>& faces)
    {
        Vector3d min_, max_;
        for (size_t i = 0; i < vertices.size(); i++) {
            if (i == 0) {
                min_ = vertices[i];
                max_ = vertices[i];
                continue;
            }
            for (int j = 0; j < 3; j++) {
                if (vertices[i][j] < min_[j]) min_[j] = vertices[i][j];
                if (vertices[i][j] > max_[j]) max_[j] = vertices[i][j];
            }
        }

        init(min_, max_);
    }
};
} // namespace wmtk::components::image_simulation
