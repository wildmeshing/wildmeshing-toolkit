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

    double epsr_simplify = 2e-3; // relative error bound (wrt diagonal) for simplification
    double eps_simplify = -1.; // absolute error bound for simplification

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
    bool stop_at_float = false;

    bool debug_output = false;
    bool perform_sanity_checks = false;

    bool smooth_without_envelope = false;

    // weighting terms for the optimization
    double w_amips = 1;
    double w_smooth = 0;
    double w_envelope = 0;
    double w_separate = 0;

    double dhat = -1;
    double separation_factor = 1;

    std::string operation = "remeshing";

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

        if (eps_simplify > 0) {
            epsr_simplify = eps_simplify / diag_l;
        } else {
            eps_simplify = epsr_simplify * diag_l;
        }

        l_min = 0.5 * eps;

        dhat = separation_factor * l_min;
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
