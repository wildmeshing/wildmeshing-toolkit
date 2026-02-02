#pragma once
#include <wmtk/Types.hpp>

namespace wmtk::components::tet_implicits {
struct Parameters
{
    // parameters set by user
    double epsr = 2e-3; // relative error bound (wrt diagonal)
    double eps = -1.; // absolute error bound
    double eps2 = -1;
    double dr = 5e-2; // target edge length (relative)
    double d = -1.; // target edge length (absolute)
    double d2 = -1; // squared d
    std::vector<std::pair<int64_t, int64_t>> input_tags;
    std::vector<std::pair<int64_t, int64_t>> output_tags;

    bool preserve_topology = false;
    std::string output_path;

    // parameters set in `init` function based on mesh bbox
    double diag_l = -1.;
    Vector3d box_min = Vector3d::Zero();
    Vector3d box_max = Vector3d::Ones();

    bool debug_output = false;
    bool perform_sanity_checks = false;

    void init(const Vector3d& min_, const Vector3d& max_)
    {
        box_min = min_;
        box_max = max_;
        diag_l = (box_max - box_min).norm();
        if (d > 0) {
            dr = d / diag_l;
        } else {
            d = dr * diag_l;
        }
        d2 = d * d;

        if (eps > 0) {
            epsr = eps / diag_l;
        } else {
            eps = epsr * diag_l;
        }
        eps2 = eps * eps;
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
} // namespace wmtk::components::tet_implicits
