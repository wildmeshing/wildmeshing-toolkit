#pragma once
#include <nlohmann/json.hpp>
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
    VectorXd box_min;
    VectorXd box_max;
    double splitting_l2 = -1.; // the lower bound length (squared) for edge split
    double collapsing_l2 =
        std::numeric_limits<double>::max(); // the upper bound length (squared) for edge collapse

    double stop_energy = 10;
    bool stop_at_float = false;

    bool debug_output = false;
    bool perform_sanity_checks = false;

    // weighting terms for the optimization
    double w_amips = 1e-4;
    double w_envelope = 0;

    std::string operation = "remeshing";

    bool skip_simplify = true;
    bool use_sample_envelope = false;
    int NUM_THREADS = 0;
    int max_its = 80;
    bool write_vtu = false;
    bool write_envelope = true;

    Parameters() = default;

    Parameters(const nlohmann::json& json_params)
    {
        output_path = json_params["output"];
        skip_simplify = json_params["skip_simplify"];
        use_sample_envelope = json_params["use_sample_envelope"];
        NUM_THREADS = json_params["num_threads"];
        max_its = json_params["max_iterations"];
        write_vtu = json_params["write_vtu"];
        write_envelope = json_params["write_envelope"];

        epsr = json_params["eps_rel"];
        eps = json_params["eps"];
        lr = json_params["length_rel"];
        l = json_params["length"];
        stop_energy = json_params["stop_energy"];
        stop_at_float = json_params["stop_at_float"];
        preserve_topology = json_params["preserve_topology"];

        epsr_simplify = json_params["eps_simplify_rel"];
        eps_simplify = json_params["eps_simplify"];

        w_amips = json_params["w_amips"];

        debug_output = json_params["DEBUG_output"];
        perform_sanity_checks = json_params["DEBUG_sanity_checks"];

        operation = json_params["operation"];
    }

    void init(const VectorXd& min_, const VectorXd& max_)
    {
        box_min = min_;
        box_max = max_;
        diag_l = (box_max - box_min).norm();
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
