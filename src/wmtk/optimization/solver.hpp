#pragma once

#include <polysolve/nonlinear/Solver.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::optimization {

const polysolve::json basic_linear_solver_params = R"({"solver": "Eigen::LDLT"})"_json;
// track_info off: the solver's info json and phase timings are kept for nobody -- wmtk reads only
// status() and current_criteria() -- and in a Newton solve of 2-3 unknowns run for every vertex of
// every smoothing pass, keeping them is a third of the solve. The iterates are the same.
const polysolve::json basic_nonlinear_solver_params = R"({
    "solver": "DenseNewton",
    "max_iterations": 10,
    "allow_out_of_iterations": true,
    "rel_grad_norm_tol": 0,
    "advanced": {"track_info": false}
})"_json;

inline std::unique_ptr<polysolve::nonlinear::Solver> create_basic_solver()
{
    return polysolve::nonlinear::Solver::create(
        basic_nonlinear_solver_params,
        basic_linear_solver_params,
        1,
        opt_logger());
}

inline void deactivate_opt_logger()
{
    opt_logger().set_level(spdlog::level::off);
}

} // namespace wmtk::optimization
