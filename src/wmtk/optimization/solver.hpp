#pragma once

#include <polysolve/nonlinear/Solver.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::optimization {

const polysolve::json basic_linear_solver_params = R"({"solver": "Eigen::LDLT"})"_json;
const polysolve::json basic_nonlinear_solver_params =
    R"({"solver": "DenseNewton", "max_iterations": 10, "allow_out_of_iterations": true})"_json;

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
