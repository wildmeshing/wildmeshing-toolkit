#pragma once

#include <polysolve/nonlinear/Problem.hpp>
#include <wmtk/Types.hpp>

namespace wmtk::optimization {

class AMIPSEnergy2D : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;

    /**
     * @brief Sum of AMIPS energies for 2D triangles
     *
     * Each triangle must be provided as an array of 6 values: {x0, y0, x1, y1, x2, y2}.
     * The first two entries (x0, y0) must be the same for all triangles and will be replaced with
     * `x` during optimization.
     */
    AMIPSEnergy2D(std::vector<std::array<double, 6>>& cells, bool area_weighted = false);

    TVector initial_position() const;

    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, MatrixXd& hessian) override;

    void solution_changed(const TVector& new_x) override;

    bool is_step_valid(const TVector& x0, const TVector& x1) override;

private:
    std::vector<std::array<double, 6>> m_cells;
    std::vector<double> m_weights;
};

} // namespace wmtk::optimization