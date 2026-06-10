#pragma once

#include <polysolve/nonlinear/Problem.hpp>
#include <wmtk/Types.hpp>

namespace wmtk::optimization {

class EnergySum : public polysolve::nonlinear::Problem
{
public:
    using Problem = polysolve::nonlinear::Problem;

    using typename Problem::Scalar;
    using typename Problem::THessian;
    using typename Problem::TVector;

    /**
     * @brief A weighted sum of multiple energies.
     */
    EnergySum() = default;

    /**
     * @brief Add an energy term to the sum of energies.
     */
    void add_energy(const std::shared_ptr<Problem>& energy, const double weight = 1);

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
    std::vector<std::shared_ptr<Problem>> m_energies;
    std::vector<double> m_weights;
};

} // namespace wmtk::optimization