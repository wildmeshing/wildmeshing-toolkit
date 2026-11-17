#pragma once

#include <polysolve/nonlinear/Problem.hpp>
#include <wmtk/Types.hpp>

#include <cstring>

namespace wmtk::optimization {

namespace detail {
/**
 * The last point a pure function of x was evaluated at, and what it returned there, compared
 * bit for bit. A Newton solve asks for value() about 32 times at about 5.5 distinct points --
 * the line search and the stopping checks re-ask at the point just accepted -- so returning the
 * stored result for a repeat is the same answer for a fraction of the cost.
 */
template <int N, typename Result>
struct LastEvaluation
{
    bool valid = false;
    double at[N] = {};
    Result result{};

    template <typename Vec>
    bool matches(const Vec& x) const
    {
        return valid && std::memcmp(at, x.data(), sizeof(at)) == 0;
    }
    template <typename Vec>
    void store(const Vec& x, const Result& r)
    {
        std::memcpy(at, x.data(), sizeof(at));
        result = r;
        valid = true;
    }
};
} // namespace detail

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
    AMIPSEnergy2D(const std::vector<std::array<double, 6>>& cells, const double weight = 1);

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
    double m_weight;
    detail::LastEvaluation<2, double> m_last_value;
    detail::LastEvaluation<2, TVector> m_last_gradient;
};


class AMIPSEnergy3D : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;

    /**
     * @brief Sum of AMIPS energies for 3D tets.
     *
     * Each tet must be provided as an array of 12 values: {x0, y0, z0, x1, y1, x2, ...}.
     * The first three entries (x0, y0, z0) must be the same for all tets and will be replaced with
     * `x` during optimization.
     */
    AMIPSEnergy3D(const std::vector<std::array<double, 12>>& cells, const double weight = 1);

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
    std::vector<std::array<double, 12>> m_cells;
    double m_weight;
    detail::LastEvaluation<3, double> m_last_value;
    detail::LastEvaluation<3, TVector> m_last_gradient;
};

} // namespace wmtk::optimization