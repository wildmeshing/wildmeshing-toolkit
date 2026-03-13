#pragma once

#include <polysolve/nonlinear/Problem.hpp>
#include <wmtk/Types.hpp>

namespace wmtk::optimization {

class DirichletEnergy2D : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;

    /**
     * @brief The Dirichlet energy of a vertex position on a polyline.
     *
     * Each edge must be provided as an array of 4 values: {x0, y0, x1, y1}.
     * The first two entries (x0, y0) must be the same for all edges and will be replaced with
     * `x` during optimization.
     */
    DirichletEnergy2D(std::vector<std::array<double, 4>>& cells);

    TVector initial_position() const;

    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, MatrixXd& hessian) override;

    void solution_changed(const TVector& new_x) override {}

private:
    std::vector<std::array<double, 4>> m_cells;
};

class SmoothingEnergy2D : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;

    /**
     * @brief The smoothing energy of a vertex position on a polyline.
     *
     * Three positions must be provided, the optimized position (p1) and its two neighbors (p0, p2).
     */
    SmoothingEnergy2D(std::vector<std::array<double, 4>> m_cells);

    TVector initial_position() const;

    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, MatrixXd& hessian) override;

    void solution_changed(const TVector& new_x) override {}

    void add_mass_and_stiffness_matrix(const double& M, const Vector3d& L_w);

    static void compute_local_mass_and_stiffness(
        std::vector<std::array<double, 4>> m_cells,
        double& M,
        Vector3d& L_w);

private:
    MatrixXd m_x;
    Vector3d m_L_w;
    double m_M;
    double m_M_inv;

    Vector3d m_LTML_row1; // derived from the other matrices
};

} // namespace wmtk::optimization