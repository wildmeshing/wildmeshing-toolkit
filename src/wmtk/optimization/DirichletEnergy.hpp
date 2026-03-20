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

class BiharmonicEnergy2D : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;

    /**
     * @brief The biharmonic energy of a vertex position on a polyline.
     *
     * The energy is defined as:
     * \int_M \Vert \Delta_M p \Vert^2
     * In descrete form:
     * \sum_{i=1}^n A_i \Vert L p_i \Vert^2
     *
     * Three positions must be provided, the optimized position (p0) and its two neighbors (p1, p2).
     *
     */
    BiharmonicEnergy2D(const std::array<Vector2d, 3>& pts, const double& M, const Vector3d& L_w);

    TVector initial_position() const;

    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, MatrixXd& hessian) override;

    void solution_changed(const TVector& new_x) override {}

    static void
    local_mass_and_stiffness(const std::array<Vector2d, 3>& pts, double& M, Vector3d& L_w);

    static void
    uniform_mass_and_stiffness(const std::array<Vector2d, 3>& pts, double& M, Vector3d& L_w);

private:
    /**
     * The optimized point and its (two) neighbors. For the optimization, we use the convention that
     * m_pts[0] is the optimized vertex and the neighbors m_pts[1] and m_pts[2] are fixed.
     */
    std::array<Vector2d, 3> m_pts;
    /**
     * The first row of stiffness matrix L_w which belongs to the optimized vertex. By convention,
     * this needs to be sorted such that the first entry belongs to the optimized vertex.
     *
     * The full stiffness matrix L_w has shape NxN (more specifically in our case 3x3 as we have
     * three points), but the rows of fixed vertices are empty. Therefore, we only need to store the
     * one non-empty row of the optimized vertex. By convention of m_pts, this must be the first
     * one, i.e., L_w.row(0).
     */
    Vector3d m_L_w_row;
    double m_M; // the mass of the optimized vertex
    double m_M_inv; // the inverse mass, 1 / M

    /**
     * We only need the first row of LTML, because all other vertices are fixed.
     *
     * L = M_inv * L_w
     * LTML = L^T * M * L = L_w^T * M_inv * L_w
     * We only want to optimize one vertex, so we only need one row of LTML:
     * LTML.row(0) = M_inv * L_w(0,0) * L_w.row(0) <-- this is what is stored in m_LTML_row
     */
    Vector3d m_LTML_row;
};

} // namespace wmtk::optimization