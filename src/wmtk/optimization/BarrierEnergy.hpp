#pragma once

#include <ipc/collisions/normal/normal_collisions.hpp>
#include <ipc/ipc.hpp>
#include <ipc/potentials/barrier_potential.hpp>
#include <polysolve/nonlinear/Problem.hpp>
#include <wmtk/Types.hpp>

namespace wmtk::optimization {

class BarrierEnergy2D : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;

    /**
     * @brief Barrier energy for a polyline in 2D
     *
     * Each triangle must be provided as an array of 6 values: {x0, y0, x1, y1, x2, y2}.
     * The first two entries (x0, y0) must be the same for all triangles and will be replaced with
     * `x` during optimization.
     */
    BarrierEnergy2D(const MatrixXd& V, const MatrixXi& E, const size_t vid, const double dhat);

    TVector initial_position() const;

    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, MatrixXd& hessian) override;

    void solution_changed(const TVector& new_x) override {}

    bool is_step_valid(const TVector& x0, const TVector& x1) override { return true; }

    void update_collisions(const TVector& x);

private:
    ipc::CollisionMesh m_collision_mesh;
    MatrixXd m_V;
    ipc::NormalCollisions m_collisions;
    ipc::BarrierPotential m_B;

    Vector2d m_x0;
    size_t m_vid;
};

} // namespace wmtk::optimization