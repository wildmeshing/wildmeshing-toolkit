#pragma once

// #define WMTK_SMOOTH_BARRIER

#include <ipc/collisions/normal/normal_collisions.hpp>
#include <ipc/ipc.hpp>
#include <ipc/potentials/barrier_potential.hpp>
#ifdef WMTK_SMOOTH_BARRIER
#include <ipc/smooth_contact/smooth_contact_potential.hpp>
#endif
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
     * The energy is defined over an entire polyline described by V and E. But the optimization only
     * considers one vertex with ID `vid`. The `vid` can be replaced to optimize another vertex
     * position later on.
     */
    BarrierEnergy2D(
        const MatrixXd& V,
        const MatrixXi& E,
        const size_t vid,
        const double dhat,
        const double weight = 1);

    TVector initial_position() const;

    void replace_vid(const size_t vid);

    MatrixXd& V() { return m_V; }

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
#ifdef WMTK_SMOOTH_BARRIER
    ipc::SmoothContactParameters m_params;
    ipc::SmoothCollisions m_smooth_collisions;
    ipc::SmoothContactPotential m_smooth_B;
#else
    ipc::NormalCollisions m_collisions;
    ipc::BarrierPotential m_B;
#endif


    Vector2d m_x0;
    size_t m_vid;

    double m_weight;
};

} // namespace wmtk::optimization