#pragma once

#include <polysolve/nonlinear/Problem.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>

namespace wmtk::optimization {

class EnvelopeEnergy2D : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;

    /**
     * @brief The energy is the squared distance to an envelope.
     *
     */
    EnvelopeEnergy2D(
        const std::shared_ptr<SampleEnvelope>& envelope,
        const std::array<Vector2d, 3>& pts,
        bool check_step_validity = true);

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
    std::shared_ptr<SampleEnvelope> m_envelope;
    std::array<Vector2d, 3> m_pts;

    bool m_check_step_validity;
};

} // namespace wmtk::optimization