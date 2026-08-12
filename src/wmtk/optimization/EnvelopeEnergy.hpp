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
    EnvelopeEnergy2D(const std::shared_ptr<SampleEnvelope>& envelope, const double weight = 1);

    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, MatrixXd& hessian) override;

    void solution_changed(const TVector& new_x) override;

private:
    std::shared_ptr<SampleEnvelope> m_envelope;
    double m_weight;
};

/**
 * @brief w * |x - t|^2 with t a FIXED target point, the no-sliding alternative to
 * EnvelopeEnergy2D.
 *
 * EnvelopeEnergy penalises the distance to the input as a SET: it recomputes the nearest
 * point at every evaluation, so its gradient 2w*(x - n(x)) is always normal to the input and
 * a vertex can slide along it at no cost. That is what the rank-one Gauss-Newton hessian
 * encodes, and it is deliberate -- sliding is what lets a surface vertex relax along the
 * curve it sits on.
 *
 * It also has a consequence. Under a competing term the vertex is pushed tangentially with
 * no restoring force at all, so it slides until the geometry rather than the force balance
 * stops it, and the leftover distance no longer depends on the weight. Measured on
 * triwild20k_202090 that floor is ~4e-5 and raising the weight by four further decades does
 * not move it.
 *
 * This is the other choice: capture the nearest point once, before the solve, and spring the
 * vertex to that single point. The energy is then an exact quadratic, the hessian is the full
 * 2w*I, and tangential motion is resisted exactly as strongly as normal motion. No sliding,
 * but also no floor -- the same sweep gives a clean 1/w over six decades.
 */
class SpringEnergy2D : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;

    /**
     * @param refresh_envelope when non-null, the target is re-captured as this envelope's
     * nearest point at every accepted iterate (via solution_changed); when null the target
     * stays fixed for the whole solve.
     */
    SpringEnergy2D(
        const Vector2d& target,
        const double weight = 1,
        const std::shared_ptr<SampleEnvelope>& refresh_envelope = nullptr);

    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, MatrixXd& hessian) override;

    void solution_changed(const TVector& new_x) override;
    bool after_line_search_custom_operation(const TVector& x0, const TVector& x1) override
    {
        return m_refresh_envelope != nullptr;
    }

private:
    Vector2d m_target;
    double m_weight;
    std::shared_ptr<SampleEnvelope> m_refresh_envelope;
};

class EnvelopeEnergy3D : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;

    /**
     * @brief The energy is the squared distance to an envelope.
     *
     */
    EnvelopeEnergy3D(const std::shared_ptr<SampleEnvelope>& envelope, const double weight = 1);

    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, MatrixXd& hessian) override;

    void solution_changed(const TVector& new_x) override;

private:
    std::shared_ptr<SampleEnvelope> m_envelope;
    double m_weight;
};

/// 3D counterpart of SpringEnergy2D; see there for what the fixed target buys and costs.
class SpringEnergy3D : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;

    /**
     * @param refresh_envelope when non-null, the target is re-captured as this envelope's
     * nearest point at every accepted iterate (via solution_changed); when null the target
     * stays fixed for the whole solve.
     */
    SpringEnergy3D(
        const Vector3d& target,
        const double weight = 1,
        const std::shared_ptr<SampleEnvelope>& refresh_envelope = nullptr);

    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, MatrixXd& hessian) override;

    void solution_changed(const TVector& new_x) override;
    bool after_line_search_custom_operation(const TVector& x0, const TVector& x1) override
    {
        return m_refresh_envelope != nullptr;
    }

private:
    Vector3d m_target;
    double m_weight;
    std::shared_ptr<SampleEnvelope> m_refresh_envelope;
};

} // namespace wmtk::optimization