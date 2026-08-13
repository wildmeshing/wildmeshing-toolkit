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


/**
 * @brief The true w * d^2 to the exact piecewise-linear input with its TRUE Hessian.
 *
 * For a PL input the distance is exactly quadratic within each closest-feature region: the
 * Hessian of w*d^2 is 2w * n n^T (n the segment normal) when the foot point lies in a
 * segment interior, and 2w * I when it is a polyline corner -- distance to a point is
 * isotropic. Both are the actual second derivative, so unlike EnvelopeEnergy2D (whose
 * hessian is the Gauss-Newton model) all three callbacks agree under finite differences,
 * everywhere except exactly on a region boundary.
 *
 * Behaviorally: free sliding along segment interiors, isotropic hold at corners -- the
 * geometry decides where tangential motion is free, not a policy.
 */
class ExactDistanceEnergy2D : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;

    ExactDistanceEnergy2D(const std::shared_ptr<SampleEnvelope>& envelope, const double weight = 1);

    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, MatrixXd& hessian) override;

    void solution_changed(const TVector& new_x) override {}

    /**
     * Clamp a proposed step at the closest-feature region boundary, so Newton lands on the
     * kink of d^2 instead of overshooting across it and rediscovering it by backtracking.
     * Returns 1 when the classification is unchanged over the step. Helps when the
     * iteration budget can absorb the extra, shorter steps; see the commit message for the
     * measured trade.
     */
    double max_step_size(const TVector& x0, const TVector& x1) override;

private:
    std::shared_ptr<SampleEnvelope> m_envelope;
    double m_weight;
};


/// 3D counterpart of ExactDistanceEnergy2D. The true Hessian of w*d^2 to a triangle mesh is
/// 2w * n n^T over a face interior (n the face normal), 2w * (I - t t^T) over an edge
/// interior (t the edge direction), and 2w * I at a mesh vertex -- each region exactly
/// quadratic, all PSD, all the actual second derivative.
class ExactDistanceEnergy3D : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;

    ExactDistanceEnergy3D(const std::shared_ptr<SampleEnvelope>& envelope, const double weight = 1);

    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, MatrixXd& hessian) override;

    void solution_changed(const TVector& new_x) override {}

    /// See ExactDistanceEnergy2D::max_step_size; identical contract, 3D features.
    double max_step_size(const TVector& x0, const TVector& x1) override;

private:
    std::shared_ptr<SampleEnvelope> m_envelope;
    double m_weight;
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