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
 * @brief The true w * d^2 to the exact piecewise-linear input with its TRUE Hessian.
 *
 * Within each closest-feature region the distance to a PL input is exactly quadratic, so
 * the true Hessian is computable and cheap: 2w n n^T over a segment interior (n the segment
 * normal), 2w I at a polyline corner. Value and gradient are the true distance and its true
 * gradient, re-queried fresh at every evaluation; unlike the Gauss-Newton model, all three
 * callbacks agree under finite differences everywhere off a region boundary.
 *
 * Behaviorally, the geometry decides where sliding is free: segment interiors slide,
 * corners hold isotropically.
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

private:
    std::shared_ptr<SampleEnvelope> m_envelope;
    double m_weight;
};

/// 3D counterpart, serving triangle envelopes (2w n n^T on faces, 2w (I - t t^T) on edges,
/// 2w I at vertices) and order-2 CURVE envelopes (rank two on segment interiors, isotropic
/// at curve vertices) through the same feature query.
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

} // namespace wmtk::optimization