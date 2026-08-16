#pragma once

#include <wmtk/Types.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>

#include <polysolve/nonlinear/Problem.hpp>

#include <memory>
#include <vector>

namespace wmtk::components::topological_offset {

/**
 * @brief The SMOOTH OFFSET POTENTIAL Phi, and the offset defined as its level set Phi = c.
 *
 * WHY THIS EXISTS. The offset used to be defined by the exact Euclidean distance to the input
 * complex, and a boundary vertex was placed by projecting it along (p - nearest)/|p - nearest|
 * to distance delta. That function is non-smooth exactly where the offset is hardest: it is
 * gradient-discontinuous across the medial axis, undefined ON the complex, and kinked at every
 * feature. Every wart in the old project_offset_vertex() traced back to it -- the |p-n| < 1e-12
 * bail-out, the ten-step bisection clamp, the golden-section tangential slide, and the whole
 * normal-deviation apparatus that existed to detect features the distance field could not
 * express. It had no usable gradient, so the placement could not be an ENERGY, so it could not
 * go through the shared smoother, so the offset boundary was the one surface in the component
 * that bypassed every check the shared smoother makes.
 *
 * Phi replaces it with a C^2 field that has an analytic gradient and Hessian, so placing a
 * vertex on the offset becomes an ordinary term in the smoothing objective and the offset
 * boundary is smoothed by exactly the same code path as every other vertex.
 *
 * WHAT PHI IS. The offset geometric contact potential of ipc-toolkit's `high_order_contact`
 * subtree, evaluated at a point q against the input complex:
 *
 *     Phi(q) = sum over ACTIVE primitives P of  b( dist(q, P), dhat )
 *     b(d, dhat) = -(d/dhat - 1)^2 * log(d/dhat)   for d < dhat, 0 otherwise
 *
 * (`ipc::NormalizedClampedLogBarrier`). "Active" is the OGC feasible-region rule: an edge is
 * active at q when q projects into its interior, a vertex when q lies in its Voronoi wedge --
 * so away from features exactly one primitive contributes and Phi is a monotone function of
 * the Euclidean distance alone. At a reentrant corner BOTH adjacent edges contribute, their
 * barriers add, and the level set bulges outward: that is the smoothing, and it is the whole
 * point. Convex corners are the mirror image -- one vertex contributes and the level set is
 * the exact circular arc.
 *
 * So Phi = c is a SMOOTHED offset, not the Euclidean one, and that difference is deliberate.
 * The Euclidean distance is still computed and reported everywhere it was before, as a
 * diagnostic, so every run says how far the two ended up apart.
 *
 * CALIBRATION. `c` is not a free parameter. It is Phi evaluated at perpendicular distance delta
 * from a long straight edge -- one active pair, no feature interaction -- computed at
 * construction time through this very class, so it cannot drift from an analytic formula that
 * someone later has to keep in step. With one active pair Phi is strictly decreasing in
 * distance, so on any flat stretch of the input the level set Phi = c IS the delta-offset, and
 * curvature moves it by an amount that shrinks with delta/radius. See tests/test_offset_potential.cpp
 * for the measured deviations on circles, squares, wedges and isolated points.
 *
 * dhat -- the support radius, beyond which Phi and every derivative are identically zero -- is
 * `dhat_factor * delta` with dhat_factor 2 by default. Two considerations pin it: delta must sit
 * well inside the support (at exactly dhat the potential is 0 with zero gradient, so a vertex
 * there gets NO direction to move in), and the support must not be so wide that distant parts of
 * the complex contribute to the level set. A vertex that ends up beyond dhat is a hard error,
 * not a warning -- see TopoOffsetTriMesh::check_offset_within_support().
 *
 * THREADING. The evaluation writes the query point into a scratch vertex matrix and builds a
 * collision set around it, so it holds per-thread state; `value`, `gradient` and `hessian` are
 * const and safe to call concurrently from the smoothing pass, which is what the offset does.
 */
class OffsetPotential
{
public:
    /**
     * @brief Build the potential of a fixed 2D complex.
     *
     * @param V         #V x 2 complex vertices.
     * @param E         #E x 2 segments of the complex (for a solid input region, its BOUNDARY --
     *                  see TopoOffsetTriMesh::init_input_complex_bvh, which extracts them).
     * @param P         indices into V of the isolated complex vertices (in no segment).
     * @param delta     the offset distance the level set is calibrated to.
     * @param dhat_factor  support radius as a multiple of delta. Must be > 1.
     */
    OffsetPotential(
        const MatrixXd& V,
        const MatrixXi& E,
        const std::vector<int>& P,
        double delta,
        double dhat_factor);

    ~OffsetPotential();

    /// The level value the offset boundary is placed on: Phi at distance `delta` from a long
    /// straight edge, evaluated through this same code path at construction.
    double target_level() const { return m_c; }

    /// Support radius. Phi and all its derivatives vanish identically beyond this.
    double dhat() const { return m_dhat; }

    /// The offset distance the level set was calibrated to.
    double delta() const { return m_delta; }

    double value(const Vector2d& p) const;
    Vector2d gradient(const Vector2d& p) const;
    Matrix2d hessian(const Vector2d& p) const;

    /**
     * @brief First-order distance from `p` to the level set Phi = c, in LENGTH units.
     *
     * |Phi(p) - c| / ||grad Phi(p)||, which is the Newton step to the level set of a function
     * linearised at p. This is what the convergence criterion measures, because it is
     * comparable to target_distance while Phi itself is not (Phi is a barrier value).
     *
     * Returns +infinity where the gradient vanishes, i.e. outside the support -- there is no
     * direction to the level set from there, and the run is meant to stop rather than pretend
     * otherwise.
     */
    double residual_length(const Vector2d& p) const;

    /// Whether `p` is inside the support at all, i.e. Phi(p) > 0.
    bool within_support(const Vector2d& p) const { return value(p) > 0.; }

private:
    /// Calibration constructor: builds the single-straight-edge reference complex without
    /// recursing into calibration itself.
    OffsetPotential(double delta, double dhat_factor, int /*calibration tag*/);

    void build(const MatrixXd& V, const MatrixXi& E, const std::vector<int>& P);

    /// Everything that mentions ipc-toolkit, kept out of this header so that no other
    /// translation unit in the component has to see it.
    struct Impl;
    std::unique_ptr<Impl> m_impl;

    double m_delta = 0.;
    double m_dhat = 0.;
    double m_c = 0.;
    /// |dPhi/dd| at distance delta from a long straight edge -- the slope of the potential at
    /// the level set on a flat stretch of input. Fixed at construction, and what turns a
    /// difference in Phi into a length. See residual_length().
    double m_grad_ref = 0.;
};


/**
 * @brief The offset term of the smoothing objective: w * (Phi(x) - c)^2.
 *
 * A polysolve::nonlinear::Problem in exactly the shape of ExactDistanceEnergy2D, so the shared
 * 2D smoother composes it into its EnergySum beside AMIPS with no special case -- which is what
 * lets an offset-boundary vertex take the same path as every other vertex.
 *
 * The residual form rather than Phi itself: Phi is a barrier, so minimising it would drive the
 * vertex to infinity and maximising it into the complex. The square of the residual has its
 * minimum exactly ON the level set, which is where the offset boundary belongs.
 *
 * Value, gradient and Hessian all follow from Phi, grad Phi and hess Phi by the chain rule:
 *
 *     E     = w (Phi - c)^2
 *     grad  = 2 w (Phi - c) grad Phi
 *     hess  = 2 w [ grad Phi grad Phi^T + (Phi - c) hess Phi ]
 *
 * The second Hessian term changes sign with the residual and can make H indefinite far from
 * the level set; `gauss_newton` drops it, leaving the always-PSD outer product. On by default,
 * for the same reason Gauss-Newton is the default anywhere else: the dropped term vanishes at
 * the solution, so it costs nothing at convergence and buys a descent direction everywhere.
 */
class OffsetEnergy2D : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;

    OffsetEnergy2D(
        const std::shared_ptr<const OffsetPotential>& potential,
        double weight = 1.,
        bool gauss_newton = true);

    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, MatrixXd& hessian) override;

    void solution_changed(const TVector& new_x) override {}

private:
    std::shared_ptr<const OffsetPotential> m_potential;
    double m_weight;
    bool m_gauss_newton;
};

} // namespace wmtk::components::topological_offset
