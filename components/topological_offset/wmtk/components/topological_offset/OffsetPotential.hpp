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
 * to distance delta (2D), or by a quadric fit to sampled offset planes (3D). That function is
 * non-smooth exactly where the offset is hardest: it is gradient-discontinuous across the medial
 * axis, undefined ON the complex, and kinked at every feature. Every wart in the old
 * project_offset_vertex() and smooth_after_offset_surface() traced back to it -- the
 * |p-n| < 1e-12 bail-out, the ten-step bisection clamp, the golden-section tangential slide, the
 * quadrics/Laplacian blend weights, and the whole normal-deviation apparatus that existed to
 * detect features the distance field could not express. It had no usable gradient, so the
 * placement could not be an ENERGY, so it could not go through the shared smoother, so the offset
 * boundary was the one surface in the component that bypassed every check the shared smoother
 * makes.
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
 * (`ipc::NormalizedClampedLogBarrier`). "Active" is the OGC feasible-region rule: a triangle is
 * active at q when q projects into its interior, an edge when q projects into its interior AND
 * lies outside the wedges its incident triangles claim, a vertex when q lies in its Voronoi
 * region -- so away from features exactly one primitive contributes and Phi is a monotone
 * function of the Euclidean distance alone. At a reentrant feature SEVERAL primitives
 * contribute, their barriers add, and the level set bulges outward: that is the smoothing, and
 * it is the whole point. Convex features are the mirror image -- one vertex or edge contributes
 * and the level set is the exact spherical or cylindrical patch.
 *
 * So Phi = c is a SMOOTHED offset, not the Euclidean one, and that difference is deliberate.
 * The Euclidean distance is still computed and reported everywhere it was before, as a
 * diagnostic, so every run says how far the two ended up apart.
 *
 * CALIBRATION. `c` is not a free parameter. It is Phi evaluated at perpendicular distance delta
 * from one large flat primitive -- a straight edge in 2D, a large triangle in 3D; one active
 * pair, no feature interaction -- computed at construction time through this very class, so it
 * cannot drift from an analytic formula that someone later has to keep in step. With one active
 * pair Phi is strictly decreasing in distance, so on any flat stretch of the input the level set
 * Phi = c IS the delta-offset, and curvature moves it by an amount that shrinks with
 * delta/radius. Because the barrier only ever sees a distance, the two dimensions calibrate to
 * the SAME c for the same delta and dhat_factor -- tests/test_offset_potential.cpp asserts it.
 * See that file for the measured deviations on circles, spheres, squares, boxes, wedges and
 * isolated points.
 *
 * dhat -- the support radius, beyond which Phi and every derivative are identically zero -- is
 * `dhat_factor * delta` with dhat_factor 2 by default. Two considerations pin it: delta must sit
 * well inside the support (at exactly dhat the potential is 0 with zero gradient, so a vertex
 * there gets NO direction to move in), and the support must not be so wide that distant parts of
 * the complex contribute to the level set. A vertex that ends up beyond dhat is a hard error,
 * not a warning -- see TopoOffsetTriMesh::check_offset_within_support() and its 3D twin.
 *
 * THREADING. The evaluation writes the query point into a scratch vertex matrix and builds a
 * collision set around it, so it holds per-thread state; `value`, `gradient` and `hessian` are
 * const and safe to call concurrently from the smoothing pass, which is what the offset does.
 */
template <int DIM>
class OffsetPotential
{
    static_assert(DIM == 2 || DIM == 3, "the offset potential exists in 2D and 3D only");

public:
    using VecD = Eigen::Matrix<double, DIM, 1>;
    using MatD = Eigen::Matrix<double, DIM, DIM>;

    /**
     * @brief Build the potential of a fixed complex.
     *
     * The complex is given as ipc gives a collision mesh -- vertices, edges, triangles -- plus
     * the indices of its isolated points. A SOLID input region enters as its BOUNDARY (see
     * TopoOffsetTriMesh::init_input_complex_bvh and TopoOffsetTetMesh::init_input_complex_bvh,
     * which extract them), because Phi has no area primitive in 2D and no volume primitive in
     * 3D; outside the region, which is the only place an offset exists, the two descriptions
     * agree exactly.
     *
     * @param V         #V x DIM complex vertices.
     * @param E         #E x 2 segments. In 3D this must contain EVERY edge of every triangle in
     *                  `F` as well as the complex's own isolated edges: ipc derives
     *                  faces_to_edges from it and throws if an edge of a face is missing, and
     *                  the OGC feasible-region test for a vertex reads its edge neighbours.
     * @param F         #F x 3 triangles. Must be empty when DIM == 2.
     * @param P         indices into V of the isolated complex vertices (in no segment/triangle).
     * @param delta     the offset distance the level set is calibrated to.
     * @param dhat_factor  support radius as a multiple of delta. Must be > 1.
     */
    OffsetPotential(
        const MatrixXd& V,
        const MatrixXi& E,
        const MatrixXi& F,
        const std::vector<int>& P,
        double delta,
        double dhat_factor);

    ~OffsetPotential();

    /// The level value the offset boundary is placed on: Phi at distance `delta` from one large
    /// flat primitive, evaluated through this same code path at construction.
    double target_level() const { return m_c; }

    /// Support radius. Phi and all its derivatives vanish identically beyond this.
    double dhat() const { return m_dhat; }

    /// The offset distance the level set was calibrated to.
    double delta() const { return m_delta; }

    double value(const VecD& p) const;
    VecD gradient(const VecD& p) const;
    MatD hessian(const VecD& p) const;

    /**
     * @brief First-order distance from `p` to the level set Phi = c, in LENGTH units.
     *
     * |Phi(p) - c| / |grad Phi| at the level set, which is the Newton step to the level set of a
     * function linearised at p. This is what the convergence criterion measures, because it is
     * comparable to target_distance while Phi itself is not (Phi is a barrier value).
     *
     * Saturates outside the support -- there is no direction to the level set from there, and
     * the runaway guard turns that state into a hard error rather than letting this number
     * decide anything.
     */
    double residual_length(const VecD& p) const;

    /// Whether `p` is inside the support at all, i.e. Phi(p) > 0.
    bool within_support(const VecD& p) const { return value(p) > 0.; }

    /// DIAGNOSTIC: the ACTIVE pairs at `p`, one per line -- what upstream's feasible-region rule
    /// decided claims this point, and each one's contribution. The only way to see why Phi has
    /// the value it has, which is what a discontinuity investigation needs.
    std::string describe_active(const VecD& p) const;

private:
    /// Calibration constructor: builds the single-flat-primitive reference complex without
    /// recursing into calibration itself.
    OffsetPotential(double delta, double dhat_factor, int /*calibration tag*/);

    void build(const MatrixXd& V, const MatrixXi& E, const MatrixXi& F, const std::vector<int>& P);

    /// Everything that mentions ipc-toolkit, kept out of this header so that no other
    /// translation unit in the component has to see it.
    struct Impl;
    std::unique_ptr<Impl> m_impl;

    double m_delta = 0.;
    double m_dhat = 0.;
    double m_c = 0.;
    /// |dPhi/dd| at distance delta from one large flat primitive -- the slope of the potential at
    /// the level set on a flat stretch of input. Fixed at construction, and what turns a
    /// difference in Phi into a length. See residual_length().
    double m_grad_ref = 0.;
};

using OffsetPotential2D = OffsetPotential<2>;
using OffsetPotential3D = OffsetPotential<3>;


/**
 * @brief The offset term of the smoothing objective: w * (Phi(x) - c)^2.
 *
 * A polysolve::nonlinear::Problem in exactly the shape of ExactDistanceEnergy2D/3D, so the shared
 * smoother composes it into its EnergySum beside AMIPS with no special case -- which is what lets
 * an offset-boundary vertex take the same path as every other vertex.
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
template <int DIM>
class OffsetEnergy : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;

    using VecD = Eigen::Matrix<double, DIM, 1>;
    using MatD = Eigen::Matrix<double, DIM, DIM>;

    OffsetEnergy(
        const std::shared_ptr<const OffsetPotential<DIM>>& potential,
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
    std::shared_ptr<const OffsetPotential<DIM>> m_potential;
    double m_weight;
    bool m_gauss_newton;
};

using OffsetEnergy2D = OffsetEnergy<2>;
using OffsetEnergy3D = OffsetEnergy<3>;

} // namespace wmtk::components::topological_offset
