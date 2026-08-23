#pragma once

#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>

#include <wmtk/threading/enumerable_thread_specific.hpp>
#include "SimplicialComplexBVH.hpp"

#include <polysolve/nonlinear/Problem.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace wmtk::components::topological_offset {

/**
 * @brief WHAT DEFINES THE OFFSET: a scalar field on space and the level set the boundary sits on.
 *
 * Two implementations, chosen by the `offset_field` JSON option:
 *
 *   SmoothOffsetPotential   Phi = the offset geometric contact potential, level set Phi = c.
 *                           A C^2 field with analytic derivatives, whose level set is a SMOOTHED
 *                           offset -- it bulges outward at reentrant features. The default, and
 *                           the field every measurement in this component was tuned against.
 *
 *   EuclideanOffsetPotential  Phi = the Euclidean distance d to the input complex, level set
 *                           d = delta. The exact offset, with exact derivatives inside each
 *                           closest-feature region and a gradient discontinuity across the medial
 *                           axis, which is precisely what Phi was introduced to avoid.
 *
 * THE TWO RUN IN OPPOSITE DIRECTIONS and that is the one thing a caller must not get wrong. Phi is
 * a BARRIER: it is huge on the complex and decays to 0 at dhat, so being inside the offset region
 * means Phi >= c. The Euclidean distance INCREASES away from the complex, so inside means d <=
 * delta. Nothing outside this file should compare value() against target_level() itself -- ask
 * is_inside_offset(), which each implementation answers with its own sense.
 *
 * The rest of the interface is deliberately identical, so the optimization, the criterion, the
 * sizing field and OffsetEnergy are written once against this base and neither knows which field
 * it has.
 */
template <int DIM>
class OffsetPotential
{
    static_assert(DIM == 2 || DIM == 3, "the offset potential exists in 2D and 3D only");

public:
    using VecD = Eigen::Matrix<double, DIM, 1>;
    using MatD = Eigen::Matrix<double, DIM, DIM>;

    /// OUT OF LINE ON PURPOSE, so this class has a KEY FUNCTION and exactly one vtable, emitted
    /// beside the explicit instantiations in OffsetPotential.cpp. With every virtual pure and
    /// the destructor defaulted inline there is no key function, the vtable is emitted weakly
    /// into every translation unit that sees the header, and the linker picks one -- which is
    /// how a virtual call through a base pointer can end up jumping somewhere that is not code.
    virtual ~OffsetPotential();

    /// The level value the offset boundary is placed on.
    double target_level() const { return m_c; }

    /// The offset distance the field is calibrated to.
    double delta() const { return m_delta; }

    /// Support radius, beyond which the field and its derivatives vanish. Infinite for a field
    /// with no compact support, which makes within_support() vacuously true.
    double dhat() const { return m_dhat; }

    /**
     * @brief |d(field)/d(distance)| at the level set on a flat stretch of input.
     *
     * THE FACTOR THAT TURNS A FIELD DIFFERENCE INTO A LENGTH, and the reason a criterion stated
     * on the field's gradient is not automatically field-independent. The smoothing objective is
     * E = (Phi - c)^2, so |grad E| = 2 |Phi - c| |grad Phi|; writing |Phi - c| as
     * slope * residual_length and taking |grad Phi| ~ slope near the level set gives
     *
     *     |grad E| ~ 2 * slope^2 * residual_length
     *
     * so a bound on |grad E| is a bound on a LENGTH only after dividing by slope^2. It is 1 for a
     * distance field, where the two coincide and this is a no-op; it is 1/delta-ish for a
     * barrier, where ignoring it makes the bound tighter by that factor squared -- measured on
     * the two spheres at delta 0.0216: slope 27.62, so 763x tighter than intended.
     *
     * NOTE the |grad Phi| ~ slope step is LOCAL to the level set on a flat stretch. It is exactly
     * what the normalisation is calibrated for, and it is not an identity: see
     * offset_gradient_tolerance() for what the criterion becomes where it does not hold.
     *
     * See TopoOffsetTetMesh::offset_gradient_tolerance(), which is the only consumer.
     */
    double level_set_slope() const { return m_grad_ref; }

    virtual double value(const VecD& p) const = 0;
    virtual VecD gradient(const VecD& p) const = 0;
    virtual MatD hessian(const VecD& p) const = 0;

    /**
     * @brief Distance from `p` to the level set, in LENGTH units.
     *
     * What the convergence criterion measures, because it is comparable to target_distance while
     * the field value need not be.
     */
    virtual double residual_length(const VecD& p) const = 0;

    /// Whether `p` is somewhere the field can give a direction to the level set at all.
    virtual bool within_support(const VecD& p) const = 0;

    /**
     * @brief Whether `p` lies inside the offset region -- on the complex's side of the level set.
     *
     * ASK THIS, never `value(p) >= target_level()`. See the class comment: the two fields are
     * monotone in opposite directions, so the literal comparison is right for one and silently
     * inverted for the other. Used by the conservative construction tests in Spatial.cpp, where
     * getting it backwards would grow the band inside out.
     */
    virtual bool is_inside_offset(const VecD& p) const = 0;

    /// DIAGNOSTIC: what the field is made of at `p`, one contribution per line.
    virtual std::string describe_active(const VecD& p) const = 0;

protected:
    /// Subclasses set delta and the support here; m_c is theirs to fill in, because one
    /// calibrates it and the other simply knows it.
    OffsetPotential(const double delta, const double dhat)
        : m_delta(delta)
        , m_dhat(dhat)
    {}

    double m_delta = 0.;
    double m_dhat = 0.;
    double m_c = 0.;
    /// 1 unless a subclass calibrates otherwise -- see level_set_slope(). A distance field leaves
    /// it alone, which is what makes the gradient criterion mean the same thing on both.
    double m_grad_ref = 1.;
};

using OffsetPotential2D = OffsetPotential<2>;
using OffsetPotential3D = OffsetPotential<3>;


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
class SmoothOffsetPotential : public OffsetPotential<DIM>
{
    static_assert(DIM == 2 || DIM == 3, "the offset potential exists in 2D and 3D only");

public:
    using VecD = typename OffsetPotential<DIM>::VecD;
    using MatD = typename OffsetPotential<DIM>::MatD;

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
    SmoothOffsetPotential(
        const MatrixXd& V,
        const MatrixXi& E,
        const MatrixXi& F,
        const std::vector<int>& P,
        double delta,
        double dhat_factor);

    ~SmoothOffsetPotential() override;

    double value(const VecD& p) const override;
    VecD gradient(const VecD& p) const override;
    MatD hessian(const VecD& p) const override;

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
    double residual_length(const VecD& p) const override;

    /// Whether `p` is inside the support at all, i.e. Phi(p) > 0.
    bool within_support(const VecD& p) const override { return value(p) > 0.; }

    /// Phi DECREASES with distance, so the offset region is where it is still above the level.
    bool is_inside_offset(const VecD& p) const override { return value(p) >= m_c; }

    /// DIAGNOSTIC: the ACTIVE pairs at `p`, one per line -- what upstream's feasible-region rule
    /// decided claims this point, and each one's contribution. The only way to see why Phi has
    /// the value it has, which is what a discontinuity investigation needs.
    std::string describe_active(const VecD& p) const override;

private:
    // Dependent base members: name them unqualified in the definitions below.
    using OffsetPotential<DIM>::m_delta;
    using OffsetPotential<DIM>::m_dhat;
    using OffsetPotential<DIM>::m_c;
    using OffsetPotential<DIM>::m_grad_ref;

    /// Calibration constructor: builds the single-flat-primitive reference complex without
    /// recursing into calibration itself.
    SmoothOffsetPotential(double delta, double dhat_factor, int /*calibration tag*/);

    void build(const MatrixXd& V, const MatrixXi& E, const MatrixXi& F, const std::vector<int>& P);

    /// Everything that mentions ipc-toolkit, kept out of this header so that no other
    /// translation unit in the component has to see it.
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

using SmoothOffsetPotential2D = SmoothOffsetPotential<2>;
using SmoothOffsetPotential3D = SmoothOffsetPotential<3>;


/**
 * @brief THE EUCLIDEAN OFFSET: Phi = d(p, input complex), level set d = delta.
 *
 * The exact offset, in exchange for the smoothness Phi was introduced to buy. Within each
 * closest-feature region the distance to a piecewise-linear complex is smooth and its derivatives
 * are exact and cheap; ACROSS a region boundary -- the medial axis -- the gradient is
 * discontinuous, and at a reentrant feature the offset has a genuine crease that no amount of
 * refinement resolves. That is the trade being offered, not a defect of this class.
 *
 * DERIVATIVES, transcribed from wmtk::optimization::ExactDistanceEnergy2D/3D rather than derived
 * here. That class gives the Hessian of d^2 by feature kind; this one needs the Hessian of d, and
 * the two are related by grad^2(d^2) = 2 (grad d grad d^T + d grad^2 d), so with u = (p - n)/d:
 *
 *     feature            their grad^2(d^2) / 2      this class's grad^2 d
 *     face interior      n n^T                      0                          (d is linear)
 *     edge interior      I - t t^T                  (I - t t^T - u u^T) / d
 *     vertex             I                          (I - u u^T) / d
 *
 * with the 2D cases being the same statement one dimension down (segment interior -> 0, corner ->
 * (I - u u^T)/d). OffsetEnergy needs nothing else: it composes w (Phi - c)^2 from value, gradient
 * and Hessian by the chain rule, so w (d - delta)^2 and its exact derivatives fall out unchanged.
 *
 * ISOLATED INPUT POINTS ARE A DEGENERATE SEGMENT, not a special primitive: SimplicialComplexBVH
 * and the envelope both carry a point as the pseudo-edge (i, i). A query near one comes back as an
 * edge-interior hit whose direction is undefined, and the edge formula above would divide by a
 * meaningless t. Such a hit is demoted to the vertex case. topological_offset_2d_vertex_input is a
 * point cloud and is entirely made of them.
 *
 * NO SUPPORT LIMIT. d is defined and informative everywhere, so within_support() is always true
 * and the runaway guard that exists for Phi's compact support has nothing to catch. dhat() is
 * reported as infinity to make that visible rather than implied.
 */
template <int DIM>
class EuclideanOffsetPotential : public OffsetPotential<DIM>
{
    static_assert(DIM == 2 || DIM == 3, "the offset potential exists in 2D and 3D only");

public:
    using VecD = typename OffsetPotential<DIM>::VecD;
    using MatD = typename OffsetPotential<DIM>::MatD;

    /**
     * @brief Build over an EXACT-kind envelope of the input complex. THE 3D PATH.
     *
     * The envelope is the query engine, not a tolerance: nearest_point_feature() is what supplies
     * the foot point and the feature kind the derivatives are cased on, and only the exact kind
     * answers it. Its eps is irrelevant here and no containment test is ever run against it.
     * 2D stopped using this constructor: the mesh keeps exactly one structure over the input
     * complex -- the BVH below -- and building an envelope beside it duplicated the geometry.
     */
    EuclideanOffsetPotential(const std::shared_ptr<SampleEnvelope>& envelope, double delta);

    /**
     * @brief Build over the input-complex BVH. THE 2D PATH, and 2D-only (checked at runtime --
     * a static_assert would fire under the explicit template instantiation).
     *
     * The same query engine the rest of the pipeline already retains: built once when the mesh
     * is initialized, never rebuilt, and now also serving value() and nearest_feature() here,
     * both through the BVH's feature query -- the distance to the complex's CURVE (its edge
     * set), which for a solid complex is the boundary, never the solid's own zero interior. The
     * containment of the input complex is none of this object's business either way -- the per-tag
     * region envelopes hold it.
     */
    EuclideanOffsetPotential(const std::shared_ptr<SimplicialComplexBVH>& bvh, double delta);

    double value(const VecD& p) const override;
    VecD gradient(const VecD& p) const override;
    MatD hessian(const VecD& p) const override;

    /// EXACT, not first-order: the level set is d = delta, so the distance to it IS |d - delta|.
    /// No reference slope, hence none of the overstatement that the calibrated conversion carries
    /// wherever Phi's isolines are packed tighter than on a flat stretch.
    double residual_length(const VecD& p) const override { return std::abs(value(p) - m_c); }

    /// Everywhere. d has no compact support.
    bool within_support(const VecD& p) const override { return true; }

    /// d INCREASES with distance, so the offset region is where it is still below the level --
    /// the opposite sense to the smooth potential. See the base class.
    bool is_inside_offset(const VecD& p) const override { return value(p) <= m_c; }

    std::string describe_active(const VecD& p) const override;

private:
    /// The foot point, feature kind and direction at `p`, with the degenerate-segment demotion
    /// already applied. dim is 2 (face interior), 1 (edge interior) or 0 (vertex).
    void nearest_feature(const VecD& p, VecD& foot, int& dim, VecD& dir) const;

    using OffsetPotential<DIM>::m_delta;
    using OffsetPotential<DIM>::m_dhat;
    using OffsetPotential<DIM>::m_c;

    /// Exactly one of these is set, by whichever constructor ran: the envelope by the 3D path,
    /// the BVH by the 2D path. Every query branches on m_bvh.
    std::shared_ptr<SampleEnvelope> m_envelope;
    std::shared_ptr<SimplicialComplexBVH> m_bvh;
};

using EuclideanOffsetPotential2D = EuclideanOffsetPotential<2>;
using EuclideanOffsetPotential3D = EuclideanOffsetPotential<3>;


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
