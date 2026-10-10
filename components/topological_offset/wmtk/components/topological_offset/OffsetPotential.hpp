#pragma once

#include <functional>

#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>

#include <wmtk/threading/enumerable_thread_specific.hpp>
#include "SimplicialComplexBVH.hpp"

#include <polysolve/nonlinear/Problem.hpp>

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace wmtk::components::topological_offset {

/**
 * @brief The offset potential: the scalar field on space whose level set the front is placed on.
 *
 * Two implementations, chosen by the `offset_field` JSON option: SmoothOffsetPotential (a C^2
 * barrier with analytic derivatives, level set Phi = c, a smoothed offset) and
 * EuclideanOffsetPotential (the distance d to the input complex, level set d = delta, the exact
 * offset). Each is documented at its own declaration below.
 *
 * The two are monotone in opposite directions, which is the one thing a caller must not get wrong:
 * the barrier is huge on the complex and decays to 0 at dhat, so inside the offset region means
 * Phi >= c, while the distance increases outward, so inside means d <= delta. Never compare
 * value() against target_level() outside this file -- ask is_inside_offset(), which each
 * implementation answers with its own sense.
 *
 * The rest of the interface is identical, so the optimization, the criterion, the sizing field and
 * OffsetEnergy are written once against this base and never learn which field they have.
 */
template <int DIM>
class OffsetPotential
{
    static_assert(DIM == 2 || DIM == 3, "the offset potential exists in 2D and 3D only");

public:
    using VecD = Eigen::Matrix<double, DIM, 1>;
    using MatD = Eigen::Matrix<double, DIM, DIM>;

    /// Must stay out of line: it is this class's key function, so exactly one vtable is emitted,
    /// beside the explicit instantiations in OffsetPotential.cpp. Defaulted inline, there is no key
    /// function and the vtable goes weakly into every translation unit that sees the header.
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
     * The factor that turns a field difference into a length, so a criterion stated on the field's
     * gradient is not by itself field-independent. The smoothing objective is E = (Phi - c)^2, so
     * |grad E| ~ 2 * slope^2 * residual_length near the level set: a bound on |grad E| bounds a
     * length only after dividing by slope^2. It is 1 for a distance field, where the two coincide,
     * and 1/delta-ish for a barrier. The |grad Phi| ~ slope step is local to the level set on a
     * flat stretch, not an identity.
     *
     * Consumer: OffsetEnergy's distance_residual branch.
     */
    double level_set_slope() const { return m_grad_ref; }

    virtual double value(const VecD& p) const = 0;
    /// The Euclidean field: value() is the distance, so a distance residual is the plain one.
    virtual bool is_euclidean() const { return false; }
    virtual VecD gradient(const VecD& p) const = 0;
    virtual MatD hessian(const VecD& p) const = 0;

    /// value() and gradient() at one point, bit for bit. A field whose two evaluations share work
    /// overrides it; the default is simply the two calls.
    virtual void value_gradient(const VecD& p, double& v, VecD& g) const
    {
        v = value(p);
        g = gradient(p);
    }

    /**
     * @brief Distance from `p` to the level set, in length units.
     *
     * What the convergence criterion measures: comparable to target_distance, while the field
     * value need not be.
     */
    virtual double residual_length(const VecD& p) const = 0;

    /**
     * @brief The signed distance from `p` to the level set, as a fraction of delta: > 0 outside
     * the offset region, < 0 inside, non-finite where it cannot be measured.
     *
     * THE number the 3D loop's measures square and compare against front_conv_frac()
     * (TopoOffsetTetMesh::face_conv_ratio(), front_vertex_conv_ratio(), edge_conv_ratio()), so
     * that under every field those measures are lengths over the length bar front_conv. The loop
     * used to form the relative FIELD error (Phi - c)/c itself, which is this number only for the
     * Euclidean field -- see SmoothOffsetPotential::relative_residual() for what it was for the
     * smooth one.
     */
    virtual double relative_residual(const VecD& p) const = 0;

    /// Whether `p` is somewhere the field can give a direction to the level set at all.
    virtual bool within_support(const VecD& p) const = 0;

    /**
     * @brief Whether `p` lies inside the offset region -- on the complex's side of the level set.
     *
     * Ask this, never `value(p) >= target_level()`: the two fields are monotone in opposite
     * directions, so the literal comparison is right for one and silently inverted for the other.
     */
    virtual bool is_inside_offset(const VecD& p) const = 0;

    /// Diagnostic: what the field is made of at `p`, one contribution per line.
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
    /// 1 unless a subclass calibrates otherwise -- see level_set_slope(). Leaving it alone is what
    /// makes the gradient criterion mean the same thing on both fields.
    double m_grad_ref = 1.;
};

using OffsetPotential2D = OffsetPotential<2>;
using OffsetPotential3D = OffsetPotential<3>;


/**
 * @brief The smooth offset potential Phi, and the offset defined as its level set Phi = c.
 *
 * Phi is C^2 with an analytic gradient and Hessian, so placing a vertex on the offset is an
 * ordinary term in the smoothing objective and the front is smoothed by the same code path as
 * every other vertex.
 *
 * What Phi is: the Extremum-Sum Potential (ESP) of ipc-toolkit's `esp` subtree
 * (ipc::ArbitraryPointESP), evaluated at a point q against the input complex,
 *
 *     Phi(q) = sum over elements s of  w_s * b( dist(q, s), dhat )
 *     b(d, dhat) = -(d/dhat - 1)^2 * log(d/dhat)   for d < dhat, 0 otherwise
 *
 * (`ipc::NormalizedClampedLogBarrier`), dist being to the closed element. The integer weights make
 * every point of the complex count once (ESP supplemental, S2): a triangle weighs 1, an edge
 * 1 - (triangles on it), a vertex 1 - (edges at it) + (triangles at it). A closed surface gets
 * +1/-1/+1 (a closed curve +1/-1), the boundary of an open sheet and the ends of an open curve
 * weigh 0, segments in no triangle and isolated points 1. Phi is b of the Euclidean distance
 * wherever the complex within every radius r < dhat of q is one piece without holes: bitwise
 * outside the cube, on any flat stretch. Where it is several pieces -- two walls within dhat: a
 * reentrant edge, a gap narrower than (1 + dhat_factor) delta -- each adds its own term, Phi is
 * larger and the level set moves outward. Measured in 3D at delta 0.1, dhat_factor 2: a
 * 90-degree reentrant edge rounds to a fillet reaching 1.175 delta; two faces 2.5 delta apart
 * hold their level sets at 1.041 delta; gaps up to 2.36 delta close, against 2 delta for the
 * Euclidean offset. Near a sharp convex vertex the ball can also wrap around the vertex before
 * reaching it (measured: Phi about 0.3% off b just outside an octahedron's apex). So Phi = c is
 * a smoothed offset, not the Euclidean one, and that difference is deliberate; the Euclidean
 * distance is still reported as a diagnostic.
 *
 * Calibration: `c` is not a free parameter. It is Phi at perpendicular distance delta from one
 * large flat primitive -- one active pair, no feature interaction -- computed at construction
 * through this same class, so it cannot drift from a hand-kept analytic formula. Both dimensions
 * therefore calibrate to the same c for the same delta and dhat_factor, which
 * tests/test_offset_potential.cpp asserts.
 *
 * dhat, the support radius beyond which Phi and every derivative are identically zero, is
 * `dhat_factor * delta`. delta must sit strictly inside the support (at exactly dhat the potential
 * and its gradient are both 0, so a vertex there gets no direction to move in) and the support
 * must not be so wide that distant parts of the complex reach the level set. A vertex beyond dhat
 * is a hard error -- see TopoOffsetTriMesh::check_offset_within_support() and its 3D twin.
 *
 * Threading: ipc builds the collision set around each query point in per-thread scratch, so
 * `value`, `gradient` and `hessian` are const and safe to call concurrently from the smoothing
 * pass.
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
     * The complex is given as ipc gives a collision mesh -- vertices, edges, triangles -- plus the
     * indices of its isolated points. Phi has no area primitive in 2D and no volume primitive in
     * 3D, so a solid input region must enter as its boundary; outside the region, the only place
     * an offset exists, the two descriptions agree exactly.
     *
     * @param V         #V x DIM complex vertices.
     * @param E         #E x 2 segments. In 3D this must contain every edge of every triangle in
     *                  `F` as well as the complex's own isolated edges: ipc derives
     *                  faces_to_edges from it and throws if an edge of a face is missing, and an
     *                  edge's weight counts the triangles on it.
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
    /// value() and gradient() at p from one collision build; see the definition.
    void value_gradient(const VecD& p, double& v, VecD& g) const override;

    /**
     * @brief The distance from `p` to the level set Phi = c ALONG THE FIELD, in length units.
     *
     * |t| for the root t of Phi(p + t n) = c nearest p along n = grad Phi(p) / |grad Phi(p)|, on
     * p's own side of the complex. No calibration constant and no reference geometry enter: it is
     * a distance in the field's own geometry, and where Phi is a function of the Euclidean
     * distance alone (one active pair) it is exactly |d - delta|. +infinity where it cannot be
     * measured -- outside the support (Phi = 0, no direction), on the complex (Phi infinite),
     * where grad Phi vanishes, or where no root lies along n within dhat. The runaway guard turns
     * the first of those into a hard error before this number decides anything. See the
     * definition for the search and its tolerance.
     */
    double residual_length(const VecD& p) const override;

    /**
     * @brief The same root, signed and over delta: t / delta, > 0 outside the offset region.
     * NaN where residual_length() is infinite.
     *
     * NOT the relative field error (Phi - c)/c, which the 3D loop measured before 2026-09-27 and
     * which is a length only for a field linear in the distance. For this one it is the length
     * error times the logarithmic slope delta |dPhi/dd| / c at the level set, which for
     * b(d) = -(d/dhat - 1)^2 ln(d/dhat) is 2/(k - 1) + 1/ln k with k = dhat/delta: 3.4427 at the
     * default k = 2, 6.47 at k = 1.5, 1.91 at k = 3, unbounded as k -> 1. Measured on the cube
     * (target_distance_rel 1e-2, front_conv_rel 1e-4, 10 threads): (Phi - c)/c over the true
     * relative distance error was 3.443 on every flat side, rounded edge and corner at convergence
     * (the regional 3.4 / 3.3 / 3.1 of turns 1 to 4 are the same factor at vertices several bars
     * off the level set, where b is visibly curved), and since the sag goes as h^2 that cost one
     * extra halving: 9 turns and 80054 front faces against the Euclidean field's 7 and 25006 on the
     * same offset surface.
     */
    double relative_residual(const VecD& p) const override;

    /// Whether `p` is inside the support at all, i.e. Phi(p) > 0.
    bool within_support(const VecD& p) const override { return value(p) > 0.; }

    /// Phi decreases with distance, so the offset region is where it is still above the level.
    bool is_inside_offset(const VecD& p) const override { return value(p) >= m_c; }

    /// Diagnostic: Phi, |grad Phi| and the trace of the Hessian at `p`.
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

    /// The signed distance t from p to the level set along the field (> 0 outside the offset
    /// region), or false where there is none to measure. residual_length() is |t|,
    /// relative_residual() t / delta.
    bool level_set_distance(const VecD& p, double& t) const;

    /// Everything that mentions ipc-toolkit, kept out of this header so that no other
    /// translation unit in the component has to see it.
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

using SmoothOffsetPotential2D = SmoothOffsetPotential<2>;
using SmoothOffsetPotential3D = SmoothOffsetPotential<3>;


/**
 * @brief The Euclidean offset: Phi = d(p, input complex), level set d = delta.
 *
 * The exact offset, in exchange for smoothness. Within each closest-feature region the distance to
 * a piecewise-linear complex is smooth and its derivatives are exact and cheap; across a region
 * boundary -- the medial axis -- the gradient is discontinuous, and at a reentrant feature the
 * offset has a crease no refinement resolves. That is the trade, not a defect of this class.
 *
 * Derivatives are transcribed from wmtk::optimization::ExactDistanceEnergy2D/3D rather than
 * re-derived. That class gives the Hessian of d^2 by feature kind, this one needs the Hessian of
 * d, and grad^2(d^2) = 2 (grad d grad d^T + d grad^2 d) relates them; with u = (p - n)/d:
 *
 *     feature            their grad^2(d^2) / 2      this class's grad^2 d
 *     face interior      n n^T                      0                          (d is linear)
 *     edge interior      I - t t^T                  (I - t t^T - u u^T) / d
 *     vertex             I                          (I - u u^T) / d
 *
 * with the 2D cases the same statement one dimension down (segment interior -> 0, corner ->
 * (I - u u^T)/d). OffsetEnergy needs nothing else: it composes w (Phi - c)^2 from value, gradient
 * and Hessian by the chain rule.
 *
 * An isolated input point is a degenerate segment, not a special primitive: SimplicialComplexBVH
 * and the envelope both carry it as the pseudo-edge (i, i), so a query near one comes back as an
 * edge-interior hit with an undefined direction and must be demoted to the vertex case.
 *
 * No support limit: d is defined and informative everywhere, so within_support() is always true
 * and dhat() is reported as infinity rather than as a large finite number.
 */
template <int DIM>
class EuclideanOffsetPotential : public OffsetPotential<DIM>
{
    static_assert(DIM == 2 || DIM == 3, "the offset potential exists in 2D and 3D only");

public:
    bool is_euclidean() const override { return true; }
    using VecD = typename OffsetPotential<DIM>::VecD;
    using MatD = typename OffsetPotential<DIM>::MatD;

    /**
     * @brief Build over an exact-kind envelope of the input complex. The 3D path.
     *
     * The envelope is the query engine here, not a tolerance: nearest_point_feature() supplies the
     * foot point and the feature kind the derivatives are cased on, and only the exact kind answers
     * it. Its eps is irrelevant and no containment test is run against it.
     */
    EuclideanOffsetPotential(const std::shared_ptr<SampleEnvelope>& envelope, double delta);

    /**
     * @brief Build over the input-complex BVH. The 2D path, and 2D-only -- checked at runtime,
     * because a static_assert would fire under the explicit template instantiation.
     *
     * value() and nearest_feature() both go through the BVH's feature query, i.e. the distance to
     * the complex's curve (its edge set), which for a solid complex is its boundary and never the
     * solid's own zero interior. Containment of the input complex is not this object's business;
     * the per-tag region envelopes hold it.
     */
    EuclideanOffsetPotential(const std::shared_ptr<SimplicialComplexBVH>& bvh, double delta);

    double value(const VecD& p) const override;
    VecD gradient(const VecD& p) const override;
    MatD hessian(const VecD& p) const override;

    /// Exact, not first-order: the level set is d = delta, so the distance to it is |d - delta|.
    /// value() is d / delta (see the constructors), so dividing by the slope returns length units.
    double residual_length(const VecD& p) const override
    {
        return std::abs(value(p) - m_c) / m_grad_ref;
    }

    /// (d - delta)/delta: value() is d / delta and the level is 1, so the relative field error is
    /// already the relative distance error. Written as the (value - c)/c the 3D loop formed itself
    /// before it asked the field, so the Euclidean path computes what it always did, bit for bit.
    double relative_residual(const VecD& p) const override { return (value(p) - m_c) / m_c; }

    /// Everywhere. d has no compact support.
    bool within_support(const VecD& p) const override { return true; }

    /// d increases with distance, so the offset region is where it is still below the level --
    /// the opposite sense to the smooth potential. See the base class.
    bool is_inside_offset(const VecD& p) const override { return value(p) <= m_c; }

    std::string describe_active(const VecD& p) const override;

private:
    /// The foot point, feature kind and direction at `p`, with the degenerate-segment demotion
    /// already applied. dim is 2 (face interior), 1 (edge interior) or 0 (vertex).
    void nearest_feature(const VecD& p, VecD& foot, int& dim, VecD& dir) const;

    using OffsetPotential<DIM>::m_delta;
    using OffsetPotential<DIM>::m_grad_ref;
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
 * A polysolve::nonlinear::Problem in the shape of ExactDistanceEnergy2D/3D, so the shared smoother
 * composes it into its EnergySum beside AMIPS with no special case, which is what lets a front
 * vertex take the same path as every other vertex.
 *
 * The residual form rather than Phi itself: Phi is a barrier, so minimising it would drive the
 * vertex to infinity and maximising it into the complex, while the squared residual has its
 * minimum exactly on the level set, where the front belongs.
 *
 * Value, gradient and Hessian all follow from Phi, grad Phi and hess Phi by the chain rule:
 *
 *     E     = w (Phi - c)^2
 *     grad  = 2 w (Phi - c) grad Phi
 *     hess  = 2 w [ grad Phi grad Phi^T + (Phi - c) hess Phi ]
 *
 * The second Hessian term changes sign with the residual and can make H indefinite far from the
 * level set; `gauss_newton` drops it, leaving the always-PSD outer product. On by default: the
 * dropped term vanishes at the solution, so it costs nothing at convergence and buys a descent
 * direction everywhere.
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

    /// distance_residual charges the signed distance from x to the level set along the field's
    /// normal, over delta -- what (d - delta)/delta already is for the Euclidean field -- instead
    /// of the value ratio (Phi - c)/c, which for the smooth field is a barrier value and not a
    /// length, and pulls far harder where two fronts are pressed together. Same level set and same
    /// root either way; only the charge changes. Both dimensions' front placement passes true.
    ///
    /// one_sided charges only points on the input's side of the level set
    /// (is_inside_offset()): beyond it the term and its derivatives are zero, so the term pushes a
    /// vertex out to the level set and never pulls one in. C^1 at the level set, where the
    /// residual is zero from both sides. The repulsion passes before the march use it
    /// (TopoOffsetTetMesh::repulsion_smoothing()).
    OffsetEnergy(
        const std::shared_ptr<const OffsetPotential<DIM>>& potential,
        double weight = 1.,
        bool gauss_newton = true,
        bool distance_residual = false,
        bool one_sided = false);

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
    bool m_distance_residual;
    bool m_one_sided;
    /// r and its gradient under either residual (see the constructor).
    void residual(const VecD& p, double& r, VecD& dr) const;
};

using OffsetEnergy2D = OffsetEnergy<2>;
using OffsetEnergy3D = OffsetEnergy<3>;

/**
 * @brief THE offset term of a front vertex's smoothing objective: the mean squared relative
 * error of the field over the stencil of each incident offset face.
 *
 *     E(x) = w * sum over the vertex's incident offset faces f of
 *                (1/N_s) * sum over f's N_s stencil points i of  r(q_i(x))^2
 *
 *     r(p) = (Phi(p) - c) / c,      q_i(x) = a_i x + b_i q1 + c_i q2
 *
 * with c the target level and (a_i, b_i, c_i) the barycentric weights of stencil point i, a_i
 * being the MOVING vertex's own weight. The face's other two corners q1, q2 are fixed for the
 * visit. The stencil is TopoOffsetTetMesh::for_each_face_sample, sized by stencil_order.
 *
 * UNITS: the front smoother passes w = 1 / front_conv_frac()^2
 * (TopoOffsetTetMesh::offset_term_weight()), which puts r in units of the tolerance: each face's
 * term is then TopoOffsetTetMesh::face_offset_term(), 1 at the bar, the very term the per-tet
 * energy (TopoOffsetTetMesh::tet_energy()) adds to the face's band cell, and E is the sum of
 * those terms over the vertex's faces. Exactly so for the euclidean field, where r IS
 * relative_residual(); see below for the smooth one.
 *
 * THIS ONE TERM REPLACES BOTH the placement term (OffsetEnergy3D on the vertex alone) and the
 * sag term (SagEnergy3D over the face interiors) that preceded it, because the stencil contains
 * the corners: at order 0 the stencil IS the three corners, so E is exactly the placement
 * residual of the vertex and its neighbours, and every higher order adds interior points that
 * ask the same question between them. There is nothing left for a separate sag measure to say.
 *
 * r IS THE PLAIN RELATIVE ERROR (Phi - c)/c, which for the euclidean field is exactly
 * (d - target_distance)/target_distance -- the same residual OffsetEnergy3D uses there. For the
 * SMOOTH field OffsetEnergy3D instead divides by g_ref * delta to get a monotone length; this
 * class does not, so under `offset_field: "smooth"` the two are scaled differently.
 *
 * NOTE THE PER-FACE MEAN, SUMMED OVER FACES, with no area weighting and no other per-face weight:
 * a vertex with V incident faces contributes its own r(x)^2 with coefficient V/N_s, since it is a
 * stencil point of every one of them. The energy therefore grows with valence, which the AMIPS
 * term beside it does too. The per-tet energy has no area in it, so neither has this, nor the
 * loop's ring measure; the A_f / A_mean face weights front_measure "vertex_ring" put here from
 * 2026-09-25 went on 2026-09-28. (On the units under "smooth": the criterion and the per-tet
 * energy read OffsetPotential::relative_residual(), which there is the distance to the level set
 * over delta, while r here stays the field's own relative error -- the same zero set, about
 * 3.44x the criterion's number at the default offset_dhat_factor.)
 *
 * AREA WEIGHTING (set_area_weighted(), EXPERIMENTAL_area_weighted_ring): the term becomes
 * n * sum_f area(f) O(f) / sum_f area(f), n the number of faces -- n times the ring measure R(v)^2
 * as the loop's exit reads it under the key, so the smoother minimises w AMIPS^3 / n + R(v)^2 up
 * to the constant n. area(f) = |(q1 - x) x (q2 - x)| / 2 is a VARIABLE, differentiated, not a
 * weight frozen at the start of the solve as the 2026-09-25 A_f / A_mean weights were: with
 * frozen weights a vertex sliding within a flat front still moves its samples toward lower error
 * at no cost, which is the slide this removes. Measured (one-slot model, stage 9, the pressed
 * front above and below the plate, where d grows along the front toward its edge): sliding 0.02
 * outward lowered the per-face-mean sum at 514 of 517 failing vertices (median -5.7%), the
 * area-weighted sum at 225 (median +0.3%). Exactly: an affine field is integrated exactly by the
 * stencil, so the force from the front's distance to the level set vanishes; the rest is the
 * stencil's quadrature error on r^2's quadratic part.
 *
 * The derivatives are exact, and so is the Hessian by default: per stencil point
 * 2 a_i^2 (dr dr^T + r hess Phi / c), whose second term is indefinite where r < 0 (inside the
 * level set). `gauss_newton` drops that term, leaving the sum of a_i^2 dr dr^T outer products,
 * PSD by construction -- the form used until 2026-09-28; hessian() says why the default changed.
 */
class StencilEnergy3D : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;

    /// One stencil point's barycentric weights. `a` is the moving vertex's, so dq_i/dx = a_i I.
    struct Sample
    {
        double a, b, c;
        /// The point's quadrature weight in its face's mean (1 = equal weights; see
        /// TopoOffsetTetMesh::for_each_face_sample(), EXPERIMENTAL_quadratic_stencil).
        double w = 1.;
    };
    /// One incident offset face, the moving vertex implicit.
    struct Face
    {
        Eigen::Vector3d q1, q2;
        std::vector<Sample> samples;
    };

    StencilEnergy3D(
        const std::shared_ptr<const OffsetPotential3D>& potential,
        std::vector<Face> faces,
        double weight,
        bool gauss_newton = false);

    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, MatrixXd& hessian) override;
    void solution_changed(const TVector& new_x) override {}

    /**
     * @brief EXPERIMENTAL_visible_distance: read the field at a sample from the mesh instead of
     * from the potential. Called with the face's index, the sample's barycentric weights (a the
     * moving vertex's), the moving vertex's iterate x and the sample point p; fills the
     * potential's value v, gradient g and
     * Hessian H at p. Returns 1 for a reading, 0 for an unmeasurable sample (dropped, as a
     * non-finite Phi is), -1 when x itself cannot be scored (the energy is +inf there, so the
     * line search refuses it).
     */
    using SampleReader = std::function<int(
        size_t face,
        const Sample& sample,
        const Eigen::Vector3d& x,
        const Eigen::Vector3d& p,
        double& v,
        Eigen::Vector3d& g,
        Eigen::Matrix3d& H)>;
    void set_sample_reader(SampleReader r) { m_reader = std::move(r); }

    /// See AREA WEIGHTING in the class comment.
    void set_area_weighted(bool on) { m_area_weighted = on; }
    /// EXPERIMENTAL_integral_energy: the term is weight * sum_f area(f) * mean_f(r^2) -- the
    /// discrete surface integral of e^2 over the vertex's faces, areas at x and differentiated --
    /// with no division by the ring's area (AREA WEIGHTING divides; this does not).
    void set_area_integral(bool on) { m_area_integral = on; }

private:
    /// One stencil point's r = (Phi - c)/c and dr = grad Phi / c. A sample whose Phi is not
    /// finite is dropped everywhere, and one whose gradient is not finite from the gradient and
    /// the Hessian, exactly as the criterion drops it.
    struct Reading
    {
        double r = 0.;
        Eigen::Vector3d dr = Eigen::Vector3d::Zero();
        bool r_ok = false;
        bool dr_ok = false;
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero(); ///< hess Phi, from the sample reader only
    };

    /// Every stencil point's reading at x, faces in order and each face's samples in order,
    /// computed once per x: polysolve asks value, gradient and Hessian at the same x in one
    /// Newton iteration (and value once more in its gradient check), and the line search's
    /// accepted point is the next iteration's x. The field is read without its gradient until
    /// a gradient or Hessian is asked for, then with it in one value_gradient() call.
    const std::vector<Reading>& readings_at(const Eigen::Vector3d& x, bool need_dr) const;

    std::shared_ptr<const OffsetPotential3D> m_potential;
    std::vector<Face> m_faces;
    double m_weight;
    bool m_gauss_newton;
    double m_c = 1.; ///< the potential's target level, cached

    mutable std::vector<Reading> m_readings;
    mutable Eigen::Vector3d m_readings_x;
    mutable bool m_readings_valid = false;
    mutable bool m_readings_have_dr = false;
    SampleReader m_reader;
    mutable bool m_readings_unscorable = false; ///< the reader refused x (energy +inf)
    bool m_area_weighted = false;
    bool m_area_integral = false;

    /// Under area weighting: per face, its O(f) / weight (the mean of r^2 over its readings), the
    /// gradient and Hessian of that mean in x (need >= 1: gradient, >= 2: Hessian), and the face's
    /// area with its gradient and Hessian; then the ring's n * sum area O / sum area. Faces with
    /// no reading are left out of both sums, as the plain sum leaves them out.
    void area_weighted(
        const Eigen::Vector3d& x,
        int need,
        double& E,
        Eigen::Vector3d& g,
        Eigen::Matrix3d& H) const;
};

/**
 * @brief The 2D twin of StencilEnergy3D: THE offset term of a front vertex's smoothing
 * objective, the mean squared relative error of the field over the stencil of each incident
 * front chord.
 *
 *     E(x) = w * sum over the vertex's incident front chords e of
 *                (1/N_s) * sum over e's N_s stencil points i of  r(q_i(x))^2
 *
 *     r(p) = (Phi(p) - c) / c,      q_i(x) = a_i x + b_i q1
 *
 * with (a_i, b_i) the barycentric weights of stencil point i on the chord, a_i the MOVING vertex's
 * own weight, and q1 the chord's other end, fixed for the visit. The stencil is
 * TopoOffsetTriMesh::for_each_edge_sample, sized by stencil_order. Everything else -- the units
 * (w = 1 / front_conv_frac()^2 makes each chord's term TopoOffsetTriMesh::edge_offset_term(), the
 * term the per-cell energy tri_energy() adds to the chord's band face), the per-chord mean summed
 * over chords with no length weight, the exact Hessian by default and `gauss_newton` -- is
 * StencilEnergy3D's, one dimension down; see there.
 */
class StencilEnergy2D : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;

    /// One stencil point's barycentric weights. `a` is the moving vertex's, so dq_i/dx = a_i I.
    struct Sample
    {
        double a, b;
    };
    /// One incident front chord, the moving vertex implicit.
    struct Edge
    {
        Eigen::Vector2d q1;
        std::vector<Sample> samples;
    };

    StencilEnergy2D(
        const std::shared_ptr<const OffsetPotential2D>& potential,
        std::vector<Edge> edges,
        double weight,
        bool gauss_newton = false);

    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, MatrixXd& hessian) override;
    void solution_changed(const TVector& new_x) override {}

private:
    /// As StencilEnergy3D::Reading.
    struct Reading
    {
        double r = 0.;
        Eigen::Vector2d dr = Eigen::Vector2d::Zero();
        bool r_ok = false;
        bool dr_ok = false;
    };

    /// As StencilEnergy3D::readings_at().
    const std::vector<Reading>& readings_at(const Eigen::Vector2d& x, bool need_dr) const;

    std::shared_ptr<const OffsetPotential2D> m_potential;
    std::vector<Edge> m_edges;
    double m_weight;
    bool m_gauss_newton;
    double m_c = 1.; ///< the potential's target level, cached

    mutable std::vector<Reading> m_readings;
    mutable Eigen::Vector2d m_readings_x;
    mutable bool m_readings_valid = false;
    mutable bool m_readings_have_dr = false;
};

/**
 * @brief AMIPS against a rest shape, for deform_others: the smoothing term of a deformable
 * region's faces.
 *
 *     E(x) = w * sum over cells of tr(F^T F) / det F,   F = A(x) * Rinv
 *
 * A(x) = [q1 - x, q2 - x] is the cell's current Jacobian with the moving vertex first (the shared
 * smoother's convention), Rinv the inverse of the cell's rest Jacobian, captured when the face last
 * changed topologically. det F <= 0 is invalid: value NaN and is_step_valid false, so the line
 * search cannot cross an inversion. This follows polyfem's AMIPSEnergy rest-pose convention
 * (assembler/AMIPSEnergy.hpp, use_rest_pose_ true: identity reference, power 1 in 2D); the
 * equilateral quality AMIPS is the special case where R is the unit equilateral triangle. Do not
 * switch to polyfem's non-rest branch: dividing by det^2 in 2D is not scale-invariant and
 * disagrees with TriWild's kernel.
 *
 * F is affine in x (dF/dx_k = -e_k * (row-sum of Rinv)), so gradient and Hessian in x are the
 * exact chain through closed-form d/dF of e/d: no Gauss-Newton truncation needed.
 */
class RestAMIPSEnergy2D : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;
    struct Cell
    {
        Eigen::Vector2d q1, q2; ///< the fixed endpoints, current positions
        Eigen::Matrix2d rest_inv; ///< inverse rest Jacobian [r1-r0, r2-r0]^-1, same corner order
    };
    RestAMIPSEnergy2D(std::vector<Cell> cells, double weight);

    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, MatrixXd& hessian) override;
    void solution_changed(const TVector& new_x) override {}
    bool is_step_valid(const TVector& x0, const TVector& x1) override;

private:
    /// e = tr(F^T F), d = det F at x for one cell; false when d <= 0.
    bool cell_F(const Eigen::Vector2d& x, const Cell& c, Eigen::Matrix2d& F, double& d) const;
    std::vector<Cell> m_cells;
    double m_weight;
};

/**
 * @brief The 3D twin of RestAMIPSEnergy2D: AMIPS of a tet against its rest shape.
 *
 *     E(x) = w * sum over cells of tr(F^T F) / det(F)^(2/3),   F = A(x) * Rinv
 *
 * A(x) = [q1 - x, q2 - x, q3 - x] with the moving vertex first (the shared smoother's
 * convention), Rinv the inverse rest Jacobian in the same corner order. det^(2/3) is what makes
 * the 3D form scale-invariant, as det^1 does in 2D; the minimum is 3 at F = I, the same scale as
 * the shared AMIPSEnergy3D against the regular tet, so the two terms sum 1:1. det F <= 0 is
 * invalid: value NaN and is_step_valid false. F is affine in x, so gradient and Hessian are the
 * exact chain through the closed-form derivatives of e / d^(2/3) in F.
 */
class RestAMIPSEnergy3D : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;
    struct Cell
    {
        Eigen::Vector3d q1, q2, q3; ///< the fixed corners, current positions
        Eigen::Matrix3d rest_inv; ///< inverse rest Jacobian [r1-r0, r2-r0, r3-r0]^-1
    };
    RestAMIPSEnergy3D(std::vector<Cell> cells, double weight);

    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, MatrixXd& hessian) override;
    void solution_changed(const TVector& new_x) override {}
    bool is_step_valid(const TVector& x0, const TVector& x1) override;

private:
    /// F and d = det F at x for one cell; false when d <= 0.
    bool cell_F(const Eigen::Vector3d& x, const Cell& c, Eigen::Matrix3d& F, double& d) const;
    std::vector<Cell> m_cells;
    double m_weight;
};

/**
 * @brief The per-tet energy's AMIPS part as the smoother minimises it: w * sum over cells of
 * AMIPS^3.
 *
 * tet_energy() charges every cell w * AMIPS^3, so the smoother minimises that same power at that
 * same weight; the engine's AMIPSEnergy3D is AMIPS to the first power. Cells in the shared
 * smoother's convention: 12 doubles with the moving vertex first, its three entries replaced by
 * x. Derivatives by the chain rule from the engine's first-power ones:
 * grad A^3 = 3 A^2 grad A, hess A^3 = 3 A^2 hess A + 6 A grad A grad A^T. A step that inverts a
 * cell is invalid, as for the engine's AMIPSEnergy3D.
 */
class CubedAMIPSEnergy3D : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;
    CubedAMIPSEnergy3D(std::vector<std::array<double, 12>> cells, double weight);
    /// EXPERIMENTAL_integral_energy: each cell's AMIPS^3 times its volume (the discrete volume
    /// integral of AMIPS^3), as (sqrt2/12) T^(3/2) AMIPS^(3/2) with T = (1/2) sum |e|^2 -- the
    /// volume never from a determinant (see cubed_amips_vol_term() in the .cpp).
    void set_volume_weighted(bool on) { m_volume_weighted = on; }

    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, MatrixXd& hessian) override;
    void solution_changed(const TVector& new_x) override {}
    bool is_step_valid(const TVector& x0, const TVector& x1) override;

private:
    std::vector<std::array<double, 12>> m_cells;
    double m_weight;
    bool m_volume_weighted = false;
};

/**
 * @brief EXPERIMENTAL_band_volume_energy: a front vertex's part of the band-volume offset term,
 *
 *     E(x) = w * sum over the vertex's band cells t of  Vol_t(x) * m_t(x),
 *     m_t(x) = (1/n_t) * sum over t's four corners and its centroid of  r(q_i(x)),
 *
 * r = (Phi - c)/c the relative error (for the euclidean field (d - delta)/delta), Vol_t the cell's
 * signed volume, positive on a valid cell, and n_t the number of those five points where Phi is
 * finite (a point where it is not is left out, as StencilEnergy3D leaves it out). The caller
 * passes w = 1 / front_conv_frac(), which makes w Vol r = Vol (d - delta) / front_conv: each cell's
 * term is TopoOffsetTetMesh::band_cell_term(), and E the vertex's part of int_B e dV.
 *
 * The rule per cell is exact wherever d is affine on the cell (one input face nearest): the mean
 * of an affine function over a tetrahedron is its mean over the four corners, and its value at
 * the centroid. Where a kink of d (two faces facing each other across a gap, a concave edge) or
 * its curvature near a convex edge crosses the cell, the error is of order the cell's size
 * relative to the cell's term; such cells lie along a surface, so the band's total error still
 * vanishes under refinement.
 *
 * Vol_t(x) = (q1 - x) . ((q2 - q1) x (q3 - q1)) / 6 is affine in x: its gradient is
 * -(q2 - q1) x (q3 - q1) / 6 and its Hessian zero. The three other corners' r are constants of
 * the visit; the moving vertex's own r and the centroid's move with x (dq/dx = I and I/4). So
 *     grad E_t = m_t grad Vol + Vol grad m_t,
 *     hess E_t = grad Vol grad m_t^T + grad m_t grad Vol^T + Vol hess m_t,
 * grad m_t = (dr(x) + dr(c)/4) / n_t, hess m_t = (hess r(x) + hess r(c)/16) / n_t, exact (hess r
 * = hess Phi / c, the field's own). The outer-product pair is indefinite in general;
 * polysolve's Newton regularises, and the AMIPS term beside it carries the shape.
 */
/**
 * @brief EXPERIMENTAL_band_volume_rule "corner_bound": the input's triangles as convex primitives.
 *
 * The distance d to the input is the min over its triangles i of d_i, the distance to triangle i
 * alone. Each d_i is convex (the distance to a convex set), which the corner-bound rule needs
 * (TopoOffsetTetMesh::band_cell_term()). nearest() is the triangle nearest to a point (the BVH's
 * nearest facet); distance() is d_i with its gradient (p - foot)/d_i and the Hessian of the
 * distance to the feature the foot lies on: 0 inside the triangle, (I - u u^T - e e^T)/d_i on an
 * edge of direction e, (I - u u^T)/d_i at a vertex (u the unit gradient). At d_i = 0 the gradient
 * and Hessian are reported as 0.
 */
class InputTriangles
{
public:
    InputTriangles(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
    int64_t nearest(const Eigen::Vector3d& p) const;
    double distance(
        int64_t tri,
        const Eigen::Vector3d& p,
        Eigen::Vector3d* grad = nullptr,
        Eigen::Matrix3d* hess = nullptr) const;
    /// EXPERIMENTAL_edge_excess_refinement: (d_P(a) + d_P(b))/2 - d(m), m = (a + b)/2, d the
    /// distance to the nearest triangle -- how far the corner bound's linear interpolant of d_P
    /// lies above the true distance at the edge's midpoint. Never negative: d(m) <= d_P(m) <=
    /// (d_P(a) + d_P(b))/2, d_P being convex. 0 where d_P is affine along the edge and P is
    /// nearest at m; the chord sag of d_P where it curves; more where another triangle is nearer.
    double edge_excess(int64_t P, const Eigen::Vector3d& a, const Eigen::Vector3d& b) const
    {
        const Eigen::Vector3d m = 0.5 * (a + b);
        return 0.5 * (distance(P, a) + distance(P, b)) - distance(nearest(m), m);
    }
    size_t size() const { return size_t(m_F.rows()); }
    /// EXPERIMENTAL_settle_mark "rms": the mean over triangle abc of (d - delta)^2 (n . grad d),
    /// n the face's unit normal (pointing out of the band), by the 3-edge-midpoint rule (weights
    /// 1/3, exact for quadratic integrands): exact over a face whose points all have one input
    /// face's interior as nearest feature (d affine, grad d constant there).
    double face_mean_sq_cos(
        const Eigen::Vector3d& a,
        const Eigen::Vector3d& b,
        const Eigen::Vector3d& c,
        const Eigen::Vector3d& n,
        double delta) const
    {
        double q = 0.;
        for (const auto& [u, v] : {std::pair{&a, &b}, std::pair{&b, &c}, std::pair{&c, &a}}) {
            const Eigen::Vector3d m = 0.5 * (*u + *v);
            Eigen::Vector3d g;
            const double dm = distance(nearest(m), m, &g);
            q += (dm - delta) * (dm - delta) * n.dot(g) / 3.;
        }
        return q;
    }
    /// EXPERIMENTAL_band_volume_exact_min: min over ALL triangles P of (1/n) sum_i d_P(p_i), n
    /// points (the cell's corners), and the minimiser in `best` (the lowest index on a tie). An
    /// exact search of an AABB tree over the triangles: a node is skipped when (1/n) sum_i
    /// dist(p_i, its box), a lower bound on every triangle inside it, exceeds the best so far.
    /// Starts from `hint` when >= 0 (else from the points' nearest triangles), which changes only
    /// the speed. Measured on the last frames of the cube (24 triangles) and 100026 (1736): over
    /// 2000 sampled band cells with a front vertex each, no triangle outside the corners' nearest
    /// (ties included) had a smaller mean, so the search visits boxes only.
    double min_mean_distance(const Eigen::Vector3d* p, int n, int64_t& best, int64_t hint = -1)
        const;
    /// EXPERIMENTAL_band_volume_exact_min: a triangle and its distances to three fixed points.
    struct BallCandidate
    {
        int64_t tri = -1;
        double d1 = 0., d2 = 0., d3 = 0.;
    };
    /// EXPERIMENTAL_band_volume_exact_min: every triangle that is min_mean_distance()'s minimiser
    /// of the four points {x, q[0], q[1], q[2]} (x first), or tied with it, for some x with
    /// (x - c).norm() <= r; ascending index, each with its distances to q. So the minimum over
    /// `out` of that mean, summed in min_mean_distance()'s order and scanned with its comparison,
    /// is min_mean_distance() bit for bit at every such x (proof in the .cpp).
    void ball_candidates(
        const Eigen::Vector3d& c,
        double r,
        const std::array<Eigen::Vector3d, 3>& q,
        std::vector<BallCandidate>& out) const;
    /// EXPERIMENTAL_settle_refinement: the distance between triangle P and the triangle abc. 0
    /// when an edge of one crosses the other, its endpoints strictly on the two sides of the
    /// other's plane (exact orientation signs); otherwise the min of the 6 corner-to-other-triangle
    /// distances (distance()'s closest point) and the 9 edge-edge distances (Ericson, Real-Time
    /// Collision Detection 5.1.9): two disjoint triangles' closest pair has a corner of one or a
    /// point on an edge of each, and two that meet without such a crossing (a corner on the
    /// other's plane, or coplanar) have one of those 15 at 0.
    double triangle_distance(
        int64_t P,
        const Eigen::Vector3d& a,
        const Eigen::Vector3d& b,
        const Eigen::Vector3d& c) const;
    /// EXPERIMENTAL_settle_refinement: min over ALL triangles P of triangle_distance(P, a, b, c),
    /// i.e. min over the triangle abc of d. An exact search of min_mean_distance()'s tree: it
    /// starts from min over the corners of d (an upper bound on the min over the triangle) and
    /// skips a node whose box is farther from the box of abc than the best so far.
    double min_triangle_distance(
        const Eigen::Vector3d& a,
        const Eigen::Vector3d& b,
        const Eigen::Vector3d& c) const;
    /// EXPERIMENTAL_settle_refinement: min over ALL triangles P of max_i d_P(p_i), n points. With
    /// p the corners of a triangle, an upper bound on max over the triangle of d: d_P is convex,
    /// so its max over the triangle is at a corner, and d <= d_P. The same tree search: it starts
    /// from the points' nearest triangles and skips a node when max_i dist(p_i, its box), a lower
    /// bound on every triangle inside it, exceeds the best so far.
    double min_max_corner_distance(const Eigen::Vector3d* p, int n) const;

private:
    Eigen::MatrixXd m_V;
    Eigen::MatrixXi m_F;
    SimpleBVH::BVH m_bvh;
    /// min_mean_distance()'s tree: a node's box holds triangles m_order[begin, end); a leaf has
    /// left < 0. Boxes padded by 1e-9 of the input's diagonal, so a box distance never exceeds a
    /// triangle distance by rounding.
    struct Node
    {
        Eigen::Vector3d lo, hi;
        int left = -1, right = -1, begin = 0, end = 0;
    };
    std::vector<Node> m_nodes;
    std::vector<int> m_order;
    double m_pad = 0.; ///< the boxes' padding
    int build_node(int begin, int end, const std::vector<Eigen::Vector3d>& centroid, double pad);
};

class BandVolumeEnergy3D : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;
    /// One band cell of the ring: its three other corners, at their current positions.
    struct Cell
    {
        Eigen::Vector3d q1, q2, q3;
    };
    /// x0: the moving vertex's current position. Each cell's corners are ordered here so that
    /// its volume at x0 is positive (the ring the smoother starts from is valid).
    BandVolumeEnergy3D(
        const std::shared_ptr<const OffsetPotential3D>& potential,
        std::vector<Cell> cells,
        const Eigen::Vector3d& x0,
        double weight);

    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, MatrixXd& hessian) override;
    void solution_changed(const TVector& new_x) override {}
    /// EXPERIMENTAL_band_volume_rule "centroid": m_t is r at the centroid alone (grad m_t = dr(c)/4,
    /// hess m_t = hess r(c)/16), the corners left out.
    void set_centroid_only(bool on) { m_centroid_only = on; }
    /**
     * @brief EXPERIMENTAL_band_volume_rule "corner_bound": m_t becomes
     *     min over the cell's candidate triangles P of (1/4) sum over its 4 corners of r_P(q),
     *     r_P = (d_P - delta)/delta,
     * an upper bound on the cell's mean of r for every P (d <= d_P, and the convex d_P lies below
     * its linear interpolant). `candidates[k]` are cell k's triangles (in the order of the cells
     * given to the constructor); the three fixed corners' terms are computed here, once. The
     * gradient and Hessian are the active (minimising) triangle's: grad m_t = grad r_P(x)/4,
     * hess m_t = hess r_P(x)/4.
     */
    void set_corner_bound(
        std::shared_ptr<const InputTriangles> tris,
        std::vector<std::vector<int64_t>> candidates,
        double delta);
    /// EXPERIMENTAL_band_volume_exact_min: as set_corner_bound(), with m_t the min over ALL
    /// triangles (InputTriangles::min_mean_distance()) at every evaluation, so the solve descends
    /// exactly the cell terms E sums. Gradient and Hessian: the minimiser's, as set_corner_bound().
    /// Every value, gradient and Hessian equals, bit for bit, that of a min_mean_distance() call
    /// per cell and evaluation; the work behind them is cut twice (exact_minima()):
    ///  - the last point's minima are kept, so value, gradient and Hessian at one x (polysolve
    ///    asks for each separately, and again in its line search) search once;
    ///  - each cell searches only its certified candidates (InputTriangles::ball_candidates()),
    ///    valid at every x in a ball around the point they were built at. r, the ball's radius,
    ///    is the distance from that point to the nearest plane through a cell's three fixed
    ///    corners: the largest ball around it in which none of these cells can invert. The
    ///    solver evaluates only points where no ring cell is inverted (polysolve tests
    ///    is_step_valid() before each trial value), so it leaves this ball only away from the
    ///    nearest face. Correctness does not depend on r, only the speed. Outside the ball the
    ///    candidates are rebuilt around the new point; where no ball can be certified there
    ///    (r <= 0 or not finite), that evaluation searches the whole tree.
    /// Measured on the cube (cube_on, 3 turns, serial): 7.45M evaluations, 57% at the last point;
    /// of the rest 96% inside the ball, with 1.37 candidates per cell; the whole tree 8 times.
    /// Smoothing 68 s -> 15 s, the run 76 s -> 23 s, the output bit-identical.
    void set_exact_corner_bound(std::shared_ptr<const InputTriangles> tris, double delta);
    /// EXPERIMENTAL_band_volume_exact_min: the current certified ball, its centre and radius
    /// (radius < 0: none yet). For the tests.
    std::pair<Eigen::Vector3d, double> certified_ball() const { return {m_ball_c, m_ball_r}; }

private:
    /// need: 0 value, 1 + gradient, 2 + Hessian.
    double eval(const Eigen::Vector3d& x, int need, Eigen::Vector3d& g, Eigen::Matrix3d& H) const;
    /// EXPERIMENTAL_band_volume_exact_min: m_min_md and m_min_tri at x (see
    /// set_exact_corner_bound()).
    void exact_minima(const Eigen::Vector3d& x) const;
    /// EXPERIMENTAL_band_volume_exact_min: centre the certified ball at x, when its radius there
    /// is positive and finite; false (the old ball kept) otherwise.
    bool certify_ball(const Eigen::Vector3d& x) const;
    bool m_centroid_only = false;
    std::shared_ptr<const InputTriangles> m_tris; ///< corner_bound when set
    std::vector<std::vector<int64_t>> m_cand;
    std::vector<std::vector<double>>
        m_cand_fixed; ///< per cell, per candidate: sum of r_P at q1..q3
    double m_delta = 1.;
    bool m_exact = false; ///< set_exact_corner_bound()
    mutable std::vector<int64_t> m_hint; ///< per cell: the last minimiser (search start only)
    /// exact_minima(): per cell min_mean_distance() and its minimiser at m_min_x (bitwise), the
    /// last point evaluated; valid once m_min_set.
    mutable bool m_min_set = false;
    mutable Eigen::Vector3d m_min_x = Eigen::Vector3d::Zero();
    mutable std::vector<double> m_min_md;
    mutable std::vector<int64_t> m_min_tri;
    /// certify_ball(): the ball's centre and radius (< 0: none), and per cell its candidates.
    mutable Eigen::Vector3d m_ball_c = Eigen::Vector3d::Zero();
    mutable double m_ball_r = -1.;
    mutable std::vector<std::vector<InputTriangles::BallCandidate>> m_ball_cand;

    std::shared_ptr<const OffsetPotential3D> m_potential;
    std::vector<Cell> m_cells;
    std::vector<double> m_fixed_sum; ///< per cell: the sum of r over q1, q2, q3 where finite
    std::vector<int> m_fixed_n; ///< per cell: how many of q1, q2, q3 are finite
    double m_weight;
    double m_c = 1.;
};

} // namespace wmtk::components::topological_offset
