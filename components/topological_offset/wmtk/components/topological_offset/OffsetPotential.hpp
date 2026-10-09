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
#include <type_traits>
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
 * @brief A point's squared residual to the level set: w * (Phi(x) - c)^2.
 *
 * A diagnostic since the 2026-10-09 cleanup -- no smoothing objective reads it (the smoother
 * minimises E_V, see TopoOffsetTetMesh::vertex_energy()): the placement-gradient split
 * (gradient_split()) and the front profile (log_front_profile()) evaluate it.
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
    /// root either way; only the charge changes.
    ///
    /// one_sided charges only points on the input's side of the level set
    /// (is_inside_offset()): beyond it the term and its derivatives are zero, so the term pushes a
    /// vertex out to the level set and never pulls one in. C^1 at the level set, where the
    /// residual is zero from both sides.
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
 * @brief The input complex as convex primitives, for D(t) (see VolAMIPSEnergy, BandVolumeEnergy):
 * its triangles in 3D (InputTriangles), its segments in 2D (InputSegments).
 *
 * The distance d to the input is the min over its primitives i of d_i, the distance to primitive
 * i alone, and each d_i is convex (the distance to a convex set) -- which is what makes the corner
 * mean of d_i an upper bound of d's mean over a simplex. nearest() is the primitive nearest a
 * point (lowest index on a tie); distance() is d_i with its gradient (p - foot)/d_i and the Hessian
 * of the distance to the feature the foot lies on: 0 in a triangle's interior (3D) or a segment's
 * interior (2D), (I - u u^T - e e^T)/d_i on a triangle edge of direction e, (I - u u^T)/d_i at a
 * vertex (u the unit gradient). At d_i = 0 the gradient and Hessian are reported as 0.
 */
class InputTriangles
{
public:
    InputTriangles(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
    /**
     * EXPERIMENTAL_convex_pieces: the primitives are convex pieces of the input SOLID instead of
     * its surface triangles. The pieces are built by merging the solid's tets (V: positions, T:
     * #T x 4) greedily across shared faces while the union stays convex. For a point outside the
     * solid, the distance to a piece K inside it is convex and never below d, so the corner bound
     * stays an upper bound, and one piece covers corners on several faces of the solid (a convex
     * edge, two triangles of one face). distance() is 0 inside a piece.
     */
    static std::shared_ptr<InputTriangles> convex_pieces(
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXi& T);
    int64_t nearest(const Eigen::Vector3d& p) const;
    /// The primitives nearest p: nearest() alone for triangles; for pieces, every piece within
    /// the nearest distance (a point on the solid's surface touches several pieces at d = 0).
    void nearest_all(const Eigen::Vector3d& p, std::vector<int64_t>& out) const;
    double distance(
        int64_t tri,
        const Eigen::Vector3d& p,
        Eigen::Vector3d* grad = nullptr,
        Eigen::Matrix3d* hess = nullptr) const;
    size_t size() const { return m_pieces ? m_piece_tris.size() : size_t(m_F.rows()); }
    bool is_pieces() const { return m_pieces; }
    /// Pieces mode: the number of tets merged into each piece.
    const std::vector<size_t>& piece_tet_counts() const { return m_piece_tet_counts; }

private:
    InputTriangles() = default;
    /// The candidates within the nearest distance of p, as (triangle row, distance) pairs.
    void near_triangles(const Eigen::Vector3d& p, std::vector<std::pair<int64_t, double>>& out)
        const;
    Eigen::MatrixXd m_V;
    Eigen::MatrixXi m_F;
    SimpleBVH::BVH m_bvh;
    // Pieces mode: m_F holds every piece's boundary triangles, row r belongs to m_tri_piece[r].
    bool m_pieces = false;
    std::vector<int64_t> m_tri_piece;
    std::vector<std::vector<int64_t>> m_piece_tris;
    std::vector<std::vector<Eigen::Vector4d>> m_piece_planes; ///< outward n and c: inside n.x <= c
    std::vector<size_t> m_piece_tet_counts;
    double m_tol = 0.;
};

class InputSegments
{
public:
    /// V: #V x 2 (or x 3, the third column ignored), E: #E x 2.
    InputSegments(const Eigen::MatrixXd& V, const Eigen::MatrixXi& E);
    int64_t nearest(const Eigen::Vector2d& p) const;
    double distance(
        int64_t seg,
        const Eigen::Vector2d& p,
        Eigen::Vector2d* grad = nullptr,
        Eigen::Matrix2d* hess = nullptr) const;
    size_t size() const { return size_t(m_E.rows()); }

private:
    Eigen::MatrixXd m_V; ///< #V x 3, z = 0
    Eigen::MatrixXi m_E;
    SimpleBVH::BVH m_bvh;
};

/// The primitive type D(t) reads in each dimension.
template <int DIM>
using InputPrimitives = std::conditional_t<DIM == 3, InputTriangles, InputSegments>;

/**
 * @brief THE AMIPS PART OF THE PER-CELL ENERGY E_T at one moving vertex x (see the energy spec):
 *
 *     E(x) = weight * sum over the vertex's cells t of  V_t(x) A_t(x)^DIM
 *
 * V_t the cell's volume (3D) or area (2D), A_t its AMIPS against a reference R_t -- the cell's
 * stamped rest shape when it is plastic, the regular simplex when it is elastic. With E_t the
 * edge matrix (columns q_k - x, the moving vertex first, positively oriented), F = E_t R_t^-1,
 * f = |F|_F^2 and g = det E_t:
 *
 *     A = f / J^(2/DIM),  J = det F = g / det R,  V = g / DIM!,  so  V A^DIM = c f^DIM / g,
 *     c = det(R)^2 / DIM!
 *
 * -- one closed form for both dimensions and both references, no determinant of F and no
 * volume taken apart from the AMIPS, so a nearly flat cell reads huge rather than 0 x huge. g is
 * affine in x (a rank-one update of the edge matrix), so grad g is constant and hess g = 0; f is
 * quadratic, grad f = -2 F s with s = R^-T 1 and hess f = 2 |s|^2 I. The derivatives are exact:
 *
 *     grad = c (k f^(k-1) grad f / g - f^k grad g / g^2)
 *     hess = c (k(k-1) f^(k-2) grad f grad f^T / g + k f^(k-1) hess f / g
 *               - k f^(k-1) (grad f grad g^T + grad g grad f^T) / g^2
 *               + 2 f^k grad g grad g^T / g^3)
 *
 * with k = DIM. A cell with g <= 0 makes the energy +inf (the line search refuses the point).
 * value_of() is the same quantity for one cell from its corners, which tet_energy() /
 * tri_energy() read, so the guards and the smoother compare one number.
 */
template <int DIM>
class VolAMIPSEnergy : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;
    using VecD = Eigen::Matrix<double, DIM, 1>;
    using MatD = Eigen::Matrix<double, DIM, DIM>;
    /// One cell at the moving vertex: its other corners in the oriented order after it, and its
    /// reference (cell()).
    struct Cell
    {
        std::array<VecD, DIM> q;
        MatD rest_inv;
        VecD s; ///< rest_inv^T 1
        double c; ///< det(R)^2 / DIM!
    };
    /// A cell with reference edge matrix R (columns r_k - r_0, the moving vertex's rest first);
    /// false when R is not positively oriented.
    static bool cell(const std::array<VecD, DIM>& q, const MatD& R, Cell& out);
    /// The regular simplex's edge matrix (unit edges): the elastic reference.
    static MatD regular_rest();
    /// V A^DIM of the simplex p[0..DIM] against reference R (regular_rest() for elastic):
    /// +inf when p is not positively oriented, or when R is not.
    static double value_of(const std::array<VecD, DIM + 1>& p, const MatD& R);

    VolAMIPSEnergy(std::vector<Cell> cells, double weight);

    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, MatrixXd& hessian) override;
    void solution_changed(const TVector& new_x) override {}

private:
    double eval(const VecD& x, int need, VecD& g, MatD& H) const;
    std::vector<Cell> m_cells;
    double m_weight;
};
using VolAMIPSEnergy2D = VolAMIPSEnergy<2>;
using VolAMIPSEnergy3D = VolAMIPSEnergy<3>;

/**
 * @brief THE BAND PART OF THE PER-CELL ENERGY E_T at one moving vertex x:
 *
 *     E(x) = weight * sum over the vertex's band cells t of  V_t(x) D_t(x),
 *     D_t(x) = min over P in C_t of (1/(DIM+1)) sum over t's corners q of (d_P(q) - delta)/delta
 *
 * d_P the distance to input primitive P alone (InputPrimitives), C_t the cell's candidates (the
 * nearest primitive of each corner and the one the cell inherited at the last split pass). The
 * other corners' terms are constants of the visit; the moving corner's moves with x. V_t(x) =
 * det(E_t)/DIM! is affine in x. Gradient and Hessian are the active (minimising) candidate's:
 *     grad = D grad V + V grad d_P(x) / ((DIM+1) delta),
 *     hess = grad V grad m^T + grad m grad V^T + V hess d_P(x) / ((DIM+1) delta).
 */
template <int DIM>
class BandVolumeEnergy : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;
    using VecD = Eigen::Matrix<double, DIM, 1>;
    using MatD = Eigen::Matrix<double, DIM, DIM>;
    /// One band cell at the moving vertex: its other corners in the oriented order after it, and
    /// its candidate primitives.
    struct Cell
    {
        std::array<VecD, DIM> q;
        std::vector<int64_t> candidates;
    };
    BandVolumeEnergy(
        std::shared_ptr<const InputPrimitives<DIM>> prims,
        std::vector<Cell> cells,
        double delta,
        double weight);

    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, MatrixXd& hessian) override;
    void solution_changed(const TVector& new_x) override {}

private:
    double eval(const VecD& x, int need, VecD& g, MatD& H) const;
    std::shared_ptr<const InputPrimitives<DIM>> m_prims;
    std::vector<Cell> m_cells;
    std::vector<std::vector<double>>
        m_fixed; ///< per cell, per candidate: sum over the other corners
    double m_delta;
    double m_weight;
};
using BandVolumeEnergy2D = BandVolumeEnergy<2>;
using BandVolumeEnergy3D = BandVolumeEnergy<3>;

} // namespace wmtk::components::topological_offset
