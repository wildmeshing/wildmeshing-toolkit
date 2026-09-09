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
     * See TopoOffsetTetMesh::offset_gradient_tolerance(), which is the only consumer.
     */
    double level_set_slope() const { return m_grad_ref; }

    virtual double value(const VecD& p) const = 0;
    /// The Euclidean field: value() is the distance, so a distance residual is the plain one.
    virtual bool is_euclidean() const { return false; }
    virtual VecD gradient(const VecD& p) const = 0;
    virtual MatD hessian(const VecD& p) const = 0;

    /**
     * @brief Distance from `p` to the level set, in length units.
     *
     * What the convergence criterion measures: comparable to target_distance, while the field
     * value need not be.
     */
    virtual double residual_length(const VecD& p) const = 0;

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
 * What Phi is: the offset geometric contact potential of ipc-toolkit's `high_order_contact`
 * subtree, evaluated at a point q against the input complex,
 *
 *     Phi(q) = sum over active primitives P of  b( dist(q, P), dhat )
 *     b(d, dhat) = -(d/dhat - 1)^2 * log(d/dhat)   for d < dhat, 0 otherwise
 *
 * (`ipc::NormalizedClampedLogBarrier`). "Active" is the OGC feasible-region rule: a triangle is
 * active at q when q projects into its interior, an edge when q projects into its interior and
 * lies outside the wedges its incident triangles claim, a vertex when q lies in its Voronoi
 * region. Away from features exactly one primitive contributes and Phi is a monotone function of
 * the Euclidean distance alone; at a reentrant feature several contribute, their barriers add,
 * and the level set bulges outward. So Phi = c is a smoothed offset, not the Euclidean one, and
 * that difference is deliberate; the Euclidean distance is still reported as a diagnostic.
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
 * Threading: an evaluation writes the query point into a scratch vertex matrix and builds a
 * collision set around it, so it holds per-thread state; `value`, `gradient` and `hessian` are
 * const and safe to call concurrently from the smoothing pass.
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
     * @brief First-order distance from `p` to the level set Phi = c, in length units.
     *
     * |Phi(p) - c| over the level-set slope, comparable to target_distance while Phi itself is not.
     * Saturates outside the support, where there is no direction to the level set; the runaway
     * guard turns that state into a hard error before this number decides anything.
     */
    double residual_length(const VecD& p) const override;

    /// Whether `p` is inside the support at all, i.e. Phi(p) > 0.
    bool within_support(const VecD& p) const override { return value(p) > 0.; }

    /// Phi decreases with distance, so the offset region is where it is still above the level.
    bool is_inside_offset(const VecD& p) const override { return value(p) >= m_c; }

    /// Diagnostic: the active pairs at `p`, one per line, with each one's contribution -- the only
    /// way to see why Phi has the value it has.
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
    OffsetEnergy(
        const std::shared_ptr<const OffsetPotential<DIM>>& potential,
        double weight = 1.,
        bool gauss_newton = true,
        bool distance_residual = false);

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
    /// The signed distance s from p to the level set along the field's normal n at p (n points
    /// toward the input; s > 0: the level set lies outward of p). Safeguarded Newton. false
    /// when no root brackets within 2 dhat, e.g. outside the support.
    bool root_distance(const VecD& p, double& s, VecD& n) const;
    /// r and its gradient under either residual (see the constructor). The last point's
    /// answer is cached: value, gradient and Hessian are asked at the same x in one iteration.
    void residual(const VecD& p, double& r, VecD& dr) const;
    mutable double m_last_root = 0.; ///< warm start for root_distance(), see there
    mutable bool m_cache_valid = false;
    mutable VecD m_cache_p;
    mutable double m_cache_r = 0.;
    mutable VecD m_cache_dr;
};

using OffsetEnergy2D = OffsetEnergy<2>;

/**
 * @brief First-order offset term: each front edge at the vertex against the field.
 *
 *     E(x) = w * sum over the vertex's incident front edges e of (1 - n_e(x) . g(m_e(x)))^2
 *
 * n_e(x) = sigma_e R90 (q_e - x) / |q_e - x| is the edge's outward unit normal (q_e the other
 * endpoint, fixed; sigma_e = +-1 chosen at construction so the normal points away from the band),
 * g(m) = s grad Phi(m) / |grad Phi(m)| the field's outward unit direction at the edge midpoint
 * m_e = (x + q_e) / 2 (s = -1 for the smooth potential, larger inside; +1 for the Euclidean
 * distance). It is the orientation criterion's quantity as an energy: zero when the edge lies
 * along the level set, 4 w per edge when it points the wrong way.
 *
 * Both dependences on x are differentiated: the edge's rotation (d n_e / dx, exact) and the
 * field's turning (d g / dx through the potential's Hessian). Do not re-add a vertex normal frozen
 * for the visit; measured worse -- see git history of this file. The term's own Hessian would need
 * the third derivative of Phi, so its block is the Gauss-Newton 2 w sum J_e J_e^T, the same choice
 * OffsetEnergy makes.
 */
class AlignEnergy2D : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;
    struct Edge
    {
        Eigen::Vector2d q; ///< the other endpoint
        double sigma; ///< +-1: sigma * R90 (q - x) points away from the band
        /// Gradient agreement at the edge's two endpoints, max(0, ghat(x) . ghat(q)), frozen for
        /// the visit: the term's target direction is only meaningful where the field's gradient is
        /// consistent along the edge. At a concave corner the gradient flips across the corner's
        /// bisector, and an edge spanning the flip must not be charged for a target it cannot meet.
        /// ~1 on smooth stretches, 0 at a right-angle flip. A frozen scalar, not a frozen
        /// direction -- the residual below keeps both dependences on x.
        double agree = 1.;
    };
    AlignEnergy2D(
        const std::shared_ptr<const OffsetPotential2D>& potential,
        std::vector<Edge> edges,
        double outward_sign,
        double weight);
    /// r_e and its derivative for one edge; r = 0 with J = 0 where grad Phi vanishes.
    void residual(const Eigen::Vector2d& x, const Edge& e, double& r, Eigen::Vector2d& J) const;
    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, MatrixXd& hessian) override;
    void solution_changed(const TVector& new_x) override {}

private:
    std::shared_ptr<const OffsetPotential2D> m_potential;
    std::vector<Edge> m_edges;
    double m_sign, m_weight;
};
using OffsetEnergy3D = OffsetEnergy<3>;

/**
 * @brief The 3D twin of AlignEnergy2D: each incident front FACE at the vertex against the field.
 *
 *     E(x) = w * sum over the vertex's incident front faces f of agree_f (1 - n_f(x) . g(c_f(x)))^2
 *
 * n_f(x) = sigma_f (q1 - x) x (q2 - x) / |...| is the face's outward unit normal (q1, q2 the two
 * fixed corners; sigma_f = +-1 chosen at construction so the normal points away from the band
 * tet), g(c) = s grad Phi(c) / |grad Phi(c)| the field's outward unit direction at the face
 * centroid c_f = (x + q1 + q2) / 3. Zero when the face lies along the level set, 4 w per face
 * when it points the wrong way. Both dependences on x are differentiated, as in 2D: the normal's
 * rotation (exact) and the field's turning (through the potential's Hessian); the Hessian block is
 * the Gauss-Newton 2 w sum J_f J_f^T.
 */
class AlignEnergy3D : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;
    struct Face
    {
        Eigen::Vector3d q1, q2; ///< the two other corners, in the band tet's orientation
        double sigma; ///< +-1: sigma * (q1 - x) x (q2 - x) points away from the band
        /// Gradient agreement across the face, min over its three corner pairs of
        /// max(0, ghat_a . ghat_b), frozen for the visit -- the 3D reading of the 2D endpoint
        /// agreement: the target direction only means something where the field's gradient is
        /// consistent over the face.
        double agree = 1.;
    };
    AlignEnergy3D(
        const std::shared_ptr<const OffsetPotential3D>& potential,
        std::vector<Face> faces,
        double outward_sign,
        double weight);
    /// r_f and its derivative for one face; r = 0 with J = 0 where grad Phi or the face vanishes.
    void residual(const Eigen::Vector3d& x, const Face& f, double& r, Eigen::Vector3d& J) const;
    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, MatrixXd& hessian) override;
    void solution_changed(const TVector& new_x) override {}

private:
    std::shared_ptr<const OffsetPotential3D> m_potential;
    std::vector<Face> m_faces;
    double m_sign, m_weight;
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

} // namespace wmtk::components::topological_offset
