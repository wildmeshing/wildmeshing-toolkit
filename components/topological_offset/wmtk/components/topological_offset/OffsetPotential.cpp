#include "OffsetPotential.hpp"

#include <wmtk/utils/Logger.hpp>

#include <SimpleBVH/BVH.hpp>

#include <ipc/candidates/candidates.hpp>
#include <ipc/collision_mesh.hpp>
#include <ipc/high_order_contact/high_order_contact_parameters.hpp>
#include <ipc/high_order_contact/quadrature_potential.hpp>

#include <cmath>
#include <limits>

namespace wmtk::components::topological_offset {

namespace {
/// Quadrature order is irrelevant on this path -- Phi is a POINT evaluation against a cached
/// collision set, with no quadrature anywhere in it -- but HighOrderContactParameters requires
/// one and logs a warning for order 1. 2 is the smallest value it accepts silently.
constexpr int UNUSED_QUAD_ORDER = 2;

/// dbar_factor, likewise unread by the 2D vertex path (it feeds the near/far barrier split,
/// which this does not use). 1.0 is upstream's default.
constexpr double UNUSED_DBAR_FACTOR = 1.0;
} // namespace


/**
 * @brief Everything that mentions ipc-toolkit.
 *
 * The collision mesh carries the complex's vertices plus ONE extra row for the query point,
 * which is the pattern the upstream API is written around ("with the point q appended to the
 * last row"). Its topology never changes, so it is built once; an evaluation only writes the
 * last row and rebuilds the small collision set around it.
 */
struct OffsetPotential::Impl
{
    ipc::CollisionMesh mesh; ///< complex vertices + the query vertex; edges = complex segments
    ipc::HighOrderContactParameters params;
    size_t n_complex_v = 0; ///< index of the query vertex, one past the complex's own

    /// Broad phase. ipc's own Candidates::build sweeps the WHOLE mesh and, in 2D, only ever
    /// pairs isolated vertices with each other -- neither of which suits a single moving query
    /// point against a fixed complex. So the broad phase is ours: one box query per evaluation,
    /// against the complex as loaded.
    SimpleBVH::BVH edge_bvh;
    bool has_edges = false;
    SimpleBVH::BVH point_bvh; ///< isolated complex vertices, as degenerate segments
    bool has_points = false;
    std::vector<int> isolated; ///< complex vertex ids with no incident segment

    /// Per-thread evaluation state. The smoothing pass is parallel, so `value`, `gradient` and
    /// `hessian` must not share the query row or the candidate sets.
    struct Scratch
    {
        Eigen::MatrixXd V;
        ipc::Candidates candidates;
        bool initialized = false;
    };
    mutable wmtk::threading::enumerable_thread_specific<Scratch> scratch;

    Impl(const double dhat)
        : params(dhat, UNUSED_DBAR_FACTOR, UNUSED_QUAD_ORDER, /*ogc_collisions=*/true)
    {
    }

    /// The collision set at `p`, or null when nothing is within the support. Writes the query
    /// point into the calling thread's scratch V first, so the caller evaluates against `s.V`.
    std::unique_ptr<ipc::HighOrderCollisionDict<ipc::PointType::VERTEX, 2>>
    collisions(const Vector2d& p, Scratch& s) const
    {
        const auto q = static_cast<ipc::index_t>(n_complex_v);
        const double dhat = params.dhat;

        if (!s.initialized) {
            s.V = mesh.rest_positions();
            s.candidates.mesh_ = mesh;
            s.initialized = true;
        }
        s.V.row(n_complex_v) = p.transpose();

        // Everything within dhat of p, from our own BVH. In 2D, Candidates::vv_set() derives
        // the vertex-vertex candidates from the ENDPOINTS of the edge candidates, so seeding
        // the edge set alone also yields every complex vertex whose incident segment is in
        // range -- which is what makes a convex corner work at all, since there no edge claims
        // the point and only the vertex does.
        const SimpleBVH::Vector3d lo(p[0] - dhat, p[1] - dhat, -dhat);
        const SimpleBVH::Vector3d hi(p[0] + dhat, p[1] + dhat, dhat);

        s.candidates.m_ve_set.clear();
        s.candidates.m_vv_set.clear();

        std::vector<unsigned int> hits;
        if (has_edges) {
            edge_bvh.intersect_box(lo, hi, hits);
            if (!hits.empty()) {
                std::set<ipc::index_t>& ve = s.candidates.m_ve_set[q];
                for (const unsigned int e : hits) {
                    ve.insert(static_cast<ipc::index_t>(e));
                }
            }
        }
        if (has_points) {
            point_bvh.intersect_box(lo, hi, hits);
            if (!hits.empty()) {
                // An isolated complex vertex is in no segment, so it can only reach the
                // potential as an explicit vertex-vertex candidate.
                std::set<ipc::index_t>& vv = s.candidates.m_vv_set[q];
                for (const unsigned int i : hits) {
                    vv.insert(static_cast<ipc::index_t>(isolated[i]));
                }
            }
        }
        if (s.candidates.m_ve_set.empty() && s.candidates.m_vv_set.empty()) {
            return nullptr;
        }

        // Upstream decides which of those candidates actually contribute: the OGC
        // feasible-region rule, the P_E interiority test and the dhat cut. Deliberately not
        // reimplemented here.
        const ipc::PointPotential pp(mesh, s.candidates, params, nullptr);
        size_t n_pairs = 0;
        auto dict = pp.build_collisions_at_vertex_ogc_2d(s.V, q, n_pairs);
        if (!dict || dict->size() == 0) {
            return nullptr;
        }
        return dict;
    }
};


OffsetPotential::OffsetPotential(
    const MatrixXd& V,
    const MatrixXi& E,
    const std::vector<int>& P,
    const double delta,
    const double dhat_factor)
    : m_delta(delta)
    , m_dhat(dhat_factor * delta)
{
    if (!(delta > 0.)) {
        log_and_throw_error("OffsetPotential: target_distance must be positive, got {}", delta);
    }
    if (!(dhat_factor > 1.)) {
        // At dhat_factor == 1 the offset sits exactly ON the support boundary, where Phi and its
        // gradient are both zero: a vertex there is given no direction to move in and the level
        // set Phi = c does not exist. Below 1 there is no level set at all.
        log_and_throw_error(
            "OffsetPotential: offset_dhat_factor must be > 1 (the offset distance has to lie "
            "strictly inside the potential's support), got {}",
            dhat_factor);
    }

    build(V, E, P);

    // CALIBRATION, through this same code path rather than from a closed form. Phi at
    // perpendicular distance delta from a long straight edge: one active pair, no feature
    // interaction, so this is the level the offset takes on any flat stretch of the input.
    const OffsetPotential reference(delta, dhat_factor, 0);
    m_c = reference.value(Vector2d(0., delta));
    m_grad_ref = reference.gradient(Vector2d(0., delta)).norm();

    if (!(m_c > 0.) || !(m_grad_ref > 0.)) {
        log_and_throw_error(
            "OffsetPotential: calibration failed (c = {}, |grad| = {}). The straight-edge "
            "reference produced no active contact pair, which means the collision set is not "
            "being built.",
            m_c,
            m_grad_ref);
    }

    logger().info(
        "\tSmooth offset potential: delta {:.6}, dhat {:.6} ({}x delta), level c {:.6}, "
        "|grad Phi| at the level set {:.6} | complex: {} vertices, {} segments, {} isolated points",
        m_delta,
        m_dhat,
        dhat_factor,
        m_c,
        m_grad_ref,
        V.rows(),
        E.rows(),
        P.size());
}


OffsetPotential::OffsetPotential(const double delta, const double dhat_factor, int)
    : m_delta(delta)
    , m_dhat(dhat_factor * delta)
{
    // A single segment long enough that the query point at (0, delta) projects to its interior
    // and both endpoints are far outside the support, so exactly one Vertex2-Edge2P1 pair is
    // active -- which is the definition of "a flat stretch of input".
    const double L = 100. * m_dhat;
    MatrixXd V(2, 2);
    V << -L, 0., L, 0.;
    MatrixXi E(1, 2);
    E << 0, 1;
    build(V, E, {});
    // m_c and m_grad_ref stay 0 here: the reference is only ever asked for value() and
    // gradient(), never for a residual.
}


OffsetPotential::~OffsetPotential() = default;


void OffsetPotential::build(const MatrixXd& V, const MatrixXi& E, const std::vector<int>& P)
{
    if (V.cols() != 2) {
        log_and_throw_error("OffsetPotential is 2D only, got {} columns", V.cols());
    }

    m_impl = std::make_unique<Impl>(m_dhat);
    m_impl->n_complex_v = static_cast<size_t>(V.rows());

    // The collision mesh: the complex, plus one trailing row that every evaluation overwrites
    // with the query point. It belongs to no segment, so it never appears as a contact
    // PRIMITIVE -- only as the point the primitives are measured from.
    //
    // A COMPLEX OF ISOLATED POINTS ONLY (topological_offset_2d_vertex_input is exactly this)
    // needs one more row still. ipc::CollisionMesh::are_adjacencies_initialized() requires all
    // three adjacency tables to be non-empty, and the edge-vertex table is sized by the edge
    // count -- so with no edges it reads as "not initialized" and every accessor throws,
    // including the one the OGC feasible-region test calls. A single sentinel segment, placed
    // far outside the complex, makes the table non-empty. It is unreachable by construction:
    // the only source of collision CANDIDATES is our own BVH below, which does not index it.
    const bool needs_sentinel = (E.rows() == 0);
    const int n_extra = needs_sentinel ? 3 : 1;

    MatrixXd V_ext(V.rows() + n_extra, 2);
    V_ext.topRows(V.rows()) = V;
    V_ext.bottomRows(n_extra).setZero();

    MatrixXi E_ext = E;
    if (needs_sentinel) {
        const Vector2d far = V.colwise().maxCoeff().transpose() +
                             Vector2d::Constant(1e3 * m_dhat + 1.);
        V_ext.row(V.rows() + 1) = far.transpose();
        V_ext.row(V.rows() + 2) = (far + Vector2d(m_dhat, 0.)).transpose();
        E_ext.resize(1, 2);
        E_ext << static_cast<int>(V.rows()) + 1, static_cast<int>(V.rows()) + 2;
    }

    m_impl->mesh = ipc::CollisionMesh(V_ext, E_ext, Eigen::MatrixXi());
    m_impl->mesh.init_adjacencies(); // the OGC feasible-region test reads them

    // Broad phase. SimpleBVH is 3D, so pad; the complex lies in z = 0.
    MatrixXd V3(V.rows(), 3);
    V3.setZero();
    V3.leftCols(2) = V;

    if (E.rows() > 0) {
        m_impl->edge_bvh.init(V3, E, 1e-6);
        m_impl->has_edges = true;
    }

    m_impl->isolated = P;
    if (!P.empty()) {
        // As pseudo-segments (p, p), the same encoding SimplicialComplexBVH uses for the
        // isolated vertices of the complex.
        MatrixXi PE(P.size(), 2);
        for (size_t i = 0; i < P.size(); ++i) {
            PE(i, 0) = P[i];
            PE(i, 1) = P[i];
        }
        m_impl->point_bvh.init(V3, PE, 1e-6);
        m_impl->has_points = true;
    }
}


double OffsetPotential::value(const Vector2d& p) const
{
    auto& s = m_impl->scratch.local();
    const auto dict = m_impl->collisions(p, s);
    if (!dict) {
        return 0.; // nothing within the support: Phi is identically zero out here
    }
    return ipc::PointPotentialHelper::evaluate_potential_at_vertex_2d(
        s.V, *dict, m_impl->params, nullptr);
}


Vector2d OffsetPotential::gradient(const Vector2d& p) const
{
    auto& s = m_impl->scratch.local();
    const auto dict = m_impl->collisions(p, s);
    if (!dict) {
        return Vector2d::Zero();
    }
    const Eigen::VectorXd g = ipc::PointPotentialHelper::evaluate_potential_gradient_at_vertex_2d(
        s.V, *dict, m_impl->params, nullptr);
    // The gradient covers the whole stencil -- the query point and every complex vertex it
    // touches. Only the query point moves, so only its 2x2 block is ours.
    const ipc::index_t li =
        dict->vertex_ids_inverse(static_cast<ipc::index_t>(m_impl->n_complex_v));
    return g.segment<2>(2 * li);
}


Matrix2d OffsetPotential::hessian(const Vector2d& p) const
{
    auto& s = m_impl->scratch.local();
    const auto dict = m_impl->collisions(p, s);
    if (!dict) {
        return Matrix2d::Zero();
    }
    // PSDProjectionMethod::NONE: the TRUE Hessian, projected (or not) by the caller. The
    // smoothing energy squares the residual and takes its own Gauss-Newton approximation, which
    // is a different and better-motivated way to reach a PSD matrix than clamping this one.
    const Eigen::MatrixXd H = ipc::PointPotentialHelper::evaluate_potential_hessian_at_vertex_2d(
        s.V, *dict, m_impl->params, nullptr, ipc::PSDProjectionMethod::NONE);
    const ipc::index_t li =
        dict->vertex_ids_inverse(static_cast<ipc::index_t>(m_impl->n_complex_v));
    return H.block<2, 2>(2 * li, 2 * li);
}


double OffsetPotential::residual_length(const Vector2d& p) const
{
    // |Phi(p) - c| divided by the REFERENCE gradient magnitude -- the slope of Phi at the level
    // set on a flat stretch of input -- rather than by the local |grad Phi(p)|.
    //
    // The local form is the textbook Newton distance to a level set and is the more accurate of
    // the two NEAR the level set, where the two agree anyway. It is badly wrong everywhere else,
    // in the direction that matters most: as p approaches the input complex, Phi ~ -log(d) and
    // |grad Phi| ~ 1/d, so |Phi - c|/|grad Phi| ~ d log(1/d) -> 0. A vertex sitting ON the
    // complex would report a residual of zero and the loop would call it converged, when it is
    // in fact the worst-placed vertex in the mesh.
    //
    // Dividing by the fixed reference slope keeps the quantity MONOTONE in Phi, hence monotone
    // in distance wherever one pair is active: it is a length, it agrees with |d - delta| to
    // first order at the level set, it goes to infinity on the complex, and it saturates at
    // c / |grad_ref| ~ 0.29*delta outside the support -- which is above any sane tolerance, and
    // in any case the runaway guard turns "outside the support" into a hard error before this
    // number is ever the thing deciding.
    return std::abs(value(p) - m_c) / m_grad_ref;
}


// ---------------------------------------------------------------------------------------------


OffsetEnergy2D::OffsetEnergy2D(
    const std::shared_ptr<const OffsetPotential>& potential,
    const double weight,
    const bool gauss_newton)
    : m_potential(potential)
    , m_weight(weight)
    , m_gauss_newton(gauss_newton)
{
}


double OffsetEnergy2D::value(const TVector& x)
{
    const double r = m_potential->value(Vector2d(x[0], x[1])) - m_potential->target_level();
    return m_weight * r * r;
}


void OffsetEnergy2D::gradient(const TVector& x, TVector& gradv)
{
    const Vector2d p(x[0], x[1]);
    const double r = m_potential->value(p) - m_potential->target_level();
    gradv = 2. * m_weight * r * m_potential->gradient(p);
}


void OffsetEnergy2D::hessian(const TVector& x, MatrixXd& hessian)
{
    const Vector2d p(x[0], x[1]);
    const Vector2d g = m_potential->gradient(p);
    Matrix2d H = 2. * m_weight * g * g.transpose();
    if (!m_gauss_newton) {
        const double r = m_potential->value(p) - m_potential->target_level();
        H += 2. * m_weight * r * m_potential->hessian(p);
    }
    hessian = H;
}

} // namespace wmtk::components::topological_offset
