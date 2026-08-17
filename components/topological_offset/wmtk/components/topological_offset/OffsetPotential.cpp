#include "OffsetPotential.hpp"

#include <wmtk/utils/Logger.hpp>

#include <SimpleBVH/BVH.hpp>

#include <ipc/candidates/candidates.hpp>
#include <ipc/collision_mesh.hpp>
#include <ipc/high_order_contact/high_order_contact_parameters.hpp>
#include <ipc/high_order_contact/quadrature_potential.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <string>

namespace wmtk::components::topological_offset {

namespace {
/// Quadrature order is irrelevant on this path -- Phi is a POINT evaluation against a cached
/// collision set, with no quadrature anywhere in it -- but HighOrderContactParameters requires
/// one and logs a warning for order 1. 2 is the smallest value it accepts silently.
constexpr int UNUSED_QUAD_ORDER = 2;

/// dbar_factor, likewise unread by the vertex path (it feeds the near/far barrier split, which
/// this does not use). 1.0 is upstream's default.
constexpr double UNUSED_DBAR_FACTOR = 1.0;

/// The dict type upstream hands back for a vertex query. In 3D DIM is the template's own
/// default, so `<VERTEX, 3>` and `<VERTEX>` name the same type and the helpers below match.
template <int DIM>
using VertexDict = ipc::HighOrderCollisionDict<ipc::PointType::VERTEX, DIM>;
} // namespace


/**
 * @brief Everything that mentions ipc-toolkit.
 *
 * The collision mesh carries the complex's vertices plus ONE extra row for the query point,
 * which is the pattern the upstream API is written around ("with the point q appended to the
 * last row"). Its topology never changes, so it is built once; an evaluation only writes the
 * last row and rebuilds the small collision set around it.
 */
template <int DIM>
struct SmoothOffsetPotential<DIM>::Impl
{
    ipc::CollisionMesh mesh; ///< complex vertices + the query vertex
    ipc::HighOrderContactParameters params;
    size_t n_complex_v = 0; ///< index of the query vertex, one past the complex's own

    /// Broad phase. ipc's own Candidates::build sweeps the WHOLE mesh and pairs its primitives
    /// with each other, neither of which suits a single moving query point against a fixed
    /// complex. So the broad phase is ours: one box query per evaluation, against the complex as
    /// loaded.
    ///
    /// 3D indexes the TRIANGLES and derives their edges and vertices from a hit, which is
    /// complete: a primitive within dhat of q lies inside the AABB of every triangle containing
    /// it, and that AABB therefore meets the query box. 2D has no triangles and indexes the
    /// segments instead. Isolated edges (a wire, which belongs to no triangle) and isolated
    /// points can only be reached by their own trees.
    SimpleBVH::BVH face_bvh; ///< 3D only: the complex's triangles
    bool has_faces = false;
    SimpleBVH::BVH edge_bvh; ///< 2D: every segment. 3D: only the segments in no triangle.
    bool has_edges = false;
    std::vector<int> edge_ids; ///< edge_bvh row -> edge id in `mesh`
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
    {}

    /// The collision set at `p`, or null when nothing is within the support. Writes the query
    /// point into the calling thread's scratch V first, so the caller evaluates against `s.V`.
    std::unique_ptr<VertexDict<DIM>> collisions(const VecD& p, Scratch& s) const
    {
        const auto q = static_cast<ipc::index_t>(n_complex_v);
        const double dhat = params.dhat;

        if (!s.initialized) {
            s.V = mesh.rest_positions();
            s.candidates.mesh_ = mesh;
            s.initialized = true;
        }
        s.V.row(n_complex_v) = p.transpose();

        SimpleBVH::Vector3d lo(-dhat, -dhat, -dhat);
        SimpleBVH::Vector3d hi(dhat, dhat, dhat);
        for (int d = 0; d < DIM; ++d) {
            lo[d] += p[d];
            hi[d] += p[d];
        }

        s.candidates.m_vf_set.clear();
        s.candidates.m_ve_set.clear();
        s.candidates.m_vv_set.clear();

        std::vector<unsigned int> hits;

        if (has_faces) {
            // 3D. vf_set, ve_set and vv_set are read INDEPENDENTLY by the 3D builder -- unlike
            // 2D, where Candidates::vv_set() derives the vertex candidates from the endpoints of
            // the edge candidates -- so a triangle's edges and vertices have to be seeded here
            // too. Missing one is silent and wrong in the direction that looks fine: a convex
            // feature whose vertex is never a candidate contributes nothing and the level set
            // has a hole where it should have a spherical cap.
            face_bvh.intersect_box(lo, hi, hits);
            if (!hits.empty()) {
                std::set<ipc::index_t>& vf = s.candidates.m_vf_set[q];
                std::set<ipc::index_t>& ve = s.candidates.m_ve_set[q];
                std::set<ipc::index_t>& vv = s.candidates.m_vv_set[q];
                for (const unsigned int f : hits) {
                    vf.insert(static_cast<ipc::index_t>(f));
                    for (int j = 0; j < 3; ++j) {
                        ve.insert(mesh.faces_to_edges()(f, j));
                        vv.insert(mesh.faces()(f, j));
                    }
                }
            }
        }
        if (has_edges) {
            edge_bvh.intersect_box(lo, hi, hits);
            if (!hits.empty()) {
                std::set<ipc::index_t>& ve = s.candidates.m_ve_set[q];
                for (const unsigned int e : hits) {
                    const auto ei = static_cast<ipc::index_t>(edge_ids[e]);
                    ve.insert(ei);
                    if constexpr (DIM == 3) {
                        // 2D gets its vertex candidates for free: Candidates::vv_set() derives
                        // them from the endpoints of the edge candidates, which is what makes a
                        // convex corner work at all, since there no edge claims the point and
                        // only the vertex does. 3D has no such derivation.
                        std::set<ipc::index_t>& vv = s.candidates.m_vv_set[q];
                        vv.insert(mesh.edges()(ei, 0));
                        vv.insert(mesh.edges()(ei, 1));
                    }
                }
            }
        }
        if (has_points) {
            point_bvh.intersect_box(lo, hi, hits);
            if (!hits.empty()) {
                // An isolated complex vertex is in no segment and no triangle, so it can only
                // reach the potential as an explicit vertex-vertex candidate.
                std::set<ipc::index_t>& vv = s.candidates.m_vv_set[q];
                for (const unsigned int i : hits) {
                    vv.insert(static_cast<ipc::index_t>(isolated[i]));
                }
            }
        }
        if (s.candidates.m_vf_set.empty() && s.candidates.m_ve_set.empty() &&
            s.candidates.m_vv_set.empty()) {
            return nullptr;
        }

        // Upstream decides which of those candidates actually contribute: the OGC
        // feasible-region rule, the interiority tests and the dhat cut. Deliberately not
        // reimplemented here.
        const ipc::PointPotential pp(mesh, s.candidates, params, nullptr);
        size_t n_pairs = 0;
        std::unique_ptr<VertexDict<DIM>> dict;
        if constexpr (DIM == 2) {
            dict = pp.build_collisions_at_vertex_ogc_2d(s.V, q, n_pairs);
        } else {
            dict = pp.build_collisions_at_vertex_ogc_3d(s.V, q, n_pairs);
        }
        if (!dict || dict->size() == 0) {
            return nullptr;
        }
        return dict;
    }

    /// Phi's stencil covers the query point and every complex vertex it touches; only the query
    /// point moves, so only its DIM-sized block of a gradient (or DIMxDIM of a Hessian) is ours.
    ipc::index_t local_query_index(const VertexDict<DIM>& dict) const
    {
        return dict.vertex_ids_inverse(static_cast<ipc::index_t>(n_complex_v));
    }
};


template <int DIM>
SmoothOffsetPotential<DIM>::SmoothOffsetPotential(
    const MatrixXd& V,
    const MatrixXi& E,
    const MatrixXi& F,
    const std::vector<int>& P,
    const double delta,
    const double dhat_factor)
    : OffsetPotential<DIM>(delta, dhat_factor * delta)
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

    build(V, E, F, P);

    // CALIBRATION, through this same code path rather than from a closed form. Phi at
    // perpendicular distance delta from one large flat primitive: one active pair, no feature
    // interaction, so this is the level the offset takes on any flat stretch of the input.
    const SmoothOffsetPotential reference(delta, dhat_factor, 0);
    VecD probe = VecD::Zero();
    probe[DIM - 1] = delta;
    m_c = reference.value(probe);
    m_grad_ref = reference.gradient(probe).norm();

    if (!(m_c > 0.) || !(m_grad_ref > 0.)) {
        log_and_throw_error(
            "OffsetPotential: calibration failed (c = {}, |grad| = {}). The flat reference "
            "produced no active contact pair, which means the collision set is not being built.",
            m_c,
            m_grad_ref);
    }

    logger().info(
        "\tSmooth offset potential ({}D): delta {:.6}, dhat {:.6} ({}x delta), level c {:.6}, "
        "|grad Phi| at the level set {:.6} | complex: {} vertices, {} segments, {} triangles, "
        "{} isolated points",
        DIM,
        m_delta,
        m_dhat,
        dhat_factor,
        m_c,
        m_grad_ref,
        V.rows(),
        E.rows(),
        F.rows(),
        P.size());
}


template <int DIM>
SmoothOffsetPotential<DIM>::SmoothOffsetPotential(const double delta, const double dhat_factor, int)
    : OffsetPotential<DIM>(delta, dhat_factor * delta)
{
    // One primitive large enough that the probe at perpendicular distance delta projects into
    // its interior and every one of its boundary features is far outside the support, so exactly
    // one pair is active -- which is the definition of "a flat stretch of input".
    const double L = 100. * m_dhat;
    if constexpr (DIM == 2) {
        MatrixXd V(2, 2);
        V << -L, 0., L, 0.;
        MatrixXi E(1, 2);
        E << 0, 1;
        build(V, E, MatrixXi(0, 3), {});
    } else {
        // (0,0) sits strictly inside this triangle at ~0.45 L from its nearest edge.
        MatrixXd V(3, 3);
        V << -L, -L, 0., L, -L, 0., 0., L, 0.;
        MatrixXi E(3, 2);
        E << 0, 1, 1, 2, 2, 0;
        MatrixXi F(1, 3);
        F << 0, 1, 2;
        build(V, E, F, {});
    }
    // m_c and m_grad_ref stay 0 here: the reference is only ever asked for value() and
    // gradient(), never for a residual.
}


template <int DIM>
SmoothOffsetPotential<DIM>::~SmoothOffsetPotential() = default;


template <int DIM>
void SmoothOffsetPotential<DIM>::build(
    const MatrixXd& V,
    const MatrixXi& E,
    const MatrixXi& F,
    const std::vector<int>& P)
{
    if (V.cols() != DIM) {
        log_and_throw_error("OffsetPotential<{}> was given {}-column vertices", DIM, V.cols());
    }
    if constexpr (DIM == 2) {
        if (F.rows() != 0) {
            log_and_throw_error("OffsetPotential<2> has no triangle primitive, got {}", F.rows());
        }
    }

    m_impl = std::make_unique<Impl>(m_dhat);
    m_impl->n_complex_v = static_cast<size_t>(V.rows());

    // The collision mesh: the complex, plus one trailing row that every evaluation overwrites
    // with the query point. It belongs to no segment and no triangle, so it never appears as a
    // contact PRIMITIVE -- only as the point the primitives are measured from.
    //
    // A COMPLEX WITH NO SEGMENTS AT ALL (topological_offset_2d_vertex_input is exactly this, a
    // set of isolated points) needs one more row still. ipc::CollisionMesh's
    // are_adjacencies_initialized() requires all three adjacency tables to be non-empty, and the
    // edge-vertex table is sized by the EDGE count -- so with no edges it reads as "not
    // initialized" and every accessor throws, including the one the OGC feasible-region test
    // calls. A single sentinel segment, placed far outside the complex, makes the table
    // non-empty. It is unreachable by construction: the only source of collision CANDIDATES is
    // our own BVHs below, which do not index it. No sentinel TRIANGLE is ever needed, because a
    // complex with triangles necessarily has their edges.
    const bool needs_sentinel = (E.rows() == 0);
    const int n_extra = needs_sentinel ? 3 : 1;

    MatrixXd V_ext(V.rows() + n_extra, DIM);
    V_ext.topRows(V.rows()) = V;
    V_ext.bottomRows(n_extra).setZero();

    MatrixXi E_ext = E;
    if (needs_sentinel) {
        const VecD far = V.colwise().maxCoeff().transpose() + VecD::Constant(1e3 * m_dhat + 1.);
        VecD far2 = far;
        far2[0] += m_dhat;
        V_ext.row(V.rows() + 1) = far.transpose();
        V_ext.row(V.rows() + 2) = far2.transpose();
        E_ext.resize(1, 2);
        E_ext << static_cast<int>(V.rows()) + 1, static_cast<int>(V.rows()) + 2;
    }

    m_impl->mesh = ipc::CollisionMesh(V_ext, E_ext, F);
    m_impl->mesh.init_adjacencies(); // the OGC feasible-region test reads them

    // Broad phase. SimpleBVH is 3D, so pad; a 2D complex lies in z = 0.
    MatrixXd V3(V.rows(), 3);
    V3.setZero();
    V3.leftCols(DIM) = V;

    if (F.rows() > 0) {
        m_impl->face_bvh.init(V3, F, 1e-6);
        m_impl->has_faces = true;
    }

    // In 3D a triangle's own edges are reached through the triangle, so the edge tree only has
    // to carry the segments that belong to no triangle -- the complex's wires. Indexing all of
    // them would be correct but would seed the same candidates twice.
    std::vector<int> tree_edges;
    if constexpr (DIM == 3) {
        std::vector<bool> in_face(E.rows(), false);
        std::map<std::pair<int, int>, int> edge_of;
        for (int i = 0; i < E.rows(); ++i) {
            edge_of[{std::min(E(i, 0), E(i, 1)), std::max(E(i, 0), E(i, 1))}] = i;
        }
        for (int f = 0; f < F.rows(); ++f) {
            for (int j = 0; j < 3; ++j) {
                const int a = F(f, j);
                const int b = F(f, (j + 1) % 3);
                const auto it = edge_of.find({std::min(a, b), std::max(a, b)});
                if (it == edge_of.end()) {
                    // ipc would throw the same thing from construct_faces_to_edges, but with no
                    // hint about which caller built the list.
                    log_and_throw_error(
                        "OffsetPotential<3>: edge ({}, {}) of triangle {} is missing from E. The "
                        "edge list must contain every edge of every triangle.",
                        a,
                        b,
                        f);
                }
                in_face[it->second] = true;
            }
        }
        for (int i = 0; i < E.rows(); ++i) {
            if (!in_face[i]) tree_edges.push_back(i);
        }
    } else {
        for (int i = 0; i < E.rows(); ++i) tree_edges.push_back(i);
    }

    if (!tree_edges.empty()) {
        MatrixXi E_tree(tree_edges.size(), 2);
        for (size_t i = 0; i < tree_edges.size(); ++i) {
            E_tree.row(i) = E.row(tree_edges[i]);
        }
        m_impl->edge_bvh.init(V3, E_tree, 1e-6);
        m_impl->has_edges = true;
        m_impl->edge_ids = tree_edges;
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


template <int DIM>
double SmoothOffsetPotential<DIM>::value(const VecD& p) const
{
    auto& s = m_impl->scratch.local();
    const auto dict = m_impl->collisions(p, s);
    if (!dict) {
        return 0.; // nothing within the support: Phi is identically zero out here
    }
    if constexpr (DIM == 2) {
        return ipc::PointPotentialHelper::evaluate_potential_at_vertex_2d(
            s.V,
            *dict,
            m_impl->params,
            nullptr);
    } else {
        return ipc::PointPotentialHelper::evaluate_potential_at_vertex_with_cached_collisions(
            s.V,
            *dict,
            m_impl->params,
            nullptr);
    }
}


template <int DIM>
typename SmoothOffsetPotential<DIM>::VecD SmoothOffsetPotential<DIM>::gradient(const VecD& p) const
{
    auto& s = m_impl->scratch.local();
    const auto dict = m_impl->collisions(p, s);
    if (!dict) {
        return VecD::Zero();
    }
    Eigen::VectorXd g;
    if constexpr (DIM == 2) {
        g = ipc::PointPotentialHelper::evaluate_potential_gradient_at_vertex_2d(
            s.V,
            *dict,
            m_impl->params,
            nullptr);
    } else {
        g = ipc::PointPotentialHelper::evaluate_potential_gradient_at_vertex_with_cached_collisions(
            s.V,
            *dict,
            m_impl->params,
            nullptr);
    }
    const ipc::index_t li = m_impl->local_query_index(*dict);
    return g.template segment<DIM>(DIM * li);
}


template <int DIM>
typename SmoothOffsetPotential<DIM>::MatD SmoothOffsetPotential<DIM>::hessian(const VecD& p) const
{
    auto& s = m_impl->scratch.local();
    const auto dict = m_impl->collisions(p, s);
    if (!dict) {
        return MatD::Zero();
    }
    // PSDProjectionMethod::NONE: the TRUE Hessian, projected (or not) by the caller. The
    // smoothing energy squares the residual and takes its own Gauss-Newton approximation, which
    // is a different and better-motivated way to reach a PSD matrix than clamping this one.
    Eigen::MatrixXd H;
    if constexpr (DIM == 2) {
        H = ipc::PointPotentialHelper::evaluate_potential_hessian_at_vertex_2d(
            s.V,
            *dict,
            m_impl->params,
            nullptr,
            ipc::PSDProjectionMethod::NONE);
    } else {
        H = ipc::PointPotentialHelper::evaluate_potential_hessian_at_vertex_with_cached_collisions(
            s.V,
            *dict,
            m_impl->params,
            nullptr,
            ipc::PSDProjectionMethod::NONE);
    }
    const ipc::index_t li = m_impl->local_query_index(*dict);
    return H.template block<DIM, DIM>(DIM * li, DIM * li);
}


template <int DIM>
std::string SmoothOffsetPotential<DIM>::describe_active(const VecD& p) const
{
    auto& s = m_impl->scratch.local();
    const auto dict = m_impl->collisions(p, s);
    if (!dict) return "<nothing active>";
    std::string out;
    for (int i = 0; i < dict->size(); ++i) {
        const auto& cc = (*dict)[i];
        std::string ids;
        for (const ipc::index_t v : cc.vertex_ids()) {
            ids += fmt::format("{} ", v);
        }
        out += fmt::format("[{} w={} verts: {}] ", cc.name(), cc.weight, ids);
    }
    return out;
}


template <int DIM>
double SmoothOffsetPotential<DIM>::residual_length(const VecD& p) const
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


template <int DIM>
OffsetEnergy<DIM>::OffsetEnergy(
    const std::shared_ptr<const OffsetPotential<DIM>>& potential,
    const double weight,
    const bool gauss_newton)
    : m_potential(potential)
    , m_weight(weight)
    , m_gauss_newton(gauss_newton)
{}


template <int DIM>
double OffsetEnergy<DIM>::value(const TVector& x)
{
    const double r = m_potential->value(VecD(x.head(DIM))) - m_potential->target_level();
    return m_weight * r * r;
}


template <int DIM>
void OffsetEnergy<DIM>::gradient(const TVector& x, TVector& gradv)
{
    const VecD p = x.head(DIM);
    const double r = m_potential->value(p) - m_potential->target_level();
    gradv = 2. * m_weight * r * m_potential->gradient(p);
}


template <int DIM>
void OffsetEnergy<DIM>::hessian(const TVector& x, MatrixXd& hessian)
{
    const VecD p = x.head(DIM);
    const VecD g = m_potential->gradient(p);
    MatD H = 2. * m_weight * g * g.transpose();
    if (!m_gauss_newton) {
        const double r = m_potential->value(p) - m_potential->target_level();
        H += 2. * m_weight * r * m_potential->hessian(p);
    }
    hessian = H;
}


// ---------------------------------------------------------------------------------------------
// The Euclidean field.
// ---------------------------------------------------------------------------------------------

template <int DIM>
EuclideanOffsetPotential<DIM>::EuclideanOffsetPotential(
    const std::shared_ptr<SampleEnvelope>& envelope,
    const double delta)
    // NO SUPPORT LIMIT. d is defined and informative everywhere, so the runaway guard that exists
    // for Phi's compact support has nothing to catch here; infinity says so rather than implying
    // it with a large finite number that something might later compare against.
    : OffsetPotential<DIM>(delta, std::numeric_limits<double>::infinity())
    , m_envelope(envelope)
{
    if (!(delta > 0.)) {
        log_and_throw_error(
            "EuclideanOffsetPotential: target_distance must be positive, got {}",
            delta);
    }
    if (!m_envelope) {
        log_and_throw_error("EuclideanOffsetPotential: needs an envelope to query");
    }
    // NO CALIBRATION. The level IS the offset distance -- that is the entire content of "the
    // Euclidean offset" -- where the smooth potential has to discover its own level by evaluating
    // Phi at distance delta from a flat reference.
    m_c = delta;
}

template <int DIM>
void EuclideanOffsetPotential<DIM>::nearest_feature(const VecD& p, VecD& foot, int& dim, VecD& dir)
    const
{
    if constexpr (DIM == 2) {
        bool on_corner = false;
        int feature_id = -1;
        Eigen::Vector2d seg_normal;
        m_envelope->nearest_point_feature(p, foot, on_corner, seg_normal, feature_id);
        // The 2D query reports the segment NORMAL; the Hessian below is cased on the direction
        // ALONG the feature, so a segment interior is dim 1 with the tangent, obtained by
        // rotating the normal a quarter turn.
        dim = on_corner ? 0 : 1;
        dir = on_corner ? VecD::Zero().eval() : VecD(-seg_normal.y(), seg_normal.x());
    } else {
        long long feature_id = -1;
        m_envelope->nearest_point_feature(p, foot, dim, dir, feature_id);
    }

    // A DEGENERATE SEGMENT IS A POINT, not an edge. Both SimplicialComplexBVH and the envelope
    // carry an isolated input vertex as the pseudo-edge (i, i), so a query near one comes back as
    // an edge-interior hit whose direction is whatever normalising a zero vector produced. The
    // edge Hessian would then subtract a meaningless t t^T. Demote it to the vertex case, which
    // is what the geometry actually is. topological_offset_2d_vertex_input is a point cloud and
    // is made entirely of these.
    if (dim == 1 && !(dir.norm() > 0.5)) {
        dim = 0;
        dir = VecD::Zero();
    }
}

template <int DIM>
double EuclideanOffsetPotential<DIM>::value(const VecD& p) const
{
    return std::sqrt(m_envelope->squared_distance(p));
}

template <int DIM>
typename EuclideanOffsetPotential<DIM>::VecD EuclideanOffsetPotential<DIM>::gradient(
    const VecD& p) const
{
    VecD foot = VecD::Zero(), dir = VecD::Zero();
    int dim = -1;
    nearest_feature(p, foot, dim, dir);

    const VecD r = p - foot;
    const double d = r.norm();
    // ON the complex the gradient of d does not exist -- every direction increases it equally.
    // Zero is the honest answer and the one the smoother handles: it contributes no offset force,
    // so the vertex is moved by the quality term alone. Such a vertex is excluded from the offset
    // term anyway (smoothing_extra_energy refuses an input-complex vertex) and from the criterion
    // (band_vertex_is_reachable books it as pinned), so this is a belt-and-braces case.
    if (!(d > 1e-14)) {
        return VecD::Zero();
    }
    return r / d;
}

template <int DIM>
typename EuclideanOffsetPotential<DIM>::MatD EuclideanOffsetPotential<DIM>::hessian(
    const VecD& p) const
{
    VecD foot = VecD::Zero(), dir = VecD::Zero();
    int dim = -1;
    nearest_feature(p, foot, dim, dir);

    const VecD r = p - foot;
    const double d = r.norm();
    if (!(d > 1e-14)) {
        return MatD::Zero();
    }
    const VecD u = r / d;

    // grad^2 d, by feature kind. Transcribed from ExactDistanceEnergy2D/3D, which state the
    // Hessian of d^2; grad^2(d^2) = 2 (grad d grad d^T + d grad^2 d) converts one to the other.
    // See the class comment for the table.
    if (dim == DIM - 1) {
        // Face interior in 3D, segment interior in 2D: d is LINEAR in p there, so no curvature.
        return MatD::Zero();
    }
    if (dim == 1) {
        // 3D edge interior: free along the edge, curved around it.
        return (MatD::Identity() - dir * dir.transpose() - u * u.transpose()) / d;
    }
    // Vertex: distance to a point, curved in every direction but radially.
    return (MatD::Identity() - u * u.transpose()) / d;
}

template <int DIM>
std::string EuclideanOffsetPotential<DIM>::describe_active(const VecD& p) const
{
    VecD foot = VecD::Zero(), dir = VecD::Zero();
    int dim = -1;
    nearest_feature(p, foot, dim, dir);
    static constexpr std::array<const char*, 3> kinds = {{"vertex", "edge interior", "face"}};
    return fmt::format(
        "nearest feature: {} at ({}), d = {:.6g}, level = {:.6g}, residual = {:.6g}",
        kinds[size_t(std::clamp(dim, 0, 2))],
        fmt::join(std::vector<double>(foot.data(), foot.data() + DIM), ", "),
        (p - foot).norm(),
        m_c,
        residual_length(p));
}


template <int DIM>
OffsetPotential<DIM>::~OffsetPotential() = default;


template class OffsetPotential<2>;
template class OffsetPotential<3>;
template class SmoothOffsetPotential<2>;
template class SmoothOffsetPotential<3>;
template class EuclideanOffsetPotential<2>;
template class EuclideanOffsetPotential<3>;
template class OffsetEnergy<2>;
template class OffsetEnergy<3>;

} // namespace wmtk::components::topological_offset
