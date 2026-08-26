#include "OffsetPotential.hpp"

#include <wmtk/utils/Logger.hpp>

#include <SimpleBVH/BVH.hpp>

#include <ipc/candidates/candidates.hpp>
#include <ipc/collision_mesh.hpp>
#include <ipc/high_order_contact/arbitrary_point_potential.hpp>
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

/// The query point in the row form ipc::ArbitraryPointPotential takes. 3D only -- ESP at an
/// arbitrary point has no 2D counterpart upstream, which is why 2D keeps the OGC vertex builder.
inline Eigen::RowVector3d esp_query(const Eigen::Vector3d& p)
{
    return p.transpose();
}

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

    /// 3D ONLY: the ESP evaluator, which subsumes everything above. It owns its own broad phase
    /// and does its own feature classification, so on this path `mesh` is the ONLY thing built
    /// -- no scratch, no candidate sets, and none of the four BVHs.
    ///
    /// `V_complex` is kept because every call takes the vertex configuration as an argument, and
    /// it must contain the complex and NOTHING ELSE: ArbitraryPointBVH indexes every row of it as
    /// a vertex primitive, so a padding row would be a real point of the complex sitting at
    /// whatever coordinates it happened to hold.
    /// The 2-manifold sub-complex, compacted and padded; NOT the same mesh as `mesh` above,
    /// which the OGC part needs in the complex's own indexing.
    ipc::CollisionMesh mesh_esp;
    Eigen::MatrixXd V_complex;
    std::unique_ptr<ipc::ArbitraryPointPotential> esp; ///< null when the complex has no triangles

    /// Per-thread evaluation state. The smoothing pass is parallel, so `value`, `gradient` and
    /// `hessian` must not share the query row or the candidate sets.
    struct Scratch
    {
        Eigen::MatrixXd V;
        ipc::Candidates candidates;
        bool initialized = false;
    };
    mutable wmtk::threading::enumerable_thread_specific<Scratch> scratch;

    /// ogc_collisions tracks which builder this dimension uses. Neither path actually READS it
    /// -- 2D calls build_collisions_at_vertex_ogc_2d() by name, and ArbitraryPointPotential
    /// never consults it -- so it is documentation that happens to be a field.
    Impl(const double dhat)
        : params(dhat, UNUSED_DBAR_FACTOR, UNUSED_QUAD_ORDER, /*ogc_collisions=*/DIM == 2)
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

    // ---- 3D: ESP over the 2-MANIFOLD part of the complex ----
    //
    // ESP's alternating sum (Eq. 6: +faces -edges +vertices) is inclusion-exclusion over a CLOSED
    // SURFACE. Every -1 edge term exists to cancel the +1 the incident faces contribute when the
    // query's closest point on them lands on that shared edge; the net over a genuine surface is
    // exactly one +b(d) at the true closest feature. A primitive with NO incident face has
    // nothing to cancel against, so the sum inverts: measured on the wire fixture (one segment,
    // no triangles), Phi = -b(d) exactly -- a barrier with the wrong sign, unbounded BELOW as the
    // query approaches the segment.
    //
    // So the complex is split by what ESP is defined on. Triangles, their edges and their
    // vertices go to ArbitraryPointPotential. Segments in no triangle (wires) and isolated points
    // stay on the OGC vertex builder below, which weights every active primitive +1 and is
    // therefore right for a sub-manifold piece. The two sums add, which is the same superposition
    // the potential already applies wherever two features both claim a point.
    if constexpr (DIM == 3) {
        if (F.rows() > 0) {
            std::map<std::pair<int, int>, int> edge_of;
            for (int i = 0; i < E.rows(); ++i) {
                edge_of[{std::min(E(i, 0), E(i, 1)), std::max(E(i, 0), E(i, 1))}] = i;
            }
            // COMPACTED to the face-incident vertices. ipc::LBVH indexes every row it is given as
            // a vertex primitive, so carrying a wire or isolated vertex here would give it a +1
            // term in the surface sum on top of the one the OGC part already gives it.
            std::vector<int> to_surf(V.rows(), -1);
            std::vector<int> surf_v;
            const auto claim = [&](const int v) {
                if (to_surf[v] < 0) {
                    to_surf[v] = static_cast<int>(surf_v.size());
                    surf_v.push_back(v);
                }
                return to_surf[v];
            };
            MatrixXi F_s(F.rows(), 3);
            std::set<std::pair<int, int>> surf_e;
            for (int f = 0; f < F.rows(); ++f) {
                for (int j = 0; j < 3; ++j) F_s(f, j) = claim(F(f, j));
                for (int j = 0; j < 3; ++j) {
                    const int a0 = F(f, j), b0 = F(f, (j + 1) % 3);
                    if (edge_of.find({std::min(a0, b0), std::max(a0, b0)}) == edge_of.end()) {
                        // ipc would throw the same thing from construct_faces_to_edges, but with
                        // no hint about which caller built the list.
                        log_and_throw_error(
                            "OffsetPotential<3>: edge ({}, {}) of triangle {} is missing from E. "
                            "The edge list must contain every edge of every triangle.",
                            a0,
                            b0,
                            f);
                    }
                    surf_e.insert(
                        {std::min(to_surf[a0], to_surf[b0]), std::max(to_surf[a0], to_surf[b0])});
                }
            }

            // A LONE PRIMITIVE IS COUNTED TWICE, so pad until no tree has exactly one leaf.
            //
            // ipc::LBVH sizes a tree as 2N-1 nodes, so with N == 1 the ROOT IS THE LEAF, while
            // ArbitraryPointBVH::query_point() starts at node 0 asserting it is inner and reads
            // node.left / node.right. A leaf's union puts primitive_id where `left` is and the
            // (zero) inner-marker where `right` is, so for the single primitive -- whose id is
            // necessarily 0 -- both children resolve to the node itself, both are leaves, both
            // intersect, and it is pushed TWICE. insert_pair() then accumulates its weight to 2
            // and the term enters doubled. Measured: the flat calibration reference, one
            // triangle, gave c = 0.346574 = 2 x b(delta) instead of 0.173287.
            //
            // Three vertices, three segments and one triangle, far enough out that nothing within
            // dhat of the real complex can see them, take all three trees to N >= 2.
            const int n_sv = static_cast<int>(surf_v.size());
            const Eigen::RowVector3d far = V.colwise().maxCoeff().array() + 8. * m_dhat + 1.;
            MatrixXd V_s(n_sv + 3, 3);
            for (int i = 0; i < n_sv; ++i) V_s.row(i) = V.row(surf_v[i]);
            V_s.row(n_sv) = far;
            V_s.row(n_sv + 1) = far + Eigen::RowVector3d(m_dhat, 0., 0.);
            V_s.row(n_sv + 2) = far + Eigen::RowVector3d(0., m_dhat, 0.);

            MatrixXi E_s(static_cast<int>(surf_e.size()) + 3, 2);
            int k = 0;
            for (const auto& [x, y] : surf_e) E_s.row(k++) << x, y;
            E_s.row(k++) << n_sv, n_sv + 1;
            E_s.row(k++) << n_sv + 1, n_sv + 2;
            E_s.row(k++) << n_sv + 2, n_sv;

            MatrixXi F_pad(F.rows() + 1, 3);
            F_pad.topRows(F.rows()) = F_s;
            F_pad.row(F.rows()) << n_sv, n_sv + 1, n_sv + 2;

            m_impl->V_complex = V_s;
            m_impl->mesh_esp = ipc::CollisionMesh(V_s, E_s, F_pad);
            m_impl->esp =
                std::make_unique<ipc::ArbitraryPointPotential>(m_impl->mesh_esp, m_impl->params);
            // Once: the complex is fixed for this potential's lifetime.
            m_impl->esp->update(m_impl->V_complex);
        }
    }

    // ---- 2D: the OGC vertex builder, which needs a real mesh vertex to measure from ----
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

    // 2D ONLY. In 3D the triangles (and their edges and vertices) belong to the ESP sum above,
    // so seeding them as OGC candidates as well would count the surface twice.
    if constexpr (DIM == 2) {
        (void)V3;
    } else {
        // has_faces stays false: nothing here indexes F.
    }

    // WHAT THE OGC PART OWNS. In 2D that is every segment. In 3D it is only the segments in no
    // triangle -- the complex's wires -- because the rest went to ESP above.
    std::vector<int> tree_edges;
    if constexpr (DIM == 3) {
        std::set<std::pair<int, int>> in_face;
        for (int f = 0; f < F.rows(); ++f) {
            for (int j = 0; j < 3; ++j) {
                const int a0 = F(f, j), b0 = F(f, (j + 1) % 3);
                in_face.insert({std::min(a0, b0), std::max(a0, b0)});
            }
        }
        for (int i = 0; i < E.rows(); ++i) {
            const int a0 = E(i, 0), b0 = E(i, 1);
            if (in_face.count({std::min(a0, b0), std::max(a0, b0)}) == 0) tree_edges.push_back(i);
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
    // THE TWO SUMS ADD: ESP over the triangles, the OGC vertex builder over the wires and
    // isolated points. Either may be empty -- a closed surface uses only the first, a point
    // cloud or a 2D complex only the second. See build() for why the complex is split.
    double phi = 0.;
    if constexpr (DIM == 3) {
        if (m_impl->esp) phi += (*m_impl->esp)(m_impl->V_complex, esp_query(p));
    }
    auto& s = m_impl->scratch.local();
    const auto dict = m_impl->collisions(p, s);
    if (!dict) {
        return phi; // nothing more within the support
    }
    if constexpr (DIM == 2) {
        phi += ipc::PointPotentialHelper::evaluate_potential_at_vertex_2d(
            s.V,
            *dict,
            m_impl->params,
            nullptr);
    } else {
        phi += ipc::PointPotentialHelper::evaluate_potential_at_vertex_with_cached_collisions(
            s.V,
            *dict,
            m_impl->params,
            nullptr);
    }
    return phi;
}


template <int DIM>
typename SmoothOffsetPotential<DIM>::VecD SmoothOffsetPotential<DIM>::gradient(const VecD& p) const
{
    VecD g_total = VecD::Zero();
    if constexpr (DIM == 3) {
        if (m_impl->esp) g_total += m_impl->esp->gradient(m_impl->V_complex, esp_query(p));
    }
    auto& s = m_impl->scratch.local();
    const auto dict = m_impl->collisions(p, s);
    if (!dict) {
        return g_total;
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
    return g_total + g.template segment<DIM>(DIM * li);
}


template <int DIM>
typename SmoothOffsetPotential<DIM>::MatD SmoothOffsetPotential<DIM>::hessian(const VecD& p) const
{
    // PSDProjectionMethod::NONE: the TRUE Hessian, projected (or not) by the caller. The
    // smoothing energy squares the residual and takes its own Gauss-Newton approximation, which
    // is a different and better-motivated way to reach a PSD matrix than clamping this one. ESP
    // could not be projected per-term in any case -- its -1 weights make that invalid, which is
    // why upstream's own projection block in quadrature_potential.cpp is commented out.
    MatD H_total = MatD::Zero();
    if constexpr (DIM == 3) {
        if (m_impl->esp) H_total += m_impl->esp->hessian(m_impl->V_complex, esp_query(p));
    }
    auto& s = m_impl->scratch.local();
    const auto dict = m_impl->collisions(p, s);
    if (!dict) {
        return H_total;
    }
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
    return H_total + H.template block<DIM, DIM>(DIM * li, DIM * li);
}


template <int DIM>
std::string SmoothOffsetPotential<DIM>::describe_active(const VecD& p) const
{
    std::string out;
    if constexpr (DIM == 3) {
        // ESP builds its collision dict inside ArbitraryPointPotential and does not hand it
        // back, so the per-pair breakdown the OGC part prints is not available for the surface.
        // Report the field instead: what a discontinuity investigation reads off this is whether
        // Phi or |grad Phi| jumps between neighbouring samples, and both are here.
        if (m_impl->esp) {
            const auto [v, g, h] = m_impl->esp->evaluate(m_impl->V_complex, esp_query(p));
            out += fmt::format(
                "[ESP surface Phi={:.6g} |grad|={:.6g} tr(H)={:.6g}] ",
                v,
                g.norm(),
                h.trace());
        }
    }
    auto& s = m_impl->scratch.local();
    const auto dict = m_impl->collisions(p, s);
    if (!dict) {
        return out.empty() ? "<nothing active>" : out;
    }
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


AlignEnergy2D::AlignEnergy2D(
    const std::shared_ptr<const OffsetPotential2D>& potential,
    std::vector<Edge> edges,
    const double outward_sign,
    const double weight)
    : m_potential(potential)
    , m_edges(std::move(edges))
    , m_sign(outward_sign)
    , m_weight(weight)
{}

void AlignEnergy2D::residual(const Eigen::Vector2d& x, const Edge& e, double& r, Eigen::Vector2d& J)
    const
{
    r = 0.;
    J.setZero();
    // The edge's outward unit normal and its derivative. t = (q - x) / |q - x|,
    // dt/dx = -(I - t t^T) / |q - x|; n = sigma R90 t, R90 = [[0, -1], [1, 0]].
    const Eigen::Vector2d d = e.q - x;
    const double len = d.norm();
    if (!(len > 0.)) return;
    const Eigen::Vector2d t = d / len;
    const Eigen::Matrix2d R90 = (Eigen::Matrix2d() << 0., -1., 1., 0.).finished();
    const Eigen::Vector2d n = e.sigma * (R90 * t);
    const Eigen::Matrix2d dn_dx =
        -e.sigma * R90 * (Eigen::Matrix2d::Identity() - t * t.transpose()) / len;
    // The field's outward unit direction at the midpoint and its derivative:
    // m = (x + q) / 2, u = grad Phi / |grad Phi|, du/dm = (I - u u^T) H / |grad Phi|, dm/dx = 1/2.
    const Eigen::Vector2d m = 0.5 * (x + e.q);
    const Eigen::Vector2d g = m_potential->gradient(m);
    const double gn = g.norm();
    if (!std::isfinite(gn) || gn <= 1e-300) return;
    const Eigen::Vector2d u = g / gn;
    const Eigen::Matrix2d H = m_potential->hessian(m);
    if (!H.allFinite()) return;
    const Eigen::Vector2d ghat = m_sign * u;
    const Eigen::Matrix2d dghat_dx =
        0.5 * m_sign * (Eigen::Matrix2d::Identity() - u * u.transpose()) * H / gn;
    r = 1. - n.dot(ghat);
    // d r / dx = -(dn/dx)^T ghat - (dghat/dx)^T n
    J = -(dn_dx.transpose() * ghat) - (dghat_dx.transpose() * n);
}

double AlignEnergy2D::value(const TVector& x)
{
    double E = 0., r;
    Eigen::Vector2d J;
    for (const Edge& e : m_edges) {
        residual(x.head(2), e, r, J);
        E += m_weight * r * r;
    }
    return E;
}

void AlignEnergy2D::gradient(const TVector& x, TVector& gradv)
{
    Eigen::Vector2d G = Eigen::Vector2d::Zero(), J;
    double r;
    for (const Edge& e : m_edges) {
        residual(x.head(2), e, r, J);
        G += 2. * m_weight * r * J;
    }
    gradv = G;
}

void AlignEnergy2D::hessian(const TVector& x, MatrixXd& hessian)
{
    Eigen::Matrix2d Hs = Eigen::Matrix2d::Zero();
    Eigen::Vector2d J;
    double r;
    for (const Edge& e : m_edges) {
        residual(x.head(2), e, r, J);
        Hs += 2. * m_weight * J * J.transpose();
    }
    hessian = Hs;
}

template <int DIM>
double OffsetEnergy<DIM>::value(const TVector& x)
{
    // NORMALISED BY THE LEVEL (Uday, 2026-08-25): r = (Phi - c) / c, so the term is O(1) for
    // every field and every target_distance. The Euclidean field has c = 1 and is unchanged; the
    // smooth potential's c ranges 0.19-2.4 across the two_circles deltas, which scaled this term
    // by up to 6x against AMIPS and the alignment term. Gradient and Hessian below carry 1/c.
    const double c = std::max(m_potential->target_level(), 1e-300);
    const double r = (m_potential->value(VecD(x.head(DIM))) - c) / c;
    return m_weight * r * r;
}


template <int DIM>
void OffsetEnergy<DIM>::gradient(const TVector& x, TVector& gradv)
{
    const VecD p = x.head(DIM);
    const double c = std::max(m_potential->target_level(), 1e-300);
    const double r = (m_potential->value(p) - c) / c;
    gradv = 2. * m_weight * r * m_potential->gradient(p) / c;
}


template <int DIM>
void OffsetEnergy<DIM>::hessian(const TVector& x, MatrixXd& hessian)
{
    const VecD p = x.head(DIM);
    const double c = std::max(m_potential->target_level(), 1e-300);
    const VecD g = m_potential->gradient(p) / c;
    MatD H = 2. * m_weight * g * g.transpose();
    if (!m_gauss_newton) {
        const double r = (m_potential->value(p) - c) / c;
        H += 2. * m_weight * r * m_potential->hessian(p) / c;
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
    // IN UNITS OF target_distance: value = d / delta, level c = 1, so |grad| = 1 / delta.
    // The raw distance made the placement's pull, 2 (d - delta) |grad d|, ~1/delta^2 weaker
    // than the smooth potential's at the same misplacement (measured on two_circles at 0.1:
    // 0.023 against the smooth field's ~86), so the 1e-4 AMIPS term was no longer negligible
    // and construction zigzags 12% off the offset survived as exact balance points. Every
    // consumer works in ratios of value to c or divides by level_set_slope(), so nothing else
    // changes: the dist_and_orient distance |value - c| / |grad| is still d - delta.
    m_c = 1.;
    m_grad_ref = 1. / delta;
}

template <int DIM>
EuclideanOffsetPotential<DIM>::EuclideanOffsetPotential(
    const std::shared_ptr<SimplicialComplexBVH>& bvh,
    const double delta)
    : OffsetPotential<DIM>(delta, std::numeric_limits<double>::infinity())
    , m_bvh(bvh)
{
    if constexpr (DIM != 2) {
        // The BVH's feature query is 2D; 3D still runs on its input-complex envelope. A runtime
        // check because the explicit instantiations below compile every member for both DIMs.
        log_and_throw_error("EuclideanOffsetPotential<3>: the BVH-backed path is 2D-only");
    }
    if (!(delta > 0.)) {
        log_and_throw_error(
            "EuclideanOffsetPotential: target_distance must be positive, got {}",
            delta);
    }
    if (!m_bvh) {
        log_and_throw_error("EuclideanOffsetPotential: needs a BVH to query");
    }
    // IN UNITS OF target_distance: value = d / delta, level c = 1, so |grad| = 1 / delta.
    // The raw distance made the placement's pull, 2 (d - delta) |grad d|, ~1/delta^2 weaker
    // than the smooth potential's at the same misplacement (measured on two_circles at 0.1:
    // 0.023 against the smooth field's ~86), so the 1e-4 AMIPS term was no longer negligible
    // and construction zigzags 12% off the offset survived as exact balance points. Every
    // consumer works in ratios of value to c or divides by level_set_slope(), so nothing else
    // changes: the dist_and_orient distance |value - c| / |grad| is still d - delta.
    m_c = 1.;
    m_grad_ref = 1. / delta;
}

template <int DIM>
void EuclideanOffsetPotential<DIM>::nearest_feature(const VecD& p, VecD& foot, int& dim, VecD& dir)
    const
{
    if constexpr (DIM == 2) {
        bool on_corner = false;
        int feature_id = -1;
        Eigen::Vector2d seg_normal;
        // Same query, either engine; the algorithm and its results are identical (the BVH's
        // copy of it was lifted verbatim from the envelope's).
        if (m_bvh) {
            m_bvh->nearest_point_feature(p, foot, on_corner, seg_normal, feature_id);
        } else {
            m_envelope->nearest_point_feature(p, foot, on_corner, seg_normal, feature_id);
        }
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
    if constexpr (DIM == 2) {
        if (m_bvh) {
            // THE CURVE'S distance, through the feature query, exactly as the envelope-backed
            // path measured it -- NOT squared_dist(), which is the distance to the SOLID
            // complex and is identically zero inside a solid region. The two agree everywhere
            // the offset lives (outside), but value() should not silently change meaning
            // inside.
            Eigen::Vector2d foot;
            bool on_corner = false;
            Eigen::Vector2d seg_normal;
            int feature_id = -1;
            return std::sqrt(
                       m_bvh->nearest_point_feature(p, foot, on_corner, seg_normal, feature_id)) /
                   m_delta;
        }
    }
    return std::sqrt(m_envelope->squared_distance(p)) / m_delta;
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
    // term anyway (smooth_before refuses an input-complex vertex in Phase B) and from the criterion
    // (band_vertex_is_reachable books it as pinned), so this is a belt-and-braces case.
    if (!(d > 1e-14)) {
        return VecD::Zero();
    }
    return r / (d * m_delta);
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
        return (MatD::Identity() - dir * dir.transpose() - u * u.transpose()) / (d * m_delta);
    }
    // Vertex: distance to a point, curved in every direction but radially.
    return (MatD::Identity() - u * u.transpose()) / (d * m_delta);
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
