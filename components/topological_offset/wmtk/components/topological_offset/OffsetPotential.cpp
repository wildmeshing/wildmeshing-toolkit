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
/// Unread on this path -- Phi is a point evaluation against a cached collision set, with no
/// quadrature in it -- but HighOrderContactParameters requires an order and warns about 1.
constexpr int UNUSED_QUAD_ORDER = 2;

/// Also unread by the vertex path (it feeds the near/far barrier split). Upstream's default.
constexpr double UNUSED_DBAR_FACTOR = 1.0;

/// The query point in the row form ipc::ArbitraryPointPotential takes.
template <int DIM>
inline Eigen::RowVector<double, DIM> esp_query(const Eigen::Matrix<double, DIM, 1>& p)
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
 * The collision mesh carries the complex's vertices plus one extra row for the query point, the
 * pattern the upstream API is written around. Its topology never changes, so it is built once; an
 * evaluation only writes the last row and rebuilds the small collision set around it.
 */
template <int DIM>
struct SmoothOffsetPotential<DIM>::Impl
{
    ipc::CollisionMesh mesh; ///< complex vertices + the query vertex
    ipc::HighOrderContactParameters params;
    size_t n_complex_v = 0; ///< index of the query vertex, one past the complex's own

    /// Broad phase. ipc's own Candidates::build pairs a whole mesh's primitives with each other,
    /// which does not suit one moving query point against a fixed complex, so this is one box
    /// query per evaluation against the complex as loaded. 3D indexes the triangles and derives
    /// their edges and vertices from a hit, which is complete because a primitive within dhat of q
    /// lies inside the AABB of every triangle containing it. Segments in no triangle and isolated
    /// points are reachable only through their own trees.
    SimpleBVH::BVH face_bvh; ///< 3D only: the complex's triangles
    bool has_faces = false;
    SimpleBVH::BVH edge_bvh; ///< 2D: every segment. 3D: only the segments in no triangle.
    bool has_edges = false;
    std::vector<int> edge_ids; ///< edge_bvh row -> edge id in `mesh`
    SimpleBVH::BVH point_bvh; ///< isolated complex vertices, as degenerate segments
    bool has_points = false;
    std::vector<int> isolated; ///< complex vertex ids with no incident segment

    /// 3D only: the ESP evaluator, which owns its own broad phase and feature classification.
    ///
    /// `V_complex` is kept because every call takes the vertex configuration as an argument, and it
    /// must contain the complex and nothing else: ArbitraryPointBVH indexes every row of it as a
    /// vertex primitive, so a padding row would be a real point of the complex at whatever
    /// coordinates it happened to hold. `mesh_esp` is the 2-manifold sub-complex, compacted and
    /// padded -- not `mesh` above, which the OGC part needs in the complex's own indexing.
    ipc::CollisionMesh mesh_esp;
    Eigen::MatrixXd V_complex;
    std::unique_ptr<ipc::ArbitraryPointPotential<DIM>> esp; ///< null when nothing feeds ESP

    /// Per-thread evaluation state. The smoothing pass is parallel, so `value`, `gradient` and
    /// `hessian` must not share the query row or the candidate sets.
    struct Scratch
    {
        Eigen::MatrixXd V;
        ipc::Candidates candidates;
        bool initialized = false;
    };
    mutable wmtk::threading::enumerable_thread_specific<Scratch> scratch;

    /// ogc_collisions records which builder this dimension uses; neither path reads it, since 2D
    /// calls build_collisions_at_vertex_ogc_2d() by name and ArbitraryPointPotential ignores it.
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
            // The 3D builder reads vf_set, ve_set and vv_set independently, so a triangle's edges
            // and vertices must be seeded here too. Missing one fails silently: a convex feature
            // whose vertex is never a candidate contributes nothing, leaving the level set with a
            // hole where it should have a spherical cap.
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
                        // 2D gets its vertex candidates for free -- Candidates::vv_set() derives
                        // them from the endpoints of the edge candidates, which is what makes a
                        // convex corner work, since there only the vertex claims the point. 3D
                        // has no such derivation.
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

        // Upstream decides which candidates actually contribute -- the OGC feasible-region rule,
        // the interiority tests and the dhat cut. Deliberately not reimplemented here.
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
        // At dhat_factor == 1 the offset sits exactly on the support boundary, where Phi and its
        // gradient are both zero: a vertex there gets no direction to move in and the level set
        // Phi = c does not exist. Below 1 there is no level set at all.
        log_and_throw_error(
            "OffsetPotential: offset_dhat_factor must be > 1 (the offset distance has to lie "
            "strictly inside the potential's support), got {}",
            dhat_factor);
    }

    build(V, E, F, P);

    // Calibration runs through this same code path rather than a closed form: Phi at perpendicular
    // distance delta from one large flat primitive, i.e. the level the offset takes on any flat
    // stretch of the input.
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
    // One primitive large enough that the probe at perpendicular distance delta projects into its
    // interior and all of its boundary features lie outside the support, so exactly one pair is
    // active -- the definition of a flat stretch of input.
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

    // ---- 3D: ESP over the 2-manifold part of the complex ----
    //
    // ESP's alternating sum (+faces -edges +vertices) is inclusion-exclusion over a closed surface:
    // each -1 edge term cancels the +1 its incident faces contribute when the closest point lands
    // on that shared edge, netting exactly one +b(d) at the true closest feature. A primitive with
    // no incident face has nothing to cancel against, so the sum inverts to -b(d) -- a barrier with
    // the wrong sign, unbounded below as the query approaches it.
    //
    // The complex is therefore split by what ESP is defined on. Triangles, their edges and their
    // vertices go to ArbitraryPointPotential; segments in no triangle and isolated points stay on
    // the OGC vertex builder below, which weights every active primitive +1 and is right for a
    // sub-manifold piece. The two sums add.
    if constexpr (DIM == 3) {
        if (F.rows() > 0) {
            std::map<std::pair<int, int>, int> edge_of;
            for (int i = 0; i < E.rows(); ++i) {
                edge_of[{std::min(E(i, 0), E(i, 1)), std::max(E(i, 0), E(i, 1))}] = i;
            }
            // Compacted to the face-incident vertices: ipc::LBVH indexes every row it is given as a
            // vertex primitive, so carrying a wire or isolated vertex here would add a +1 term in
            // the surface sum on top of the one the OGC part already gives it.
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

            // No ESP tree may have exactly one leaf: ipc::LBVH sizes a tree as 2N-1 nodes, so with
            // N == 1 the root is the leaf, while ArbitraryPointBVH::query_point() starts at node 0
            // assuming it is inner and reads node.left / node.right, both of which resolve back to
            // the node itself. The lone primitive is then pushed twice and its term enters doubled.
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
            m_impl->esp = std::make_unique<ipc::ArbitraryPointPotential<DIM>>(
                m_impl->mesh_esp,
                m_impl->params);
            // Once: the complex is fixed for this potential's lifetime.
            m_impl->esp->update(m_impl->V_complex);
        }
    }

    // ---- 2D: ESP over the segments of the complex ----
    //
    // Same inclusion-exclusion, one codimension down: segments +1, their endpoints -1, so at a
    // corner the two segments' reductions and the direct vertex term cancel to exactly one b(d).
    // An isolated point has no incident segment to cancel against and would invert to -b(d), so
    // it stays on the OGC vertex builder below, exactly as a wire edge does in 3D.
    if constexpr (DIM == 2) {
        if (E.rows() > 0) {
            std::vector<int> to_seg(V.rows(), -1);
            std::vector<int> seg_v;
            const auto claim = [&](const int v) {
                if (to_seg[v] < 0) {
                    to_seg[v] = static_cast<int>(seg_v.size());
                    seg_v.push_back(v);
                }
                return to_seg[v];
            };
            MatrixXi E_s(E.rows(), 2);
            for (int i = 0; i < E.rows(); ++i) {
                E_s(i, 0) = claim(E(i, 0));
                E_s(i, 1) = claim(E(i, 1));
            }
            // No ESP tree may have exactly one leaf -- see the 3D note above. One far segment
            // takes the edge tree to N >= 2 (the vertex tree already is, a segment having two
            // ends), and being beyond dhat it never enters a sum.
            const int n_sv = static_cast<int>(seg_v.size());
            const Eigen::RowVector2d far = V.colwise().maxCoeff().array() + 8. * m_dhat + 1.;
            MatrixXd V_s(n_sv + 2, 2);
            for (int i = 0; i < n_sv; ++i) V_s.row(i) = V.row(seg_v[i]);
            V_s.row(n_sv) = far;
            V_s.row(n_sv + 1) = far + Eigen::RowVector2d(m_dhat, 0.);
            MatrixXi E_pad(E.rows() + 1, 2);
            E_pad.topRows(E.rows()) = E_s;
            E_pad.row(E.rows()) << n_sv, n_sv + 1;

            m_impl->V_complex = V_s;
            m_impl->mesh_esp = ipc::CollisionMesh(V_s, E_pad, MatrixXi(0, 3));
            m_impl->esp = std::make_unique<ipc::ArbitraryPointPotential<DIM>>(
                m_impl->mesh_esp,
                m_impl->params);
            m_impl->esp->update(m_impl->V_complex);
        }
    }

    // ---- 2D: the OGC vertex builder, which needs a real mesh vertex to measure from ----
    // The collision mesh is the complex plus one trailing row that every evaluation overwrites with
    // the query point. That row belongs to no segment and no triangle, so it never appears as a
    // contact primitive, only as the point the primitives are measured from.
    //
    // A complex with no segments at all needs one row more. ipc::CollisionMesh's
    // are_adjacencies_initialized() requires all three adjacency tables to be non-empty and sizes
    // the edge-vertex table by the edge count, so with no edges every accessor throws, including
    // the one the OGC feasible-region test calls. One sentinel segment far outside the complex
    // makes the table non-empty, and it is unreachable because the only source of candidates is
    // our own BVHs below, which do not index it. No sentinel triangle is needed: a complex with
    // triangles necessarily has their edges.
    // In 2D every segment went to ESP, so the OGC mesh deliberately carries none: a vertex with no
    // incident segment passes the feasible-region test from every direction, which is what both an
    // isolated point and an open end need. In 3D the mesh keeps the wire edges the OGC part owns.
    const bool needs_sentinel = (DIM == 2) || (E.rows() == 0);
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

    // 2D only. In 3D the triangles, and their edges and vertices, belong to the ESP sum above, so
    // seeding them as OGC candidates as well would count the surface twice.
    if constexpr (DIM == 2) {
        (void)V3;
    } else {
        // has_faces stays false: nothing here indexes F.
    }

    // What the OGC part owns: in both dimensions only what ESP cannot take -- in 3D the segments
    // in no triangle, in 2D nothing at all (every segment went to ESP; the isolated points below
    // are its whole remit).
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

    // ESP's vertex term is -1 for every complex vertex, cancelled by the +1 its incident segments
    // contribute when their closest point to the query lands on it. A vertex with exactly ONE
    // incident segment has only one +1 to cancel with, so its -1 survives everywhere inside the
    // support -- not merely beyond the end -- and Phi comes out short by exactly b(dist to it),
    // reaching 0 where the end should be the closest feature. Adding that vertex to the OGC point
    // set restores the missing +1 at every query point, which is exact rather than a cap.
    // (Degree >= 3 would over-count by the same argument; no complex here has one, and a junction
    // is not a manifold boundary, so it is left to fail loudly rather than be silently patched.)
    std::vector<int> pts = P;
    if constexpr (DIM == 2) {
        std::map<int, int> degree;
        for (int i = 0; i < E.rows(); ++i) {
            ++degree[E(i, 0)];
            ++degree[E(i, 1)];
        }
        for (const auto& [v, d] : degree) {
            if (d == 1) pts.push_back(v);
        }
    }
    m_impl->isolated = pts;
    if (!pts.empty()) {
        // As pseudo-segments (p, p), the same encoding SimplicialComplexBVH uses for the
        // isolated vertices of the complex.
        MatrixXi PE(pts.size(), 2);
        for (size_t i = 0; i < pts.size(); ++i) {
            PE(i, 0) = pts[i];
            PE(i, 1) = pts[i];
        }
        m_impl->point_bvh.init(V3, PE, 1e-6);
        m_impl->has_points = true;
    }
}

template <int DIM>
double SmoothOffsetPotential<DIM>::value(const VecD& p) const
{
    // The two sums add: ESP over the triangles, the OGC vertex builder over the segments in no
    // triangle and the isolated points. Either may be empty -- a closed surface uses only the
    // first, a point cloud or a 2D complex only the second. See build() for the split.
    double phi = 0.;
    if (m_impl->esp) phi += (*m_impl->esp)(m_impl->V_complex, esp_query<DIM>(p));
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
    if (m_impl->esp) g_total += m_impl->esp->gradient(m_impl->V_complex, esp_query<DIM>(p));
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
    // PSDProjectionMethod::NONE returns the true Hessian, for the caller to project or not: the
    // smoothing energy squares the residual and takes its own Gauss-Newton approximation, which is
    // a better-motivated route to a PSD matrix than clamping this one. ESP could not be projected
    // per term in any case, because its -1 weights make that invalid.
    MatD H_total = MatD::Zero();
    if (m_impl->esp) H_total += m_impl->esp->hessian(m_impl->V_complex, esp_query<DIM>(p));
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
    {
        // ESP builds its collision dict inside ArbitraryPointPotential and does not hand it back,
        // so the per-pair breakdown the OGC part prints is not available for the surface; report
        // Phi and |grad Phi| instead, which is what a discontinuity investigation compares between
        // neighbouring samples.
        if (m_impl->esp) {
            const auto [v, g, h] = m_impl->esp->evaluate(m_impl->V_complex, esp_query<DIM>(p));
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
    // Divide by the reference slope -- the slope of Phi at the level set on a flat stretch -- and
    // never by the local |grad Phi(p)|: as p approaches the complex, Phi ~ -log(d) and
    // |grad Phi| ~ 1/d, so the local ratio tends to 0 and a vertex sitting on the complex would
    // report a residual of zero and be called converged. The fixed slope keeps the quantity
    // monotone in Phi, hence in distance wherever one pair is active; it saturates outside the
    // support, which the runaway guard turns into a hard error before this number decides anything.
    return std::abs(value(p) - m_c) / m_grad_ref;
}


// ---------------------------------------------------------------------------------------------


template <int DIM>
OffsetEnergy<DIM>::OffsetEnergy(
    const std::shared_ptr<const OffsetPotential<DIM>>& potential,
    const double weight,
    const bool gauss_newton,
    const bool distance_residual)
    : m_potential(potential)
    , m_weight(weight)
    , m_gauss_newton(gauss_newton)
    , m_distance_residual(distance_residual)
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
        E += m_weight * e.agree * r * r;
    }
    return E;
}

void AlignEnergy2D::gradient(const TVector& x, TVector& gradv)
{
    Eigen::Vector2d G = Eigen::Vector2d::Zero(), J;
    double r;
    for (const Edge& e : m_edges) {
        residual(x.head(2), e, r, J);
        G += 2. * m_weight * e.agree * r * J;
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
        Hs += 2. * m_weight * e.agree * J * J.transpose();
    }
    hessian = Hs;
}

template <int DIM>
bool OffsetEnergy<DIM>::root_distance(const VecD& p, double& s, VecD& n) const
{
    // The signed distance from p to the level set Phi = c along the field's normal at p: the root
    // of g(t) = Phi(p - t n) - c, t > 0 outward. Safeguarded Newton -- the step is Newton's while
    // it stays inside the bracket and a bisection of the bracket otherwise, so it converges in a
    // handful of evaluations where Phi is smooth and cannot escape. Evaluation count matters: the
    // residual is asked for three times per Newton iteration of the placement.
    const double c = m_potential->target_level();
    const VecD g0 = m_potential->gradient(p);
    const double gn0 = g0.norm();
    if (!std::isfinite(gn0) || !(gn0 > 0.)) return false;
    n = g0 / gn0; // toward the input: Phi grows that way
    const double delta = std::max(m_potential->delta(), 1e-300);
    const double reach = 2. * std::max(m_potential->dhat(), delta);
    const double tol = 1e-8 * delta;
    const auto f = [&](const double t) { return m_potential->value(p - t * n) - c; };
    const auto fprime = [&](const double t) { return -m_potential->gradient(p - t * n).dot(n); };
    // Warm start from this vertex's previous root: the placement moves the vertex toward the level
    // set, so the next call's root is near the last one minus the step taken. Plain Newton from
    // there converges in a few evaluations, and the bracket below is only built when it does not.
    double t = std::isfinite(m_last_root) ? m_last_root : 0., ft = f(t);
    if (!std::isfinite(ft)) {
        t = 0.;
        ft = f(0.);
        if (!std::isfinite(ft)) return false;
    }
    if (ft == 0.) {
        s = t;
        m_last_root = t;
        return true;
    }
    for (int it = 0; it < 8; ++it) {
        const double fp = fprime(t);
        if (!std::isfinite(fp) || fp == 0.) break;
        const double tn = t - ft / fp;
        if (!std::isfinite(tn) || std::abs(tn) > reach) break;
        const double ftn = f(tn);
        if (!std::isfinite(ftn) || std::abs(ftn) > std::abs(ft)) break; // not converging: bracket
        const bool done = std::abs(tn - t) < tol || ftn == 0.;
        t = tn;
        ft = ftn;
        if (done) {
            s = t;
            m_last_root = t;
            return true;
        }
    }
    // Bracket [lo, hi] with f(lo) > 0 (too close, inside the level set) and f(hi) < 0.
    t = 0.;
    ft = f(0.);
    if (!std::isfinite(ft)) return false;
    double lo, hi;
    if (ft > 0.) {
        lo = 0.;
        hi = 0.125 * delta;
        while (hi < reach && f(hi) > 0.) hi *= 2.;
        if (!(f(hi) < 0.)) return false;
    } else {
        hi = 0.;
        lo = -0.125 * delta;
        while (lo > -reach && f(lo) < 0.) lo *= 2.;
        if (!(f(lo) > 0.)) return false;
    }
    for (int it = 0; it < 60; ++it) {
        const double fp = fprime(t);
        double tn = (std::isfinite(fp) && fp != 0.) ? t - ft / fp : 0.5 * (lo + hi);
        if (!(tn > lo && tn < hi)) tn = 0.5 * (lo + hi); // Newton left the bracket: bisect
        const double ftn = f(tn);
        if (!std::isfinite(ftn)) return false;
        if (ftn > 0.) {
            lo = tn;
        } else {
            hi = tn;
        }
        const bool done = std::abs(tn - t) < tol || ftn == 0. || (hi - lo) < tol;
        t = tn;
        ft = ftn;
        if (done) break;
    }
    s = t;
    m_last_root = t;
    return true;
}

template <int DIM>
void OffsetEnergy<DIM>::residual(const VecD& p, double& r, VecD& dr) const
{
    const double c = std::max(m_potential->target_level(), 1e-300);
    if (m_distance_residual && !m_potential->is_euclidean()) {
        const double delta = std::max(m_potential->delta(), 1e-300);
        if (m_cache_valid && (p - m_cache_p).squaredNorm() == 0.) {
            r = m_cache_r;
            dr = m_cache_dr;
            return;
        }
        double s;
        VecD n;
        if (root_distance(p, s, n)) {
            // r = -s/delta: negative when p is too close, like (d - delta)/delta. Moving p
            // outward by t shrinks s by t, so dr/dp = (outward unit)/delta = -n/delta.
            r = -s / delta;
            dr = -n / delta;
        } else {
            // No root within reach (outside the support, or a vanishing gradient): the
            // monotone length (Phi - c)/grad_ref, still in units of delta.
            const double g_ref = std::max(m_potential->level_set_slope(), 1e-300);
            r = (m_potential->value(p) - c) / (g_ref * delta);
            dr = m_potential->gradient(p) / (g_ref * delta);
        }
        m_cache_p = p;
        m_cache_r = r;
        m_cache_dr = dr;
        m_cache_valid = true;
        return;
    }
    // Normalised by the level: r = (Phi - c) / c, so the term is O(1) for every field and every
    // target_distance at the level set. For the Euclidean field this is exactly (d - delta)/delta,
    // and it stays the Euclidean residual under either flag.
    r = (m_potential->value(p) - c) / c;
    dr = m_potential->gradient(p) / c;
}

template <int DIM>
double OffsetEnergy<DIM>::value(const TVector& x)
{
    double r;
    VecD dr;
    residual(VecD(x.head(DIM)), r, dr);
    return m_weight * r * r;
}

template <int DIM>
void OffsetEnergy<DIM>::gradient(const TVector& x, TVector& gradv)
{
    double r;
    VecD dr;
    residual(VecD(x.head(DIM)), r, dr);
    gradv = 2. * m_weight * r * dr;
}

template <int DIM>
void OffsetEnergy<DIM>::hessian(const TVector& x, MatrixXd& hessian)
{
    const VecD p = x.head(DIM);
    double r;
    VecD dr;
    residual(p, r, dr);
    MatD H = 2. * m_weight * dr * dr.transpose();
    if (!m_gauss_newton && !(m_distance_residual && !m_potential->is_euclidean())) {
        const double c = std::max(m_potential->target_level(), 1e-300);
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
    // No support limit: d is defined everywhere, so the runaway guard that exists for Phi's compact
    // support has nothing to catch. Infinity says so, rather than a large finite number something
    // might later compare against.
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
    // No calibration: the level is the offset distance, where the smooth potential has to discover
    // its own level by evaluating Phi at distance delta from a flat reference.
    //
    // In units of target_distance: value = d / delta, level c = 1, so |grad| = 1 / delta. The raw
    // distance made the placement's pull ~1/delta^2 weaker than the smooth potential's at the same
    // misplacement, weak enough for the small AMIPS term to hold a misplaced vertex at a balance
    // point. Every consumer works in ratios of value to c or divides by level_set_slope(), so
    // |value - c| / |grad| is still d - delta.
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
    // In units of target_distance: value = d / delta, level c = 1, so |grad| = 1 / delta. See the
    // envelope-backed constructor above for why the raw distance is not used.
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
        // Same query, either engine: the two implementations of it are identical.
        if (m_bvh) {
            m_bvh->nearest_point_feature(p, foot, on_corner, seg_normal, feature_id);
        } else {
            m_envelope->nearest_point_feature(p, foot, on_corner, seg_normal, feature_id);
        }
        // The 2D query reports the segment normal, while the Hessian below is cased on the
        // direction along the feature: a segment interior is dim 1 with the tangent, obtained by
        // rotating the normal a quarter turn.
        dim = on_corner ? 0 : 1;
        dir = on_corner ? VecD::Zero().eval() : VecD(-seg_normal.y(), seg_normal.x());
    } else {
        long long feature_id = -1;
        m_envelope->nearest_point_feature(p, foot, dim, dir, feature_id);
    }

    // A degenerate segment is a point, not an edge: both SimplicialComplexBVH and the envelope
    // carry an isolated input vertex as the pseudo-edge (i, i), so a query near one comes back as
    // an edge-interior hit whose direction is whatever normalising a zero vector produced, and the
    // edge Hessian would subtract a meaningless t t^T. Demote it to the vertex case.
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
            // The distance to the complex's curve, through the feature query -- never
            // squared_dist(), which measures the solid complex and is identically zero inside a
            // solid region. The two agree everywhere the offset lives, but value() must not
            // silently change meaning inside.
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
    // On the complex the gradient of d does not exist, since every direction increases it equally.
    // Zero contributes no offset force, so such a vertex is moved by the quality term alone. Phase
    // B excludes input-complex vertices from the offset term and the criterion books them as
    // pinned, so this case is a backstop.
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
        // Face interior in 3D, segment interior in 2D: d is linear in p there, so no curvature.
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

RestAMIPSEnergy2D::RestAMIPSEnergy2D(std::vector<Cell> cells, const double weight)
    : m_cells(std::move(cells))
    , m_weight(weight)
{}

bool RestAMIPSEnergy2D::cell_F(
    const Eigen::Vector2d& x,
    const Cell& c,
    Eigen::Matrix2d& F,
    double& d) const
{
    Eigen::Matrix2d A;
    A.col(0) = c.q1 - x;
    A.col(1) = c.q2 - x;
    F = A * c.rest_inv;
    d = F.determinant();
    return d > 0.;
}

double RestAMIPSEnergy2D::value(const TVector& x)
{
    double E = 0.;
    Eigen::Matrix2d F;
    double d;
    for (const Cell& c : m_cells) {
        if (!cell_F(x.head(2), c, F, d)) return std::nan("");
        E += m_weight * F.squaredNorm() / d;
    }
    return E;
}

void RestAMIPSEnergy2D::gradient(const TVector& x, TVector& gradv)
{
    // dE/dF = 2F/d - (e/d) F^-T, in closed form for 2x2; dF/dx_k = -e_k * r^T with
    // r = Rinv^T (1,1)^T, so grad_x = -(dE/dF) r summed over cells.
    Eigen::Vector2d G = Eigen::Vector2d::Zero();
    Eigen::Matrix2d F;
    double d;
    for (const Cell& c : m_cells) {
        if (!cell_F(x.head(2), c, F, d)) {
            gradv = Eigen::Vector2d::Zero(); // invalid point: the line search never accepts it
            return;
        }
        const double e = F.squaredNorm();
        Eigen::Matrix2d FinvT;
        FinvT << F(1, 1), -F(1, 0), -F(0, 1), F(0, 0);
        FinvT /= d;
        const Eigen::Matrix2d dEdF = 2. / d * F - (e / d) * FinvT;
        const Eigen::Vector2d r = c.rest_inv.transpose() * Eigen::Vector2d::Ones();
        G += -m_weight * (dEdF * r);
    }
    gradv = G;
}

void RestAMIPSEnergy2D::hessian(const TVector& x, MatrixXd& hessian)
{
    // F is affine in x, so H_x = M^T H_F M exactly, with M the constant 4x2 dvecF/dx
    // (column-major vec) and H_F the closed-form Hessian of e/d in F:
    //   dE = e'/d - e d'/d^2,  d2E = e''/d - (e' d'^T + d' e'^T)/d^2 - e d''/d^2
    //        + 2 e (d' d'^T)/d^3,
    // e' = 2 vecF, e'' = 2I, d' = (F11, -F01, -F10, F00), d'' = the constant K.
    Eigen::Matrix2d Hx = Eigen::Matrix2d::Zero();
    Eigen::Matrix2d F;
    double d;
    Eigen::Matrix4d K = Eigen::Matrix4d::Zero();
    K(0, 3) = K(3, 0) = 1.;
    K(1, 2) = K(2, 1) = -1.;
    for (const Cell& c : m_cells) {
        if (!cell_F(x.head(2), c, F, d)) continue; // invalid point: contribute nothing
        const double e = F.squaredNorm();
        Eigen::Vector4d vF(F(0, 0), F(1, 0), F(0, 1), F(1, 1));
        Eigen::Vector4d dd(F(1, 1), -F(0, 1), -F(1, 0), F(0, 0));
        const Eigen::Vector4d de = 2. * vF;
        Eigen::Matrix4d HF = (2. / d) * Eigen::Matrix4d::Identity();
        HF -= (de * dd.transpose() + dd * de.transpose()) / (d * d);
        HF += (2. * e / (d * d * d)) * (dd * dd.transpose());
        HF -= (e / (d * d)) * K;
        const Eigen::Vector2d r = c.rest_inv.transpose() * Eigen::Vector2d::Ones();
        Eigen::Matrix<double, 4, 2> M = Eigen::Matrix<double, 4, 2>::Zero();
        // dF(i,j)/dx_k = -Rinv row-sum of column j when k == i: vec index 2j + i.
        for (int j = 0; j < 2; ++j) {
            for (int i = 0; i < 2; ++i) {
                M(2 * j + i, i) = -r(j);
            }
        }
        Hx += m_weight * (M.transpose() * HF * M);
    }
    hessian = Hx;
}

bool RestAMIPSEnergy2D::is_step_valid(const TVector& /*x0*/, const TVector& x1)
{
    Eigen::Matrix2d F;
    double d;
    for (const Cell& c : m_cells) {
        if (!cell_F(x1.head(2), c, F, d)) return false;
    }
    return true;
}

} // namespace wmtk::components::topological_offset
