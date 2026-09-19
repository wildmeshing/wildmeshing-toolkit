#include <chrono>
#include <set>
#include <unordered_map>
#include "ExactSimplexDistance.hpp"
#include "TopoOffsetTetMesh.h"

namespace wmtk::components::topological_offset {

namespace {

/// A simplex of the input complex or a far simplex of a band cell, as geometry: 1, 2 or 3 points,
/// plus the bounding sphere that lets a pair be rejected without solving it.
struct DistSimplex
{
    int n = 0;
    Vector3d p[3];
    Vector3d c;
    double r = 0.;

    void set(const Vector3d* pts, const int count)
    {
        n = count;
        c.setZero();
        for (int i = 0; i < count; ++i) {
            p[i] = pts[i];
            c += pts[i];
        }
        c /= double(count);
        r = 0.;
        for (int i = 0; i < count; ++i) r = std::max(r, (pts[i] - c).norm());
    }
};

/// The exact closest distance between two simplices, dispatched on their dimensions. Squared, so
/// the caller takes one square root at the end.
double pair_sq(const DistSimplex& a, const DistSimplex& b)
{
    namespace esd = exact_simplex_distance;
    if (a.n > b.n) return pair_sq(b, a);
    if (a.n == 1 && b.n == 1) return esd::point_point_sq(a.p[0], b.p[0]);
    if (a.n == 1 && b.n == 2) return esd::point_segment_sq(a.p[0], b.p[0], b.p[1]);
    if (a.n == 1) return esd::point_triangle_sq(a.p[0], b.p[0], b.p[1], b.p[2]);
    if (a.n == 2 && b.n == 2) return esd::segment_segment_sq(a.p[0], a.p[1], b.p[0], b.p[1]);
    if (a.n == 2) return esd::segment_triangle_sq(a.p[0], a.p[1], b.p[0], b.p[1], b.p[2]);
    return esd::triangle_triangle_sq(a.p[0], a.p[1], a.p[2], b.p[0], b.p[1], b.p[2]);
}

/// d_g(p): the exact distance from a point to one simplex. Squared.
double point_simplex_sq(const Vector3d& p, const DistSimplex& g)
{
    namespace esd = exact_simplex_distance;
    if (g.n == 1) return esd::point_point_sq(p, g.p[0]);
    if (g.n == 2) return esd::point_segment_sq(p, g.p[0], g.p[1]);
    return esd::point_triangle_sq(p, g.p[0], g.p[1], g.p[2]);
}

/// The input complex as geometry, from the one extraction the whole run uses
/// (init_input_complex_bvh): its boundary triangles, its complete edge set and its isolated
/// vertices. Everything the refined march measures lies outside the complex, and outside it the
/// distance to the complex and the distance to that boundary are the same number, so d is the
/// smallest distance to one of these primitives -- each of which is a CONVEX set, which is what
/// the certified residual below rests on.
std::vector<DistSimplex> collect_input_simplices(
    const MatrixXd& phi_V,
    const MatrixXi& phi_E,
    const MatrixXi& phi_F,
    const std::vector<int>& phi_P)
{
    std::vector<DistSimplex> input;
    input.reserve(size_t(phi_F.rows() + phi_E.rows()) + phi_P.size());
    Vector3d pts[3];
    for (int i = 0; i < phi_F.rows(); ++i) {
        for (int k = 0; k < 3; ++k) pts[k] = phi_V.row(phi_F(i, k)).transpose();
        input.emplace_back().set(pts, 3);
    }
    for (int i = 0; i < phi_E.rows(); ++i) {
        for (int k = 0; k < 2; ++k) pts[k] = phi_V.row(phi_E(i, k)).transpose();
        input.emplace_back().set(pts, 2);
    }
    for (const int v : phi_P) {
        pts[0] = phi_V.row(v).transpose();
        input.emplace_back().set(pts, 1);
    }
    return input;
}

/**
 * @brief The certified residual of one front piece P: an upper bound on
 *        max over x in P of |d(x) - march_distance|, with no sampling and no constant.
 *
 * P is a simplex whose corners are roots, so d = march_distance at every corner, and
 * d(x) = min over the input primitives g of d_g(x) with every d_g CONVEX.
 *
 * INWARD, exact. max over P of (march_distance - d) = march_distance - min over P of d, and
 *   min over P of d = min over P min over g d_g = min over g min over P d_g = min over g dist(P, g)
 *                  = dist(P, input),
 * the closest distance between the piece and the complex -- a simplex-to-simplex distance, solved
 * in closed form (ExactSimplexDistance.hpp). So the inward part is march_distance - dist(P, input)
 * exactly, never a sample.
 *
 * OUTWARD, a rigorous bound. For any single primitive g, d <= d_g everywhere, and d_g is convex on
 * the simplex P, so its maximum over P is at a CORNER of P. Hence
 *   max over P of (d - march_distance) <= (max over corners of P of d_g) - march_distance
 * for every g, and the smallest such bound may be taken. The consequence, which the measurements
 * confirm: if one primitive is nearest everywhere on P then d = d_g there, d_g = march_distance at
 * the corners and convexity gives d <= march_distance on all of P, so this term is ZERO. It can
 * only bind where the nearest primitive changes across the piece -- which, for a solid whose
 * exterior distance is convex, never happens, and at a concave feature of the input costs a factor
 * of at most two.
 *
 * Both terms vanish as the piece shrinks onto a root, so driving the residual under tol terminates.
 *
 * THE CANDIDATE SET, and the invariant that makes the outward bound valid. The outward term is a
 * minimum over a set of candidate primitives, and the upper bound R(P) <= diam(P) -- what makes
 * shrinking a piece drive its residual to zero, hence what makes the loop terminate -- needs the
 * candidate set to contain a primitive NEAREST TO AT LEAST ONE CORNER of P. (With g0 nearest to
 * corner c0 we have d_g0(c0) = march_distance, so d_g0 <= march_distance + diam(P) at every corner
 * and the outward term is at most diam(P).) So the candidate set is the union of two filters:
 *  - the bounding-sphere shell around the piece, of radius piece.r + u with u = d(centroid): the
 *    cheap bulk filter, which keeps every primitive that can hold the exact minimum;
 *  - one ball per CORNER, of radius d(corner) around that corner, which keeps the primitive nearest
 *    to that corner: that primitive has a point at distance exactly d(corner) from the corner, so
 *    its bounding sphere meets the ball. d(corner) is one BVH point query per corner.
 * The shell alone does NOT guarantee the corner-nearest primitive: it is centred on the piece and
 * sized by the distance at the CENTROID, and a corner may be piece.r farther from the complex than
 * the centroid is, so the shell can be up to the piece's radius too small. The corner balls are an
 * addition, never a replacement.
 *
 * Cost. u = d(centroid) is an upper bound on dist(P, input), so a primitive farther from the piece
 * than u cannot hold the minimum and cannot be the one nearest at the centroid: the bounding-sphere
 * shell of radius u rejects all but a few tens of primitives without solving anything. Inside the
 * candidate set, d_g at the corners (cheap, point-to-simplex) both gives the outward term and seeds
 * the pruning of the expensive simplex-to-simplex solves, of which a handful per piece are left.
 * The corner balls cost one extra BVH point query per corner and widen the set a little. Measured
 * at distance_fraction 0.5, tol_rel 0.01, sphere_trace_target_rel_tol 1e-6, against the same runs
 * without them: every count is unchanged -- same bisections, same piece tests, same fraction with
 * a zero outward term, and the same 10.2 (presmooth3d/cylinder) and 8.3 (Kuhn cube) exact
 * simplex-to-simplex solves per test, because the exact solves are pruned by the Lipschitz bound
 * and by the running minimum, neither of which the wider set changes. What rises is only the
 * residual's own share of the refinement loop, from 34% to 48% on the cylinder and from 24% to 35%
 * on the Kuhn cube -- and that loop is 0.4 s of a 40 s run. So the corner balls are a cost of a
 * few tenths of a second and, on these inputs, they changed no result: the shell already happened
 * to keep a corner-nearest primitive everywhere. They are here because nothing guarantees that.
 */
struct Residual
{
    double value = 0.; ///< the certified residual: what the loop tests against tol
    bool outward_zero = false; ///< the outward term is 0: one primitive is nearest on all of P
    bool outward_binds = false; ///< the outward term, not the exact inward one, is the larger
    size_t exact_solves = 0; ///< simplex-to-simplex solves the shell and the pruning left
};

Residual certified_residual(
    const std::vector<DistSimplex>& input,
    const Vector3d* corner,
    const double* corner_dist,
    const int n_corner,
    const Vector3d& centroid,
    const double u,
    const double march_distance)
{
    DistSimplex piece;
    piece.set(corner, n_corner);
    const double in_shell = piece.r + u;

    // The candidate set: the piece's bounding-sphere shell, plus the ball of radius d(corner)
    // around every corner. The corner balls are what keep a primitive nearest to a corner, which
    // is the invariant the outward bound rests on; see the derivation above.
    const auto candidate = [&](const DistSimplex& g) {
        if ((piece.c - g.c).norm() <= in_shell + g.r) return true;
        for (int i = 0; i < n_corner; ++i) {
            if ((corner[i] - g.c).norm() <= corner_dist[i] + g.r) return true;
        }
        return false;
    };

    // best = min over the candidates of (min over corners of d_g) is an upper bound on
    // dist(P, input); outward = min over the candidates of (max over corners of d_g);
    // at_corner[i] = min over the candidates of d_g(corner i), for the invariant check below.
    double best = std::numeric_limits<double>::max();
    double outward = std::numeric_limits<double>::max();
    std::array<double, 3> at_corner;
    at_corner.fill(std::numeric_limits<double>::max());
    size_t n_shell = 0;
    for (const DistSimplex& g : input) {
        if (!candidate(g)) continue;
        ++n_shell;
        double lo = std::numeric_limits<double>::max(), hi = 0.;
        for (int i = 0; i < n_corner; ++i) {
            const double d = std::sqrt(point_simplex_sq(corner[i], g));
            lo = std::min(lo, d);
            hi = std::max(hi, d);
            at_corner[i] = std::min(at_corner[i], d);
        }
        best = std::min(best, lo);
        outward = std::min(outward, hi);
    }
    if (n_shell == 0) {
        // Unreachable: the primitive nearest to the centroid is at distance u from it, so it is
        // within piece.r + u of the piece's centre and always survives. If it fires, the primitive
        // set here and the BVH that computed u describe different geometry.
        log_and_throw_error(
            "certified_residual: no input primitive within {} of the piece centre, out of {} -- "
            "the primitive set and the distance query disagree",
            in_shell,
            input.size());
    }

    // The corner-nearest invariant, checked rather than assumed: for at least one corner i the
    // candidate set holds a primitive realizing d(corner i), i.e. at_corner[i] = d(corner i), so
    // the outward term is at most diam(P) and shrinking a piece drives its residual to zero. The
    // test is PER CORNER: comparing the pooled minimum `best` against the corner distances would
    // pass whenever any candidate is as near to any corner as the farthest corner is to the
    // complex, which holds almost always and proves nothing. The factor 1 + 1e-12 is not a
    // tolerance of the method: at_corner[i] and corner_dist[i] are the same Euclidean distance
    // evaluated by two code paths (point_simplex_sq here, the BVH there), and the factor absorbs
    // their different rounding; a genuinely missing primitive is off by far more.
    {
        bool corner_nearest = false;
        for (int i = 0; i < n_corner; ++i) {
            if (!(at_corner[i] > corner_dist[i] * (1. + 1e-12))) corner_nearest = true;
        }
        if (!corner_nearest) {
            log_and_throw_error(
                "certified_residual: no candidate primitive is nearest to any corner of the piece "
                "(nearest candidate distance at corner 0: {}, the BVH's distance there: {}) -- "
                "the outward bound is not valid",
                at_corner[0],
                corner_dist[0]);
        }
    }

    // The exact minimum. d_g is 1-Lipschitz, so dist(P, g) >= d_g(centroid) - piece.r: a primitive
    // whose lower bound already exceeds the running best cannot hold the minimum, and one rejected
    // by the shell has dist(P, g) > u >= dist(P, input). What is left is solved exactly.
    Residual out;
    for (const DistSimplex& g : input) {
        if ((piece.c - g.c).norm() > in_shell + g.r) continue;
        const double lower = std::sqrt(point_simplex_sq(centroid, g)) - piece.r;
        if (lower >= best) continue;
        ++out.exact_solves;
        best = std::min(best, std::sqrt(pair_sq(piece, g)));
    }

    const double inward_part = march_distance - best;
    const double outward_part = outward - march_distance;
    out.value = std::max(std::max(inward_part, outward_part), 0.);
    out.outward_zero = !(outward_part > 0.);
    out.outward_binds = outward_part > inward_part;
    return out;
}

/// The smallest dihedral angle of a tet, in degrees: for each of its six edges, the angle between
/// the two faces sharing it, measured between the two other vertices projected onto the plane
/// perpendicular to that edge.
double min_dihedral_degrees(const std::array<Vector3d, 4>& P)
{
    double worst = 180.;
    for (int i = 0; i < 4; ++i) {
        for (int j = i + 1; j < 4; ++j) {
            int o[2], no = 0;
            for (int z = 0; z < 4; ++z)
                if (z != i && z != j) o[no++] = z;
            Vector3d e = P[j] - P[i];
            const double len = e.norm();
            if (!(len > 0.)) continue;
            e /= len;
            const Vector3d u = (P[o[0]] - P[i]) - (P[o[0]] - P[i]).dot(e) * e;
            const Vector3d w = (P[o[1]] - P[i]) - (P[o[1]] - P[i]).dot(e) * e;
            const double nu = u.norm(), nw = w.norm();
            if (!(nu > 0.) || !(nw > 0.)) continue;
            const double cosine = std::clamp(u.dot(w) / (nu * nw), -1., 1.);
            worst = std::min(worst, std::acos(cosine) * 180. / M_PI);
        }
    }
    return worst;
}

} // namespace

double TopoOffsetTetMesh::refined_marching_bound() const
{
    // The far simplices of the band: a vertex, edge or face of a band cell with no vertex on the
    // input complex. Deduplicated -- neighbouring band cells share most of them.
    std::set<size_t> far_points;
    std::set<std::array<size_t, 2>> far_segments;
    std::set<std::array<size_t, 3>> far_triangles;
    const auto outside = [&](const size_t v) { return m_vertex_extra[v].label == 0; };
    for (const Tuple& t : get_tets()) {
        const auto vs = oriented_tet_vids(t.tid(*this));
        int n_out = 0;
        for (int i = 0; i < 4; ++i) n_out += outside(vs[i]) ? 1 : 0;
        if (n_out == 0 || n_out == 4) continue; // not a band cell: no root, so no front piece
        for (int i = 0; i < 4; ++i) {
            if (!outside(vs[i])) continue;
            far_points.insert(vs[i]);
            for (int j = i + 1; j < 4; ++j) {
                if (!outside(vs[j])) continue;
                std::array<size_t, 2> e{{vs[i], vs[j]}};
                std::sort(e.begin(), e.end());
                far_segments.insert(e);
                for (int k = j + 1; k < 4; ++k) {
                    if (!outside(vs[k])) continue;
                    std::array<size_t, 3> f{{vs[i], vs[j], vs[k]}};
                    std::sort(f.begin(), f.end());
                    far_triangles.insert(f);
                }
            }
        }
    }

    std::vector<DistSimplex> far;
    far.reserve(far_points.size() + far_segments.size() + far_triangles.size());
    Vector3d pts[3];
    for (const size_t v : far_points) {
        pts[0] = m_vertex_attribute[v].m_posf;
        far.emplace_back().set(pts, 1);
    }
    for (const auto& e : far_segments) {
        for (int i = 0; i < 2; ++i) pts[i] = m_vertex_attribute[e[size_t(i)]].m_posf;
        far.emplace_back().set(pts, 2);
    }
    for (const auto& f : far_triangles) {
        for (int i = 0; i < 3; ++i) pts[i] = m_vertex_attribute[f[size_t(i)]].m_posf;
        far.emplace_back().set(pts, 3);
    }

    const std::vector<DistSimplex> input =
        collect_input_simplices(m_phi_V, m_phi_E, m_phi_F, m_phi_P);
    if (far.empty() || input.empty()) {
        log_and_throw_error(
            "refined_marching_bound: {} far simplices, {} input primitives -- nothing to measure",
            far.size(),
            input.size());
    }

    // Brute force over the pairs. SimplicialComplexBVH answers point queries only, so there is no
    // tree query for a simplex-to-simplex distance; the bounding spheres reject nearly all pairs
    // before any of them is solved, and the log line reports the time this took.
    double best_sq = std::numeric_limits<double>::max();
    double best = std::numeric_limits<double>::max();
    for (const DistSimplex& f : far) {
        for (const DistSimplex& g : input) {
            if ((f.c - g.c).norm() > f.r + g.r + best) continue;
            const double d2 = pair_sq(f, g);
            if (d2 < best_sq) {
                best_sq = d2;
                best = std::sqrt(best_sq);
            }
        }
    }
    return best;
}

void TopoOffsetTetMesh::refine_for_marching()
{
    const auto clock_now = []() { return std::chrono::high_resolution_clock::now(); };
    const auto seconds_since = [&](const auto& t) {
        return std::chrono::duration<double>(clock_now() - t).count();
    };
    const auto t_start = clock_now();

    const auto t_bound = clock_now();
    const double B = refined_marching_bound();
    const double bound_seconds = seconds_since(t_bound);
    const double fraction = m_offset_params.refined_marching_distance_fraction;
    if (!(B > 0.) || !(fraction > 0.) || !(fraction < 1.)) {
        log_and_throw_error(
            "refine_for_marching: B = {} and refined_marching_distance_fraction = {}; the "
            "fraction must be in (0, 1) and B positive, or march_distance does not lie inside the "
            "band",
            B,
            fraction);
    }
    const double march_distance = fraction * B;
    const double tol = m_offset_params.refined_marching_tol_rel * march_distance;
    // Every root the loop computes is computed by the same trace that will place the vertex
    // there, so the loop measures the surface the marching will actually produce -- including the
    // trace's own tolerance. That tolerance is therefore a floor on the accuracy: see the spec
    // entry for refined_marching_tol_rel.
    m_march_distance = march_distance;

    const EdgeSplitMode entry_split_mode = m_edge_split_mode;
    m_edge_split_mode = EdgeSplitMode::Midpoint; // the mesh changes by midpoint bisections only

    /// What the marching will put in one band cell, and the certified residual of each piece.
    struct BandCell
    {
        std::vector<double> piece_test; ///< the certified residual, one per front triangle
        std::array<size_t, 2> longest_unmarched{{0, 0}}; ///< what a failing piece bisects
    };
    std::unordered_map<size_t, BandCell> band;

    const std::vector<DistSimplex> input_simplices =
        collect_input_simplices(m_phi_V, m_phi_E, m_phi_F, m_phi_P);
    size_t n_outward_zero = 0, n_outward_binding = 0, n_piece_tests = 0, n_exact_solves = 0;
    double residual_seconds = 0.;

    const auto inside = [&](const size_t v) { return m_vertex_extra[v].label != 0; };
    const auto pos = [&](const size_t v) -> const Vector3d& {
        return m_vertex_attribute[v].m_posf;
    };
    const auto sorted_edge = [](const size_t a, const size_t b) {
        return a < b ? std::array<size_t, 2>{{a, b}} : std::array<size_t, 2>{{b, a}};
    };
    const auto edge_length = [&](const std::array<size_t, 2>& e) {
        return (pos(e[0]) - pos(e[1])).norm();
    };

    // Roots are cached per marched edge: a bisection never moves an existing vertex and never
    // reuses a vertex id, so an entry once computed stays correct for the whole loop.
    std::map<std::array<size_t, 2>, Vector3d> roots;
    size_t traces_off_edge = 0;
    const auto root_of = [&](const size_t v_in, const size_t v_out) -> Vector3d {
        const std::array<size_t, 2> key{{v_in, v_out}};
        const auto it = roots.find(key);
        if (it != roots.end()) return it->second;
        Vector3d p;
        size_t steps = 0;
        if (!edge_split_sphere_trace(pos(v_in), pos(v_out), march_distance, p, steps)) {
            // B says this cannot happen: the outside end of a marched edge of a band cell is a
            // far simplex, so it is farther from the complex than B > march_distance and d passes
            // march_distance on the edge. Counted rather than asserted, and placed where the
            // marching itself would put it, so the loop keeps measuring the surface that will be
            // built.
            ++traces_off_edge;
            p = 0.5 * (pos(v_in) + pos(v_out));
        }
        roots.emplace(key, p);
        return p;
    };
    // March one cell: its roots and its front triangles, all geometric. Removes the cell's record
    // first, so a cell that is no longer a band cell simply leaves no record.
    const auto march_cell = [&](const size_t tid) {
        band.erase(tid);
        const auto vs = oriented_tet_vids(tid);
        std::vector<size_t> in, out;
        for (int i = 0; i < 4; ++i) (inside(vs[i]) ? in : out).push_back(vs[i]);
        if (in.empty() || out.empty()) return;
        std::sort(in.begin(), in.end());
        std::sort(out.begin(), out.end());

        BandCell c;
        std::array<Vector3d, 4> r;
        std::vector<std::array<int, 3>> tris;
        if (in.size() == 1 || out.size() == 1) {
            // Three roots, all on the edges from the lone vertex: one front triangle.
            const bool lone_inside = in.size() == 1;
            const size_t lone = lone_inside ? in[0] : out[0];
            const std::vector<size_t>& oth = lone_inside ? out : in;
            for (int k = 0; k < 3; ++k) {
                r[size_t(k)] =
                    lone_inside ? root_of(lone, oth[size_t(k)]) : root_of(oth[size_t(k)], lone);
            }
            tris.push_back({0, 1, 2});
        } else {
            // Two inside (a, b) and two outside (p, q): four roots in the cyclic order
            // (a,p), (a,q), (b,q), (b,p), cut into two triangles by the fixed diagonal
            // root(a,p) - root(b,q), the same diagonal the marching builds.
            const size_t a = in[0], b = in[1], p = out[0], q = out[1];
            r[0] = root_of(a, p);
            r[1] = root_of(a, q);
            r[2] = root_of(b, q);
            r[3] = root_of(b, p);
            tris.push_back({0, 1, 2});
            tris.push_back({0, 2, 3});
        }

        // The certified residual of each front triangle. It bounds |d - march_distance| over the
        // WHOLE triangle, edges and interior alike, so there is nothing else to test: no separate
        // test of the sides, and the quadrilateral's diagonal is covered like any other chord.
        const auto t_res = clock_now();
        for (const auto& tri : tris) {
            const Vector3d corners[3] = {r[size_t(tri[0])], r[size_t(tri[1])], r[size_t(tri[2])]};
            const Vector3d centroid = (corners[0] + corners[1] + corners[2]) / 3.;
            // One nearest-primitive query per corner: the candidate set of the residual must hold
            // a primitive nearest to a corner, and the distance at the corner is what admits it.
            const double corner_dist[3] = {
                m_input_complex_bvh->dist(corners[0]),
                m_input_complex_bvh->dist(corners[1]),
                m_input_complex_bvh->dist(corners[2])};
            const Residual res = certified_residual(
                input_simplices,
                corners,
                corner_dist,
                3,
                centroid,
                m_input_complex_bvh->dist(centroid),
                march_distance);
            c.piece_test.push_back(res.value);
            n_outward_zero += res.outward_zero ? 1 : 0;
            n_outward_binding += res.outward_binds ? 1 : 0;
            n_exact_solves += res.exact_solves;
            ++n_piece_tests;
        }
        residual_seconds += seconds_since(t_res);

        // The longest unmarched edge of the cell: what a failing piece of this cell bisects. Ties
        // to the lowest vertex ids, so the run is deterministic.
        double longest = -1.;
        for (int i = 0; i < 4; ++i) {
            for (int j = i + 1; j < 4; ++j) {
                if (inside(vs[i]) != inside(vs[j])) continue;
                const auto e = sorted_edge(vs[i], vs[j]);
                const double len = edge_length(e);
                if (len > longest || (len == longest && e < c.longest_unmarched)) {
                    longest = len;
                    c.longest_unmarched = e;
                }
            }
        }
        band.emplace(tid, std::move(c));
    };

    // Phi(e), the set of BAND FACES of the current mesh containing the edge e, searched for an
    // edge strictly longer than e. A band face is alpha(sigma) or beta(sigma) of a band cell
    // sigma when that face is a TRIANGLE, i.e. the triangle of three equally-labelled vertices in
    // a cell with one inside and three outside vertices, or three inside and one outside; a cell
    // with two of each has no band face, its alpha and beta being edges. The edges of a band face
    // are all same-side, so every candidate returned here is an unmarched edge and no marched edge
    // is ever split.
    //
    // Returns false when e is a longest edge of every band face containing it -- the exit
    // condition of the propagation loop below. Otherwise `out` is the longest edge of all the
    // faces of Phi(e), which is in particular a longest edge of the face holding it, so it is a
    // valid choice under the rule "pick a face with an edge strictly longer than e and take a
    // longest edge of that face". Ties go to the lowest sorted pair of vertex ids, as everywhere
    // else in this loop, so the run is deterministic.
    //
    // Phi(e) is read from the live mesh on every call and never cached: a split changes which
    // tetrahedra are incident to e.
    const auto longer_edge_in_band_face = [&](const std::array<size_t, 2>& e,
                                              std::array<size_t, 2>& out) {
        double best = edge_length(e);
        bool found = false;
        for (const Tuple& tt : get_incident_tets_for_edge(tuple_from_edge(e))) {
            const auto vs = oriented_tet_vids(tt.tid(*this));
            int n_in = 0;
            for (int i = 0; i < 4; ++i) n_in += inside(vs[i]) ? 1 : 0;
            if (n_in != 1 && n_in != 3) continue; // 0 and 4: no band cell; 2: no band face
            const bool face_inside = n_in == 3;
            std::array<size_t, 3> f{{0, 0, 0}};
            int nf = 0;
            for (int i = 0; i < 4; ++i) {
                if (inside(vs[i]) == face_inside) f[size_t(nf++)] = vs[i];
            }
            int ends_on_face = 0;
            for (int i = 0; i < 3; ++i) {
                ends_on_face += (f[size_t(i)] == e[0] || f[size_t(i)] == e[1]) ? 1 : 0;
            }
            if (ends_on_face != 2) continue; // e is not an edge of this band face
            for (int i = 0; i < 3; ++i) {
                for (int j = i + 1; j < 3; ++j) {
                    const auto cand = sorted_edge(f[size_t(i)], f[size_t(j)]);
                    const double len = edge_length(cand);
                    if (len > best || (found && len == best && cand < out)) {
                        best = len;
                        out = cand;
                        found = true;
                    }
                }
            }
        }
        return found;
    };

    for (const Tuple& t : get_tets()) march_cell(t.tid(*this));
    const size_t band_cells_before = band.size();

    size_t n_bisections = 0, n_input_edges = 0;
    size_t n_propagated = 0, deepest_propagation = 0;
    double worst_left = 0.;
    while (true) {
        // The front piece with the largest certified residual, over the whole front. One test and
        // one bar: a piece fails while its residual exceeds tol, and the cell it belongs to
        // bisects its own longest unmarched edge. Ties go to the lowest vertex ids of that edge,
        // so the run is deterministic.
        double worst = -1.;
        std::array<size_t, 2> pick{{0, 0}};
        for (const auto& [tid, c] : band) {
            for (const double v : c.piece_test) {
                if (v > worst || (v == worst && c.longest_unmarched < pick)) {
                    worst = v;
                    pick = c.longest_unmarched;
                }
            }
        }
        if (!(worst > tol)) {
            worst_left = std::max(worst, 0.);
            break;
        }

        // BISECT(pick), the propagated bisection of the termination proof, run iteratively on an
        // explicit stack of edges rather than by C++ recursion, because the chain of propagation
        // can be long. The top of the stack is the edge being worked on: while some band face
        // containing it has a strictly longer edge, that longer edge is pushed and dealt with
        // first, and the edge underneath survives, since the edge split above it is strictly
        // longer and therefore a different edge. When the top is a longest edge of every band face
        // containing it, it is split and popped. With refined_marching_propagate false the stack
        // never holds more than the picked edge and this is one split, the earlier behaviour.
        std::vector<std::array<size_t, 2>> to_split{pick};
        while (!to_split.empty()) {
            const std::array<size_t, 2> cur = to_split.back();
            std::array<size_t, 2> longer{{0, 0}};
            if (m_offset_params.refined_marching_propagate &&
                longer_edge_in_band_face(cur, longer)) {
                to_split.push_back(longer);
                ++n_propagated;
                deepest_propagation = std::max(deepest_propagation, to_split.size() - 1);
                continue;
            }
            to_split.pop_back();

            const size_t x = cur[0], y = cur[1];
            if (inside(x) && inside(y)) ++n_input_edges;
            const Tuple e = tuple_from_edge({{x, y}});
            for (const Tuple& tt : get_incident_tets_for_edge(e)) band.erase(tt.tid(*this));
            std::vector<Tuple> garbage;
            reserve_edge_split(e);
            if (!split_edge(e, garbage)) {
                // Unreachable, and a defect if it fires. Storage cannot refuse: the slots are
                // reserved just above (this loop used to consolidate and retry instead, which at
                // preallocation_factor 1.0 re-reserved no headroom and aborted every case). The
                // hook cannot refuse: under refined_marching marching_split_edge_after() moves an
                // inverting vertex to the exact point of the edge, and throws if even that
                // inverts a tet. And invariants() re-tests the same one-ring with the same exact
                // predicate the hook has just passed.
                log_and_throw_error(
                    "refine_for_marching: bisection of edge ({}, {}) refused after {} bisections, "
                    "with its slots reserved and the split hook passed",
                    x,
                    y,
                    n_bisections);
            }
            for (const size_t tid : get_one_ring_tids_for_vertex(m_marching_last_new_vid)) {
                march_cell(tid);
            }
            ++n_bisections;
            if (n_bisections % 1000 == 0) {
                logger().info(
                    "\t[construction] refined_marching: {} bisections, {} band cells, worst "
                    "certified residual {:.3g} (bar {:.3g}), {} exact-rational fallbacks, {} "
                    "marched edges with no root on them",
                    n_bisections,
                    band.size(),
                    worst,
                    tol,
                    m_marching_rational_fallbacks,
                    traces_off_edge);
            }
        }
    }

    if (traces_off_edge > 0) {
        logger().warn(
            "\t[construction] refined_marching: {} marched edges had no point with d = "
            "march_distance on them and fell back to their midpoint -- march_distance is not "
            "inside the band there, so B is wrong",
            traces_off_edge);
    }
    logger().info(
        "\t[construction] refined_marching: B = {} exact ({:.2f} s), march_distance = {} x B = "
        "{}, tol = {} x march_distance = {} | band cells {} -> {} | {} bisections ({} of them on "
        "an input edge, {} of them reached by longest-edge propagation, deepest propagation "
        "chain {}) | largest certified residual left: {:.3g} (bar {:.3g}) | {} exact-rational "
        "fallbacks | {:.2f} s",
        B,
        bound_seconds,
        fraction,
        march_distance,
        m_offset_params.refined_marching_tol_rel,
        tol,
        band_cells_before,
        band.size(),
        n_bisections,
        n_input_edges,
        n_propagated,
        deepest_propagation,
        worst_left,
        tol,
        m_marching_rational_fallbacks,
        seconds_since(t_start));
    // The residual's own record: how often its outward term was zero -- one input primitive
    // nearest on all of the piece, where the test is exact -- and how often that term, rather
    // than the exact inward one, was the larger of the two. The last two numbers say what the
    // certificate costs: the bounding-sphere shell and the Lipschitz pruning leave only a handful
    // of simplex-to-simplex solves per piece, and the whole residual is a fifth to two fifths of
    // the loop.
    logger().info(
        "\t[construction] refined_marching certified residual: {} piece tests, {:.1f}% with a zero "
        "outward term, {:.1f}% with the outward term binding | {} input primitives, {:.1f} exact "
        "simplex-to-simplex solves per test, {:.2f} s = {:.0f}% of the loop",
        n_piece_tests,
        n_piece_tests ? 100. * double(n_outward_zero) / double(n_piece_tests) : 0.,
        n_piece_tests ? 100. * double(n_outward_binding) / double(n_piece_tests) : 0.,
        input_simplices.size(),
        n_piece_tests ? double(n_exact_solves) / double(n_piece_tests) : 0.,
        residual_seconds,
        100. * residual_seconds / std::max(1e-12, seconds_since(t_start)));

    m_edge_split_mode = entry_split_mode;
}

void TopoOffsetTetMesh::validate_refined_marching() const
{
    const double march_distance = m_march_distance;
    const double tol = m_offset_params.refined_marching_tol_rel * march_distance;

    // A front piece is a face of a band tet all of whose vertices this marching placed. Dense
    // sampling of it, not the loop's three points: this is the measurement that can contradict
    // the loop, so it must not share the loop's information.
    const std::set<size_t> placed(m_marching_new_verts.begin(), m_marching_new_verts.end());
    std::set<std::array<size_t, 3>> pieces;
    double min_dihedral = 180.;
    size_t inverted = 0;
    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        const auto vs = oriented_tet_vids(tid);
        if (is_inverted(t)) ++inverted;
        min_dihedral = std::min(
            min_dihedral,
            min_dihedral_degrees(
                {m_vertex_attribute[vs[0]].m_posf,
                 m_vertex_attribute[vs[1]].m_posf,
                 m_vertex_attribute[vs[2]].m_posf,
                 m_vertex_attribute[vs[3]].m_posf}));
        if (m_tet_attribute[tid].label != 2) continue; // not in the band
        for (int i = 0; i < 4; ++i) {
            for (int j = i + 1; j < 4; ++j) {
                for (int k = j + 1; k < 4; ++k) {
                    if (!placed.count(vs[i]) || !placed.count(vs[j]) || !placed.count(vs[k])) {
                        continue;
                    }
                    std::array<size_t, 3> f{{vs[i], vs[j], vs[k]}};
                    std::sort(f.begin(), f.end());
                    pieces.insert(f);
                }
            }
        }
    }

    double max_sag = 0.;
    size_t over_tol = 0;
    constexpr int SIDE_SAMPLES = 65; // as in the reference bench
    constexpr int GRID = 12; // barycentric grid per triangle: (GRID + 1)(GRID + 2) / 2 points
    for (const auto& f : pieces) {
        const Vector3d a = m_vertex_attribute[f[0]].m_posf;
        const Vector3d b = m_vertex_attribute[f[1]].m_posf;
        const Vector3d c = m_vertex_attribute[f[2]].m_posf;
        double sag = 0.;
        const std::array<std::pair<Vector3d, Vector3d>, 3> edges{{{a, b}, {b, c}, {c, a}}};
        for (const auto& e : edges) {
            for (int i = 0; i < SIDE_SAMPLES; ++i) {
                const double s = double(i) / double(SIDE_SAMPLES - 1);
                const Vector3d p = e.first + s * (e.second - e.first);
                sag = std::max(sag, std::abs(m_input_complex_bvh->dist(p) - march_distance));
            }
        }
        for (int i = 0; i <= GRID; ++i) {
            for (int j = 0; i + j <= GRID; ++j) {
                const double u = double(i) / GRID, v = double(j) / GRID;
                const Vector3d p = u * a + v * b + (1. - u - v) * c;
                sag = std::max(sag, std::abs(m_input_complex_bvh->dist(p) - march_distance));
            }
        }
        max_sag = std::max(max_sag, sag);
        if (sag > tol) ++over_tol;
    }

    logger().info(
        "\t[construction] refined_marching validation (dense sampling, diagnostic): {} front "
        "pieces | true sag max {:.3f}% of march_distance, {} pieces over tol ({}% of "
        "march_distance) | smallest dihedral angle {:.3f} deg | {} inverted or zero-volume tets "
        "(exact predicate)",
        pieces.size(),
        100. * max_sag / march_distance,
        over_tol,
        100. * m_offset_params.refined_marching_tol_rel,
        min_dihedral,
        inverted);
    if (inverted > 0) {
        logger().warn(
            "\t[construction] refined_marching validation: {} inverted or zero-volume tets",
            inverted);
    }
}

void TopoOffsetTetMesh::remesh_refined_march()
{
    // WHAT IT IS. Plain TetWild (the shared engine's mesh_improvement(), Phase A) over the mesh
    // the refined march constructed, run once, before the offset potential exists. Terms: the
    // FRONT is the offset surface, the faces between the band (label 2) and the cells outside it;
    // the INPUT SURFACE is the boundary of the input complex; tol = refined_marching_tol_rel x
    // march_distance, the refined march's own tolerance, so the front as constructed is within tol
    // of the level set d(x) = march_distance (d the Euclidean distance to the input complex).
    //
    // INVARIANT: each surface stays inside its own tube of half-width tol around where the march
    // put it, and no vertex ever belongs to both. So the front ends within 2 tol of d =
    // march_distance (tol from the march, tol from the tube) and the input surface within tol of
    // the input.
    //   * the front's tube: m_offset_envelope, rebuilt here around the front faces with width tol;
    //   * every tag boundary and the input complex, the domain wall included:
    //     build_boundary_envelopes(PerTagAndComplex) with width tol, whatever deform_others says.
    //     The pass only simplifies the mesh, so it must move no surface; deform_others lets the
    //     optimization push the offset into other regions, and under it (WallComplex) the outlines
    //     of regions that are neither the input nor the band are held by nothing -- measured on
    //     presmooth2d/line: the outlines of 'left' and 'right' moved up to 0.23 during the pass.
    // Every operation checks a tracked face against the tube of the surface it belongs to through
    // one dispatch, surface_envelope_for_face() -> containment_for(face mask, all corners on the
    // front), which in Phase A answers m_offset_envelope for a front face and the face's own
    // region tube(s) for a region face:
    //   * split: TetOptimizerMesh::split_edge_after() checks both child triangles of every tracked
    //     face on the split edge;
    //   * collapse: TetOptimizerMesh::collapse_edge_after() checks every tracked face around the
    //     removed vertex, re-attached to the survivor;
    //   * swap: the shared surface flips check both new triangles (swap_edge_after, 44, 56).
    //     swap_capture_tag() refuses every flip of the front and of a solid complex's boundary,
    //     whose ring spans two cell labels; a sheet's faces have the band on both sides and are
    //     flipped (presmooth3d/sheet: 40 flips in this pass), see swap_before_surface();
    //   * smooth: smooth_vertex_3d() pulls a vertex onto smoothing_energy_envelope() (the front's
    //     tube for a front vertex, see there; the worst-violated region tube for a region vertex)
    //     and checks every tracked face at the vertex against smoothing_containment_envelope() =
    //     containment_for(vertex mask, on the front).
    // Which vertex may merge into which is collapse_before_vertex()'s, unchanged: never onto a
    // vertex of the other surface, never off its own class.
    //
    // WHY. The refined march reaches its accuracy by bisection, and leaves a background far finer
    // than either surface needs: presmooth3d/cylinder goes from 1252 input tets to 75177.
    const double tol = m_offset_params.refined_marching_tol_rel * m_march_distance;

    const OptPhase saved_phase = m_phase;
    const EdgeSplitMode saved_mode = m_edge_split_mode;
    const bool saved_plastic = m_plastic_active;
    const bool saved_freeze = m_freeze_front;
    m_phase = OptPhase::A;
    m_edge_split_mode = EdgeSplitMode::Optimization;
    m_plastic_active = false; // regular-tet AMIPS alone; no rest shapes exist yet
    m_freeze_front = false; // the front is smoothed, projected back into its tube

    // The two surfaces as the operations read them, as optimize_offset() sets them up: the front
    // faces get their own class (and every cell its quality), the vertex orders feed the link
    // condition and the open-boundary rule.
    label_offset_boundary();
    init_vertex_order();
    // m_is_on_input is what keeps the input surface and the front apart in
    // collapse_before_vertex(). Construction sets it on every vertex it creates, from the same
    // label (label 1 = on the input complex; see the construction splits in EdgeSplittingTet.cpp),
    // so the two must agree here. Checked, not re-derived: a disagreement is a construction defect,
    // and patching it here would hide it from every other consumer of the flag.
    size_t n_unflagged = 0;
    for (const Tuple& v : get_vertices()) {
        const VertexExtra& x = m_vertex_extra[v.vid(*this)];
        if ((x.label == 1) != x.m_is_on_input) ++n_unflagged;
    }
    if (n_unflagged != 0) {
        log_and_throw_error(
            "refined_marching_remesh: {} vertices whose input-complex flag disagrees with their "
            "construction label -- a construction split did not set the flag",
            n_unflagged);
    }
    // A plain TetWild run against the base target length l, as pre_optimize_input_mesh() is.
    for (const Tuple& v : get_vertices()) m_vertex_attribute[v.vid(*this)].m_sizing_scalar = 1.0;

    rebuild_offset_envelope(tol);
    build_boundary_envelopes("refined_marching_remesh", EnvelopeSetup::PerTagAndComplex, tol);

    iter_cnt_collapse_both_surfaces_reject = 0;
    iter_cnt_collapse_class_reject = 0;
    iter_cnt_split_front_chord = 0;
    iter_cnt_split_input_chord = 0;
    iter_cnt_split_input_on_input = 0;
    const int splits0 = iter_cnt_split.load(), collapses0 = iter_cnt_collapse.load(),
              swaps0 = iter_cnt_swap.load();
    // The input complex's faces as the labels say, before and after: the operations carry the
    // labels, so a count that collapses to 0 on a sheet means a label was lost.
    const auto count_complex_faces = [this]() {
        size_t n = 0;
        for (const Tuple& f : get_faces()) n += face_is_complex_boundary(f) ? 1 : 0;
        return n;
    };
    const size_t complex_faces_before = count_complex_faces();
    m_ab_round = 0;
    const double before = std::get<0>(optimization_quality_stats());
    logger().info(
        "[refined_marching_remesh] TetWild over the constructed mesh: {} vertices, {} tets, max "
        "AMIPS {:.4} (stop {:.4}) | front and input surface each in its own tube of half-width "
        "tol = {:.6g}",
        get_vertices().size(),
        get_tets().size(),
        before,
        optimization_stop_metric(),
        tol);

    m_remesh_pass = true;
    mesh_improvement(std::max(1, m_offset_params.max_iterations));
    m_remesh_pass = false;

    const double after = std::get<0>(optimization_quality_stats());
    logger().info(
        "[refined_marching_remesh] done: {} vertices, {} tets, max AMIPS {:.4} -> {:.4} | {} "
        "splits, {} collapses, {} swaps | collapses refused for joining the two surfaces {}, for "
        "leaving the vertex's own class {} | split midpoints flagged by the endpoint rule although "
        "the "
        "edge lies on no face of that surface (chords): front {}, input {}",
        get_vertices().size(),
        get_tets().size(),
        before,
        after,
        iter_cnt_split.load() - splits0,
        iter_cnt_collapse.load() - collapses0,
        iter_cnt_swap.load() - swaps0,
        iter_cnt_collapse_both_surfaces_reject.load(),
        iter_cnt_collapse_class_reject.load(),
        iter_cnt_split_front_chord.load(),
        iter_cnt_split_input_chord.load());
    logger().info(
        "\t[labels] of those input chords, {} have their midpoint ON the input complex: a lost "
        "construction label, not a chord | input-complex faces (face_is_complex_boundary): {} "
        "before the pass, {} after",
        iter_cnt_split_input_on_input.load(),
        complex_faces_before,
        count_complex_faces());

    m_phase = saved_phase;
    m_edge_split_mode = saved_mode;
    m_plastic_active = saved_plastic;
    m_freeze_front = saved_freeze;
    consolidate_mesh();
    check_no_vertex_on_both_surfaces("refined_marching_remesh");

    // What the rest of the run reads, put back the way construction leaves it. The construction
    // labels need nothing: every operation of the pass carried them (see merge_labels()), which is
    // what the driver's connected-component check, label_offset_boundary() and
    // face_is_complex_boundary() read next. They used to be re-derived here from the cells, which
    // loses a sheet -- a complex with no cells: 0 of its faces were left in the complex tube.
    // The sheet flag is geometric and nothing propagates it (see FaceExtra::on_sheet); the
    // region tubes below read it.
    classify_sheet_faces();
    // The region tubes at their own width (envelope_size) around the surfaces as the pass left
    // them, as the final pass rebuilds them; and no front tube, as before optimize_offset()
    // builds one -- a tol-wide tube must not outlive this pass.
    build_boundary_envelopes("after refined_marching_remesh", envelope_setup());
    {
        std::lock_guard<std::mutex> lock(m_isect_mutex);
        m_offset_isect_cache.clear();
    }
    m_offset_envelope = nullptr;
}

} // namespace wmtk::components::topological_offset
