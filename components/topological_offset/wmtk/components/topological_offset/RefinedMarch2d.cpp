#include <chrono>
#include <set>
#include <unordered_map>
#include "ExactSimplexDistance.hpp"
#include "TopoOffsetTriMesh.h"

namespace wmtk::components::topological_offset {

namespace {

/// A simplex of the input complex or a far simplex of a band cell, as geometry: 1 or 2 points,
/// plus the bounding circle that lets a pair be rejected without solving it. The 3D twin carries
/// triangles as well.
struct DistSimplex
{
    int n = 0;
    Vector2d p[2];
    Vector2d c;
    double r = 0.;

    void set(const Vector2d* pts, const int count)
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
    if (a.n == 1) return esd::point_segment_sq(a.p[0], b.p[0], b.p[1]);
    return esd::segment_segment_sq(a.p[0], a.p[1], b.p[0], b.p[1]);
}

/// d_g(p): the exact distance from a point to one simplex. Squared.
double point_simplex_sq(const Vector2d& p, const DistSimplex& g)
{
    namespace esd = exact_simplex_distance;
    if (g.n == 1) return esd::point_point_sq(p, g.p[0]);
    return esd::point_segment_sq(p, g.p[0], g.p[1]);
}

/// The input complex as geometry, from the one extraction the whole run uses
/// (init_input_complex_bvh): its boundary segments and its isolated vertices. Everything the
/// refined march measures lies outside the complex, and outside it the distance to the complex and
/// the distance to that boundary are the same number, so d is the smallest distance to one of these
/// primitives -- each of which is a CONVEX set, which is what the certified residual rests on. As
/// in 3D, where the primitives are triangles as well.
std::vector<DistSimplex>
collect_input_simplices(const MatrixXd& phi_V, const MatrixXi& phi_E, const std::vector<int>& phi_P)
{
    std::vector<DistSimplex> input;
    input.reserve(size_t(phi_E.rows()) + phi_P.size());
    Vector2d pts[2];
    for (int i = 0; i < phi_E.rows(); ++i) {
        for (int k = 0; k < 2; ++k) {
            pts[k] = Vector2d(phi_V(phi_E(i, k), 0), phi_V(phi_E(i, k), 1));
        }
        input.emplace_back().set(pts, 2);
    }
    for (const int v : phi_P) {
        pts[0] = Vector2d(phi_V(v, 0), phi_V(v, 1));
        input.emplace_back().set(pts, 1);
    }
    return input;
}

/**
 * @brief The certified residual of one front piece P: an upper bound on
 *        max over x in P of |d(x) - march_distance|, with no sampling and no constant. The 3D twin
 *        carries the derivation in full; in 2D the piece is a segment and its corners are its two
 *        ends.
 *
 * INWARD, exact: max over P of (march_distance - d) = march_distance - min over g of dist(P, g) =
 * march_distance - dist(P, input), because the two minima (over the piece and over the primitives)
 * may be exchanged. Each dist(P, g) is a closed-form segment-to-segment or segment-to-point
 * distance, never a sample.
 *
 * OUTWARD, a rigorous bound: d <= d_g for every primitive g and d_g is convex, so its maximum over
 * the segment P is at an END of P, giving max over P of (d - march_distance) <= max over ends of
 * d_g - march_distance for every g; the smallest such bound is taken. It is zero when one primitive
 * is nearest everywhere.
 *
 * THE CANDIDATE SET. The outward term is a minimum over candidate primitives, and the bound
 * R(P) <= diam(P) -- what makes shrinking a piece drive its residual to zero, hence what makes the
 * loop terminate -- needs the candidate set to contain a primitive NEAREST TO AT LEAST ONE END of
 * P. So the set is the union of the piece's bounding-circle shell (radius piece.r + u, u the
 * distance at the midpoint: the cheap bulk filter, which keeps every primitive that can hold the
 * exact minimum) and one disc per END, of radius d(end) around that end, which keeps the primitive
 * nearest to that end. The shell alone does not: it is sized by the distance at the MIDPOINT, and
 * an end can be piece.r farther from the complex than the midpoint is. As in 3D, where the piece
 * is a triangle and there are three corners.
 *
 * Cost, measured on presmooth2d/circle (distance_fraction 0.5, tol_rel 0.01) against the same run
 * without the end discs: every count is unchanged -- 35 bisections, 106 piece tests, 68.9% of them
 * with a zero outward term, 2.0 exact segment-to-segment solves per test -- and the residual's
 * share of the refinement loop rises from 8% to 15% of a loop that is under 0.01 s. The discs
 * changed no result here; they are here because nothing guarantees the shell keeps an end-nearest
 * primitive.
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
    const Vector2d* corner,
    const double* corner_dist,
    const int n_corner,
    const Vector2d& centroid,
    const double u,
    const double march_distance)
{
    DistSimplex piece;
    piece.set(corner, n_corner);
    const double in_shell = piece.r + u;

    // The candidate set: the piece's bounding-circle shell, plus the disc of radius d(corner)
    // around every corner. The corner discs are what keep a primitive nearest to a corner, which
    // is the invariant the outward bound rests on; see the derivation above. As in 3D.
    const auto candidate = [&](const DistSimplex& g) {
        if ((piece.c - g.c).norm() <= in_shell + g.r) return true;
        for (int i = 0; i < n_corner; ++i) {
            if ((corner[i] - g.c).norm() <= corner_dist[i] + g.r) return true;
        }
        return false;
    };

    double best = std::numeric_limits<double>::max();
    double outward = std::numeric_limits<double>::max();
    std::array<double, 2> at_corner; // min over the candidates of d_g(corner i), as in 3D
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
        log_and_throw_error(
            "certified_residual: no input primitive within {} of the piece centre, out of {} -- "
            "the primitive set and the distance query disagree",
            in_shell,
            input.size());
    }

    // The corner-nearest invariant, checked rather than assumed, PER CORNER, as in 3D (where the
    // comment says why the pooled minimum proves nothing and what the factor 1 + 1e-12 absorbs).
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

/// The smallest angle of a triangle, in degrees. The 3D twin measures dihedral angles.
double min_angle_degrees(const std::array<Vector2d, 3>& P)
{
    double worst = 180.;
    for (int i = 0; i < 3; ++i) {
        const Vector2d u = P[size_t((i + 1) % 3)] - P[size_t(i)];
        const Vector2d w = P[size_t((i + 2) % 3)] - P[size_t(i)];
        const double nu = u.norm(), nw = w.norm();
        if (!(nu > 0.) || !(nw > 0.)) continue;
        const double cosine = std::clamp(u.dot(w) / (nu * nw), -1., 1.);
        worst = std::min(worst, std::acos(cosine) * 180. / M_PI);
    }
    return worst;
}

} // namespace

double TopoOffsetTriMesh::refined_marching_bound() const
{
    // The far simplices of the band: a vertex or edge of a band cell with no vertex on the input
    // complex. Deduplicated -- neighbouring band cells share most of them.
    std::set<size_t> far_points;
    std::set<std::array<size_t, 2>> far_segments;
    const auto outside = [&](const size_t v) { return m_vertex_extra[v].label == 0; };
    for (const Tuple& f : get_faces()) {
        const auto vs = oriented_tri_vids(f.fid(*this));
        int n_out = 0;
        for (int i = 0; i < 3; ++i) n_out += outside(vs[i]) ? 1 : 0;
        if (n_out == 0 || n_out == 3) continue; // not a band cell: no root, so no front piece
        for (int i = 0; i < 3; ++i) {
            if (!outside(vs[i])) continue;
            far_points.insert(vs[i]);
            for (int j = i + 1; j < 3; ++j) {
                if (!outside(vs[j])) continue;
                std::array<size_t, 2> e{{vs[i], vs[j]}};
                std::sort(e.begin(), e.end());
                far_segments.insert(e);
            }
        }
    }

    std::vector<DistSimplex> far;
    far.reserve(far_points.size() + far_segments.size());
    Vector2d pts[2];
    for (const size_t v : far_points) {
        pts[0] = m_vertex_attribute[v].m_posf;
        far.emplace_back().set(pts, 1);
    }
    for (const auto& e : far_segments) {
        for (int i = 0; i < 2; ++i) pts[i] = m_vertex_attribute[e[size_t(i)]].m_posf;
        far.emplace_back().set(pts, 2);
    }

    const std::vector<DistSimplex> input = collect_input_simplices(m_phi_V, m_phi_E, m_phi_P);
    if (far.empty() || input.empty()) {
        log_and_throw_error(
            "refined_marching_bound: {} far simplices, {} input primitives -- nothing to measure",
            far.size(),
            input.size());
    }

    // Brute force over the pairs. SimplicialComplexBVH answers point queries only, so there is no
    // tree query for a simplex-to-simplex distance; the bounding circles reject nearly all pairs
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

void TopoOffsetTriMesh::refine_for_marching()
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
    // there, so the loop measures the front the marching will actually produce -- including the
    // trace's own tolerance. That tolerance is therefore a floor on the accuracy: see the spec
    // entry for refined_marching_tol_rel.
    m_march_distance = march_distance;

    const EdgeSplitMode entry_split_mode = m_edge_split_mode;
    m_edge_split_mode = EdgeSplitMode::Midpoint; // the mesh changes by midpoint bisections only

    /// What the marching will put in one band cell, and the certified residual of it. The 3D twin
    /// carries one residual per front triangle, a cell having one or two of them.
    struct BandCell
    {
        double piece_test = 0.; ///< the certified residual of the front segment
        std::array<size_t, 2> unmarched_edge{{0, 0}}; ///< the cell's one unmarched edge
    };
    std::unordered_map<size_t, BandCell> band;

    const std::vector<DistSimplex> input_simplices =
        collect_input_simplices(m_phi_V, m_phi_E, m_phi_P);
    size_t n_outward_zero = 0, n_outward_binding = 0, n_piece_tests = 0, n_exact_solves = 0;
    double residual_seconds = 0.;

    const auto inside = [&](const size_t v) { return m_vertex_extra[v].label != 0; };
    const auto pos = [&](const size_t v) -> const Vector2d& {
        return m_vertex_attribute[v].m_posf;
    };

    // Roots are cached per marched edge: a bisection never moves an existing vertex and never
    // reuses a vertex id, so an entry once computed stays correct for the whole loop.
    std::map<std::array<size_t, 2>, Vector2d> roots;
    size_t traces_off_edge = 0;
    const auto root_of = [&](const size_t v_in, const size_t v_out) -> Vector2d {
        const std::array<size_t, 2> key{{v_in, v_out}};
        const auto it = roots.find(key);
        if (it != roots.end()) return it->second;
        Vector2d p;
        size_t steps = 0;
        if (!edge_split_sphere_trace(pos(v_in), pos(v_out), march_distance, p, steps)) {
            // B says this cannot happen: the outside end of a marched edge of a band cell is a
            // far simplex, so it is farther from the complex than B > march_distance and d passes
            // march_distance on the edge. Counted rather than asserted, and placed where the
            // marching itself would put it, so the loop keeps measuring the front that will be
            // built.
            ++traces_off_edge;
            p = 0.5 * (pos(v_in) + pos(v_out));
        }
        roots.emplace(key, p);
        return p;
    };
    // March one cell: its two roots and the side joining them, all geometric. Removes the cell's
    // record first, so a cell that is no longer a band cell simply leaves no record.
    const auto march_cell = [&](const size_t fid) {
        band.erase(fid);
        const auto vs = oriented_tri_vids(fid);
        std::vector<size_t> in, out;
        for (int i = 0; i < 3; ++i) (inside(vs[i]) ? in : out).push_back(vs[i]);
        if (in.empty() || out.empty()) return;
        std::sort(in.begin(), in.end());
        std::sort(out.begin(), out.end());

        // Two roots, both on the edges from the lone vertex: the side is the segment joining
        // them, and it lies in the cell whose third edge (x, y) is the one unmarched one.
        const bool lone_inside = in.size() == 1;
        const size_t lone = lone_inside ? in[0] : out[0];
        const std::vector<size_t>& oth = lone_inside ? out : in;
        const Vector2d r0 = lone_inside ? root_of(lone, oth[0]) : root_of(oth[0], lone);
        const Vector2d r1 = lone_inside ? root_of(lone, oth[1]) : root_of(oth[1], lone);

        // The certified residual of the front segment. It bounds |d - march_distance| over the
        // WHOLE segment, so its midpoint is not a separate test.
        BandCell c;
        const auto t_res = clock_now();
        const Vector2d corners[2] = {r0, r1};
        const Vector2d centroid = 0.5 * (r0 + r1);
        // One nearest-primitive query per end: the candidate set of the residual must hold a
        // primitive nearest to an end, and the distance at that end is what admits it.
        const double corner_dist[2] = {
            m_input_complex_bvh->dist(corners[0]),
            m_input_complex_bvh->dist(corners[1])};
        const Residual res = certified_residual(
            input_simplices,
            corners,
            corner_dist,
            2,
            centroid,
            m_input_complex_bvh->dist(centroid),
            march_distance);
        c.piece_test = res.value;
        n_outward_zero += res.outward_zero ? 1 : 0;
        n_outward_binding += res.outward_binds ? 1 : 0;
        n_exact_solves += res.exact_solves;
        ++n_piece_tests;
        residual_seconds += seconds_since(t_res);
        c.unmarched_edge = oth[0] < oth[1] ? std::array<size_t, 2>{{oth[0], oth[1]}}
                                           : std::array<size_t, 2>{{oth[1], oth[0]}};
        band.emplace(fid, c);
    };

    for (const Tuple& f : get_faces()) march_cell(f.fid(*this));
    const size_t band_cells_before = band.size();

    size_t n_bisections = 0, n_input_edges = 0;
    double worst_left = 0.;
    while (true) {
        // The front piece with the largest certified residual, over the whole front. One test and
        // one bar: a piece fails while its residual exceeds tol, and its cell bisects its one
        // unmarched edge. Ties go to the lowest vertex ids of that edge, so the run is
        // deterministic. In 3D a cell has several unmarched edges and the longest is taken; here
        // there is only one, so the two rules coincide.
        double worst = -1.;
        std::array<size_t, 2> pick{{0, 0}};
        for (const auto& [fid, c] : band) {
            if (c.piece_test > worst || (c.piece_test == worst && c.unmarched_edge < pick)) {
                worst = c.piece_test;
                pick = c.unmarched_edge;
            }
        }
        if (!(worst > tol)) {
            worst_left = std::max(worst, 0.);
            break;
        }

        // The picked edge is split directly. NO longest-edge propagation here, and this is the one
        // place where 2D deliberately does not mirror 3D. In 3D the picked edge is not split until
        // every BAND FACE containing it -- the triangle of three equally-labelled vertices of a
        // band cell with one vertex on one side and three on the other -- has it as a longest
        // edge, the longer edges of those faces being bisected first; see
        // refined_marching_propagate and the 3D twin. That recursion is what bounds the number of
        // splits in 3D, and 2D does not need it: a band triangle's only unmarched edge is its
        // alpha or its beta, each of which descends from an edge of the initial mesh by halvings
        // alone, so every edge this loop splits is a dyadic sub-segment of an initial edge, and an
        // initial edge has only finitely many dyadic sub-segments longer than the length below
        // which every piece already passes. (Termination proof, final remark.)
        const size_t x = pick[0], y = pick[1];
        if (inside(x) && inside(y)) ++n_input_edges;
        const Tuple e = get_tuple_from_edge(simplex::Edge(x, y));
        for (const size_t fid : get_incident_fids_for_edge(e)) band.erase(fid);
        std::vector<Tuple> garbage;
        reserve_edge_split(e);
        if (!split_edge(e, garbage)) {
            // Unreachable, and a defect if it fires, for the reasons given on the 3D twin: the
            // slots are reserved just above (this loop used to consolidate and retry instead), the
            // hook moves an inverting vertex to the exact point of the edge or throws, and
            // invariants() re-tests the same one-ring with the same exact predicate.
            log_and_throw_error(
                "refine_for_marching: bisection of edge ({}, {}) refused after {} bisections, "
                "with its slots reserved and the split hook passed",
                x,
                y,
                n_bisections);
        }
        for (const Tuple& f :
             get_one_ring_tris_for_vertex(tuple_from_vertex(m_marching_last_new_vid))) {
            march_cell(f.fid(*this));
        }
        ++n_bisections;

        if (n_bisections % 1000 == 0) {
            logger().info(
                "\t[construction] refined_marching: {} bisections, {} band cells, worst certified "
                "residual {:.3g} (bar {:.3g}), {} exact-rational fallbacks, {} marched edges with "
                "no root on them",
                n_bisections,
                band.size(),
                worst,
                tol,
                m_marching_rational_fallbacks,
                traces_off_edge);
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
        "an input edge) | largest certified residual left: {:.3g} (bar {:.3g}) | {} "
        "exact-rational fallbacks | {:.2f} s",
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
        worst_left,
        tol,
        m_marching_rational_fallbacks,
        seconds_since(t_start));
    // The residual's own record, as in 3D: how often its outward term was zero -- one input
    // primitive nearest on all of the piece, where the test is exact -- how often that term rather
    // than the exact inward one was the larger, and what the certificate cost.
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

void TopoOffsetTriMesh::validate_refined_marching() const
{
    const double march_distance = m_march_distance;
    const double tol = m_offset_params.refined_marching_tol_rel * march_distance;

    // A front piece is an edge of a band triangle whose two vertices this marching placed. Dense
    // sampling of it, not the loop's one point: this is the measurement that can contradict the
    // loop, so it must not share the loop's information.
    const std::set<size_t> placed(m_marching_new_verts.begin(), m_marching_new_verts.end());
    std::set<std::array<size_t, 2>> pieces;
    double min_angle = 180.;
    size_t inverted = 0;
    for (const Tuple& f : get_faces()) {
        const size_t fid = f.fid(*this);
        const auto vs = oriented_tri_vids(fid);
        if (is_inverted(fid)) ++inverted;
        min_angle = std::min(
            min_angle,
            min_angle_degrees(
                {m_vertex_attribute[vs[0]].m_posf,
                 m_vertex_attribute[vs[1]].m_posf,
                 m_vertex_attribute[vs[2]].m_posf}));
        if (m_face_extra[fid].label != 2) continue; // not in the band
        for (int i = 0; i < 3; ++i) {
            for (int j = i + 1; j < 3; ++j) {
                if (!placed.count(vs[i]) || !placed.count(vs[j])) continue;
                std::array<size_t, 2> e{{vs[i], vs[j]}};
                std::sort(e.begin(), e.end());
                pieces.insert(e);
            }
        }
    }

    double max_sag = 0.;
    size_t over_tol = 0;
    constexpr int SIDE_SAMPLES = 65; // as in the reference bench
    for (const auto& e : pieces) {
        const Vector2d a = m_vertex_attribute[e[0]].m_posf;
        const Vector2d b = m_vertex_attribute[e[1]].m_posf;
        double sag = 0.;
        for (int i = 0; i < SIDE_SAMPLES; ++i) {
            const double s = double(i) / double(SIDE_SAMPLES - 1);
            const Vector2d p = a + s * (b - a);
            sag = std::max(sag, std::abs(m_input_complex_bvh->dist(p) - march_distance));
        }
        max_sag = std::max(max_sag, sag);
        if (sag > tol) ++over_tol;
    }

    logger().info(
        "\t[construction] refined_marching validation (dense sampling, diagnostic): {} front "
        "pieces | true sag max {:.3f}% of march_distance, {} pieces over tol ({}% of "
        "march_distance) | smallest angle {:.3f} deg | {} inverted or zero-area triangles (exact "
        "predicate)",
        pieces.size(),
        100. * max_sag / march_distance,
        over_tol,
        100. * m_offset_params.refined_marching_tol_rel,
        min_angle,
        inverted);
    if (inverted > 0) {
        logger().warn(
            "\t[construction] refined_marching validation: {} inverted or zero-area triangles",
            inverted);
    }
}

void TopoOffsetTriMesh::remesh_refined_march()
{
    // WHAT IT IS. Plain TriWild (the shared engine's mesh_improvement(), Phase A) over the mesh
    // the refined march constructed, run once, before the offset potential exists. The 2D twin of
    // TopoOffsetTetMesh::remesh_refined_march(), one dimension down: both tracked surfaces are
    // curves here. Terms: the FRONT is the offset boundary, the edges between the band (label 2)
    // and the triangles outside it (edge_is_offset_surface_live()); the INPUT is the part of the
    // input complex the operations can reach -- the boundary segments of a complex made of
    // triangles, or the complex itself when it has none (a curve: the curve group, or an interface
    // such as "left & right"); tol = refined_marching_tol_rel x march_distance, the refined
    // march's own tolerance, so the front as constructed is within tol of the level set
    // d(x) = march_distance (d the Euclidean distance to the input complex).
    //
    // INVARIANT: each curve stays inside its own tube of half-width tol around where the march
    // put it, and no vertex ever belongs to both. So the front ends within 2 tol of d =
    // march_distance (tol from the march, tol from the tube) and the input within tol of the
    // input.
    //   * the front's tube: m_offset_envelope, rebuilt here around the front segments with width
    //     tol -- a polyline envelope, where 3D's is built on triangles;
    //   * every tag boundary and the input complex, the domain wall included:
    //     build_boundary_envelopes(PerTagAndComplex) with width tol, whatever deform_others says,
    //     for the reason given in 3D: the pass only simplifies the mesh and must move no curve
    //     (measured
    //     here, on presmooth2d/line: under WallComplex the outlines of 'left' and 'right' moved up
    //     to 0.23).
    // Every operation checks a tracked edge against the tube of the curve it belongs to through
    // one dispatch, surface_envelope_for_edge() -> containment_for(edge mask, both ends on the
    // front), which in Phase A answers m_offset_envelope for a front edge and the edge's own
    // region tube(s) for a region edge:
    //   * split: TriOptimizerMesh::split_edge_after() checks both child segments of a tracked
    //     edge;
    //   * collapse: TriOptimizerMesh::collapse_edge_after() checks every tracked edge at the
    //     removed vertex, re-attached to the survivor;
    //   * swap: nothing to check, and here 2D differs from 3D. TriOptimizerMesh::swap_edge_before()
    //     refuses every tracked edge, so a flip never re-triangulates either curve; 3D's surface
    //     flips replace two tracked faces by two new ones and must check them;
    //   * smooth: smooth_vertex_2d() pulls a vertex onto smoothing_energy_envelope() (the front's
    //     tube for a front vertex, see there; the worst-violated region tube for a region vertex)
    //     and checks every tracked edge at the vertex against smoothing_containment_envelope() =
    //     containment_for(vertex mask, on the front).
    // Which vertex may merge into which is collapse_before_vertex()'s, unchanged: never onto a
    // vertex of the other curve, never off its own class.
    //
    // WHY. The refined march reaches its accuracy by bisection, and can leave a background far
    // finer than either surface needs (in 3D, presmooth3d/cylinder: 1252 input tets, 75177 after
    // the march). The 2D cases measured so far are the other way round: their inputs are coarser
    // than the target length (presmooth2d: median input edge 0.24, l = length_rel x diagonal =
    // 0.141) and the march adds 33 to 40 bisections, so the pass refines the background while it
    // coarsens the front -- circle 388 -> 398 triangles and 71 -> 50 front segments, square 386 ->
    // 428 and 80 -> 59, line 328 -> 451 and 57 -> 49 (distance_fraction 0.5, tol_rel 0.01).
    const double tol = m_offset_params.refined_marching_tol_rel * m_march_distance;

    const OptPhase saved_phase = m_phase;
    const EdgeSplitMode saved_mode = m_edge_split_mode;
    const bool saved_plastic = m_plastic_active;
    const bool saved_freeze = m_freeze_front;
    m_phase = OptPhase::A;
    m_edge_split_mode = EdgeSplitMode::Optimization;
    m_plastic_active = false; // regular-triangle AMIPS alone; no rest shapes exist yet
    m_freeze_front = false; // the front is smoothed, projected back into its tube

    // The two curves as the operations read them, as optimize_offset() sets them up: the front
    // edges get their own class (and every triangle its quality). 3D also calls
    // init_vertex_order() here, because a tet mesh stores each vertex's order for the link
    // condition and the open-boundary rule; a triangle mesh computes it from the tracked edges
    // whenever it is asked (TriMesh::get_order_of_vertex()), so 2D has nothing to initialize.
    label_offset_boundary();
    // m_is_on_input is what keeps the input and the front apart in collapse_before_vertex().
    // Construction sets it on every vertex it creates, from the same label (label 1 = on the input
    // complex; see marching_split_edge_before() and split_face_before() in EdgeSplittingTri.cpp),
    // so the two must agree here. Checked, not re-derived: a disagreement is a construction
    // defect, and patching it here would hide it from every other consumer of the flag. As in 3D.
    size_t n_unflagged = 0;
    for (const Tuple& v : get_vertices()) {
        const VertexExtra2d& x = m_vertex_extra[v.vid(*this)];
        if ((x.label == 1) != x.m_is_on_input) ++n_unflagged;
    }
    if (n_unflagged != 0) {
        log_and_throw_error(
            "refined_marching_remesh: {} vertices whose input-complex flag disagrees with their "
            "construction label -- a construction split did not set the flag",
            n_unflagged);
    }
    // A plain TriWild run against the base target length l, as pre_optimize_input_mesh() is.
    for (const Tuple& v : get_vertices()) m_vertex_attribute[v.vid(*this)].m_sizing_scalar = 1.0;

    rebuild_offset_envelope(tol);
    build_boundary_envelopes("refined_marching_remesh", EnvelopeSetup::PerTagAndComplex, tol);

    iter_cnt_collapse_both_surfaces_reject = 0;
    iter_cnt_collapse_class_reject = 0;
    // One chord counter, where 3D has two: a split midpoint's front flag is the split edge's own
    // class here (split_adjust_position()), never an endpoint rule, so the front has no chords to
    // count.
    iter_cnt_split_input_chord = 0;
    iter_cnt_split_input_on_input = 0;
    const int splits0 = iter_cnt_split.load(), collapses0 = iter_cnt_collapse.load(),
              swaps0 = iter_cnt_swap.load();
    m_ab_round = 0;
    const double before = std::get<0>(optimization_quality_stats());
    logger().info(
        "[refined_marching_remesh] TriWild over the constructed mesh: {} vertices, {} "
        "triangles, max AMIPS {:.4} (stop {:.4}) | front and input each in its own tube of "
        "half-width tol = {:.6g}",
        get_vertices().size(),
        get_faces().size(),
        before,
        optimization_stop_metric(),
        tol);

    m_remesh_pass = true;
    mesh_improvement(std::max(1, m_offset_params.max_iterations));
    m_remesh_pass = false;

    const double after = std::get<0>(optimization_quality_stats());
    logger().info(
        "[refined_marching_remesh] done: {} vertices, {} triangles, max AMIPS {:.4} -> {:.4} | {} "
        "splits, {} collapses, {} swaps | collapses refused for joining the two curves {}, for "
        "leaving the vertex's own class {} | split midpoints the endpoint rule would have put on "
        "the input although the edge lies in no input triangle and is no input edge (chords): {}",
        get_vertices().size(),
        get_faces().size(),
        before,
        after,
        iter_cnt_split.load() - splits0,
        iter_cnt_collapse.load() - collapses0,
        iter_cnt_swap.load() - swaps0,
        iter_cnt_collapse_both_surfaces_reject.load(),
        iter_cnt_collapse_class_reject.load(),
        iter_cnt_split_input_chord.load());
    logger().info(
        "\t[labels] of those input chords, {} have their midpoint ON the input complex: a lost "
        "construction label, not a chord",
        iter_cnt_split_input_on_input.load());

    m_phase = saved_phase;
    m_edge_split_mode = saved_mode;
    m_plastic_active = saved_plastic;
    m_freeze_front = saved_freeze;
    consolidate_mesh();
    check_no_vertex_on_both_surfaces("refined_marching_remesh");

    // What the rest of the run reads, put back the way construction leaves it. The construction
    // labels need nothing: every operation of the pass carried them (see merge_labels() in the
    // header), which is what label_offset_boundary(), assign_band_regions() and
    // edge_is_complex_boundary() read next. They used to be re-derived here from the faces, plus
    // an endpoint rule to recover a curve complex's edges (a curve lies in no input face); with
    // the edges carried, neither is needed. As in 3D.
    // The curve flag is geometric and nothing propagates it (see EdgeExtra2d::on_curve); the
    // region tubes below read it under PerTag. The 3D twin re-derives its sheet flag here.
    classify_curve_edges();
    // The region tubes at their own width (envelope_size) around the curves as the pass left
    // them, as the final pass rebuilds them; and no front tube, as before optimize_offset() builds
    // one -- a tol-wide tube must not outlive this pass.
    build_boundary_envelopes("after refined_marching_remesh", envelope_setup());
    {
        std::lock_guard<std::mutex> lock(m_isect_mutex);
        m_offset_isect_cache.clear();
    }
    m_offset_envelope = nullptr;
}

} // namespace wmtk::components::topological_offset
