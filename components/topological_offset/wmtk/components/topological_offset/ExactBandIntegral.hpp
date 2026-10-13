#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

namespace wmtk::components::topological_offset {

/**
 * @brief The band-volume integral of one cell, in closed form:
 *
 *     J(t) = int_t (d(x) - delta) dV,   d = Euclidean distance to the input triangle mesh.
 *
 * A port of the VALUE path of the Python reference exact_mesh.integrate_mesh (and of the
 * exact_tri closed forms it calls), scratchpad exact_integral/ of 2026-10-10/11: same algorithm,
 * same switch constants, same operation order, so it reproduces the reference's numbers to
 * rounding (tests/test_exact_band_integral.cpp). Used by EXPERIMENTAL_band_volume_rule "exact"
 * (the band cell term) and, through max_distance(), by the settle loop's max test.
 *
 * Features and local regions. The input's features are its face interiors, edges and vertices
 * (duplicate vertices merged). The local region L_g of a feature g is an intersection of
 * half-spaces built from its incident features only: a face's prism (inside the planes through
 * its edges perpendicular to it), an edge's slab between the planes through its ends perpendicular
 * to it, cut by the plane through the edge perpendicular to each incident face (away from that
 * face's third vertex), a vertex's cone {x : (x - v).(w - v) <= 0 for every edge (v, w)}. Two
 * facts make d = min over {g : x in L_g} of d_g exact, d_g the plane, line or point distance of g:
 * (i) the feature holding x's nearest point in its relative interior has x in its region, and
 * d_g(x) = d(x); (ii) on L_g, d_g is the distance to an actual point of g, so d_g >= d.
 *
 * Per cell:
 *  0. Candidates: U = min over triangles P of max over t's corners of d_P is an upper bound of
 *     d on t (d_P convex), so every feature whose bounding box is within U of t's box is kept.
 *     U is the exact min over ALL triangles (a tree search); the Python takes the min over the 32
 *     triangles nearest t by box distance, MEASURED equal on all 14180 cells checked (the five
 *     stored cases, sample + 3000 random each), although ties at rank 32 occur in up to 634 of
 *     3240 cells, where numpy's sort order decides which 32 it takes.
 *  1. t is cut by the candidates' local regions into convex pieces of constant member set
 *     S = {g : piece inside L_g}. Members that cannot attain the minimum on a piece are dropped
 *     (lower bound >= another member's upper bound); this changes no value.
 *  2. |S| = 1: the closed form of d_g. S all faces: split by the planes and the bisectors of
 *     the (then affine) distances, exact. S all vertices: split by the bisector planes, exact.
 *  3. Otherwise ("rule (a)") the boundary between members is curved and the piece returns
 *     min over g in S of int_piece d_g >= int_piece d: an UPPER BOUND, not the integral.
 *     Stats::rule_a_vol reports its volume (0.15-0.35% of the band volume on t100026).
 *
 * One-feature integrals (exact_tri.py): the face's |n.(x - a)| is affine on each side of its
 * plane (volume times centroid value); the line and point distances by Euler's theorem for
 * degree-1 homogeneous integrands, int_Q f = 1/4 sum_F h_F int_F f dA, faces by the divergence
 * theorem in their plane, edge integrals in sqrt, asinh and atan. Two pieces are convergent
 * series with explicit truncation bounds, not closed forms: a face nearly parallel to an edge
 * line (eps_F < 0.1) uses a Taylor series in the variation of the line's height over the face,
 * per edge either along the edge (ratio <= 1/4, at most 29 terms) or in eta (ratio < 1/20, at
 * most 13 terms), each stopped at its edge's actual ratio with the same tail bound.
 *
 * Conditioning (Python, test_exact_tri.py "far"): a piece of size L at distance D from the
 * singular point or line loses about eps (D/L)^2 in the Euler sums.
 *
 * The offset side (constructed with a side): the band B is the region between the input and the
 * front, so a point on the far side of the input surface from the offset is not in it, and
 * value() returns J(t) = int_{t cap B} (d - delta) dV. The side of x is decided by its nearest
 * feature g, by the sign of (x - c).N_g for its nearest point c (the pseudonormal test,
 * Baerentzen-Aanaes 2005): N_g = the face normal (face), n1 + n2 (edge), the angle-weighted
 * pseudonormal (vertex). On g's local region c is g's own nearest point to x, so the test is the
 * fixed plane {(x - q_g).N_g = 0}, q_g a point of g:
 *  - a one-member piece is clipped by its member's plane and only the offset-side part counts; a
 *    faces-only sub-piece (one face nearest, on one side of its plane) is kept or dropped whole;
 *    a vertices-only sub-piece is clipped by its vertex's plane;
 *  - a rule-(a) piece is split by its members' planes. Where all planes give the same side, the
 *    true side is that one (it is the nearest member's): the part is dropped, or takes rule (a)'s
 *    min. Where they disagree (D) the side is not known, and D adds
 *        vol(D) max(0, min over members of (max over D's vertices of d_g) - delta),
 *    an upper bound (chi (d - delta) <= max(0, d_g - delta), d_g convex). The plain min over the
 *    members' own clipped integrals is not one: MEASURED, two plates 0.02 apart, delta 0.2, a cell
 *    wholly inside the lower plate (true value 0): the upper plate's rim edge, whose region held
 *    the cell, put it on the offset side, and that min was -1.32e-5.
 *  - -delta vol(t) becomes -delta times the kept volume; a cell where nothing is clipped keeps
 *    the unclipped sum, int d - delta vol(t), bit for bit.
 * Features of isolated triangles (no side), edges without exactly two faces and zero
 * pseudonormals have no plane: every point counts there. Why (exact_mesh.band_integral): the
 * input surface inside the tet mesh may move within its envelope, and just inside the input body
 * the integrand is about -1, so pushing the band's inner boundary into the body lowered E;
 * operations accepted when they lower E ratcheted it inward. MEASURED: on the outward sphere
 * 446 of 881 inner-boundary vertices lay inside the input by turn 12, E_band ended 58.6 bars
 * below its exact minimum (Steiner's formula), and the settle test could not pass.
 *
 * Thread safety: value() is const and keeps no state; concurrent calls are safe.
 */
class ExactBandIntegral
{
public:
    /// V (n x 3) and F (m x 3): the input triangles. Vertices are merged by exact coordinates and
    /// sorted (numpy.unique, as the Python Mesh does), which fixes the order the features are
    /// visited in -- the rule-(a) pieces depend on it.
    ExactBandIntegral(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
        : ExactBandIntegral(V, F, {}, 0)
    {}
    /// With the offset side (class doc): face_side[f] = +1 if row f's normal (b - a) x (c - a)
    /// points out of the offset_selection region, -1 if into it, 0 for a triangle that bounds no
    /// solid; sigma = +1 for an outward offset (the band outside the selection), -1 inward, 0 no
    /// clip (as the two-argument constructor).
    ExactBandIntegral(
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXi& F,
        const std::vector<int8_t>& face_side,
        int sigma);

    /// Counters, volumes and (when `timing`) times of value() calls, ADDED to on every call (never
    /// reset by value()).
    struct Stats
    {
        int64_t cells = 0; ///< value() calls with a non-flat cell
        int64_t candidates = 0; ///< candidate features
        int64_t pieces = 0; ///< convex pieces of the partition
        int64_t rule_a_pieces = 0;
        double vol = 0.; ///< cell volume
        double exact_vol = 0.; ///< volume of pieces integrated exactly
        double rule_a_vol = 0.; ///< volume of rule-(a) pieces (upper bound)
        double empty_vol = 0.; ///< volume of pieces with no member (must be rounding)
        int64_t clips = 0; ///< half-space clips of a polytope
        int64_t clips_closed_up = 0; ///< cutting clips whose kept faces closed up without a cap
        int64_t caps_split = 0; ///< cutting clips whose cap took several circuits (faces)
        /// When set, every clip result is checked to be a closed surface: each directed edge once,
        /// its reverse once (closed_checked results, open_results of them failing).
        bool check_closed = false;
        int64_t closed_checked = 0, open_results = 0;
        int64_t point_faces = 0; ///< faces integrated against a point distance
        int64_t line_faces_radial = 0; ///< ... against a line distance, radial closed form
        int64_t line_faces_parallel = 0; ///< ... line exactly parallel to the face
        int64_t line_faces_series = 0; ///< ... near-parallel series
        int64_t closed_edges = 0; ///< polygon edges integrated by a closed form
        int64_t series_short = 0; ///< near-parallel edges, Taylor series along the edge
        int64_t series_long = 0; ///< near-parallel edges, Taylor series in eta
        /// The offset side (class doc): value() calls with anything clipped; rule-(a) pieces split
        /// by their members' planes; parts D where the planes disagree, of them where some
        /// member's distance exceeds delta at a vertex, and where the bound is positive.
        int64_t side_clipped = 0, side_rule_a_split = 0, side_D = 0, side_D_over = 0, side_D_pos = 0;
        /// Times in seconds, measured only when `timing` is set (the clock reads add a few percent,
        /// mac-mini). t_candidates: the U search, the box query and the local feature setup;
        /// t_clip: every half-space clip; t_closed: every face integral (point and line
        /// distance), INCLUDING t_series, the near-parallel series edges inside them.
        bool timing = false;
        double t_total = 0., t_candidates = 0., t_clip = 0., t_closed = 0., t_series = 0.;
    };

    /// J(t) = int_t (d - delta) dV for the cell with corners t (either orientation), restricted
    /// to the offset side when constructed with one (class doc); 0 for a flat cell
    /// (|6 vol| <= 64 eps L^3, L its longest edge). With `stats`, adds to it.
    double value(const std::array<Eigen::Vector3d, 4>& t, double delta, Stats* stats = nullptr)
        const;

    /**
     * @brief J and its exact gradient and Hessian with respect to corner `kv` of t, the other
     * three fixed (exact_mesh.value_grad_hess): Reynolds' transport theorem on each smooth piece,
     * a boundary integral over the three faces of t through x = t[kv], plus, on the Hessian, the
     * terms of piece edges sliding over those faces ("cut edges"; they cancel in pairs between
     * exact pieces and remain on rule-(a) pieces). J here sums the same pieces split into
     * smooth parts, so it equals value() to rounding.
     *
     * Where the derivative is not the derivative of J:
     *  - rule-(a) pieces: it is the derivative of the BOUND (the chosen member's integral over the
     *    piece). The bound is not smooth where the chosen member switches; the gradient there is
     *    one member's.
     *  - a kink of d lying on a face of t through x (a cell face on an input plane, or on a
     *    bisector): J is one-sided C^2; the Hessian is the limit for x moving INTO t (the trace
     *    of grad d from inside t).
     * With the offset side: each kept piece integrates d - delta, the side planes being fixed
     * faces of the pieces whose cut edges keep their terms (the integrand jumps to 0 across
     * them); a disagreeing rule-(a) part D adds vol(D) c(x), c = max(0, d_g*(v*) - delta) (g* the
     * member of smallest vertex max, v* its first maximising vertex of D), differentiated through
     * vol(D) and through v*'s motion with x (v* solves the 3 x 3 system of D's planes through it,
     * a face of t through x the moving one). c is not smooth where g* or v* switches.
     * Flat cells give J = 0 and zero derivatives.
     */
    double value_grad_hess(
        const std::array<Eigen::Vector3d, 4>& t,
        int kv,
        double delta,
        Eigen::Vector3d& grad,
        Eigen::Matrix3d& hess,
        Stats* stats = nullptr) const;

    /// What max_distance() found besides the max.
    struct MaxInfo
    {
        Eigen::Vector3d at; ///< a point of f where the returned max is attained
        bool rule_a = false; ///< the max came from a rule-(a) piece: an upper bound there
        double U = 0.; ///< min over P of max over f's corners of d_P (the former settle bound)
        int candidates = 0, pieces = 0, rule_a_pieces = 0;
    };

    /**
     * @brief max over the triangle f of d (exact_mesh.max_on_triangle), the settle loop's max
     * test. f is cut by the candidates' local regions as a cell is (the same search, region
     * tests, splits, tolerances and scale rule; f a planar convex ring in place of a polytope),
     * and d is convex wherever one member's distance is d, so on each piece the max is at a
     * vertex of the piece:
     *  - one member: the max of d_g over the piece's vertices, exact;
     *  - members all faces, or all vertices: split by the bisectors as in value(), each sub-ring
     *    one affine or point distance, exact;
     *  - otherwise (rule (a), the boundary between members curved): min over members g of the max
     *    of d_g over the piece's vertices, an upper bound on max of d there (d <= d_g).
     * The max over pieces is exact up to rounding unless it comes from a rule-(a) piece; never
     * below the true max. One difference from the cell's partition: a ring lying on a region's
     * boundary plane counts as inside that region (a point of the plane is in both closed
     * regions); a duplicated piece cannot lower a max, a lost one can.
     *
     * The former bound, U = min over P of max over f's corners of d_P (MaxInfo::U), is d_P's max
     * for the single best triangle P; it overshoots where f straddles a coplanar edge of the
     * input, by about s^2 / (2 delta) for a straddle s. MEASURED, cube outward at turn 28: all 8
     * faces it marked were 1.06-1.26 bars by U and -0.016..+0.020 bars by this max (the dense
     * sampled max agrees).
     *
     * Throws (with f in hex floats) if the pieces do not tile f, or leave more of f outside every
     * region, than the tolerances allow (the checks of value(), on areas).
     */
    double max_distance(const std::array<Eigen::Vector3d, 3>& f, MaxInfo* info = nullptr) const;

    size_t num_faces() const { return m_A.size(); }

private:
    /// A feature: kind 0 face (p = its first vertex, dir = unit normal), 1 edge (p = its first
    /// end, dir = unit direction), 2 vertex (p). Its local region is {x : g.(x - q) <= 0} over
    /// (g, q) in m_hs[h_begin, h_end).
    struct Feature
    {
        int kind = 0;
        Eigen::Vector3d p, dir;
        int h_begin = 0, h_end = 0;
        Eigen::Vector3d lo, hi; ///< bounding box
    };
    std::vector<Feature> m_feat; ///< faces, then edges, then vertices (the Python's order)
    /// Half-spaces as (unit normal g, point q on the plane). A cell takes the offset in its own
    /// frame, h = g.(q - c0), c0 its centroid: g.q - g.c0 in global coordinates carries
    /// eps |x_global|, which exceeds the clip tolerance 16 eps (|h| + max|V|) of the local frame
    /// when the input is large against the cell (t100026: |x| <= 143, cells ~1). On a plane two
    /// features share (a face's prism plane through an edge = that edge's wedge plane), in/out
    /// was then rounding noise deciding how a rule-(a) piece is cut: MEASURED, one t100026 cell
    /// differed from the Python by 7.4e-5 while every exact cell agreed to 1e-14. Fixed the same
    /// way in the Python (exact_mesh.Mesh, _partition).
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> m_hs;
    int m_nF = 0, m_nE = 0;
    std::vector<Eigen::Vector3d> m_A, m_B, m_C; ///< triangle corners, for d_P
    std::vector<std::array<int, 6>> m_face_feats; ///< per face: its 3 edges, its 3 vertices
    /// The offset side (class doc): sigma (0: none), and per feature its side plane, the unit
    /// normal N oriented out of the selection region and a point q on it; m_side_has[g] false
    /// where the feature has no plane (every point counts).
    int m_sigma = 0;
    std::vector<char> m_side_has;
    std::vector<Eigen::Vector3d> m_side_n, m_side_q;

    /// A tree over the triangles' boxes (median split on the longest axis of the centroids, as
    /// InputTriangles' tree), for U and the candidate query. Boxes padded by 1e-9 of the input's
    /// diagonal, so a box distance never exceeds a triangle distance by rounding.
    struct Node
    {
        Eigen::Vector3d lo, hi;
        int left = -1, right = -1, begin = 0, end = 0;
    };
    std::vector<Node> m_nodes;
    std::vector<int> m_order;
    int build_node(int begin, int end, const std::vector<Eigen::Vector3d>& centroid, double pad);

    /// Per-call scratch of the partitions (defined in the .cpp): the candidates in the frame of a
    /// cell or front face (Local), and the pieces of a cell (Work) or of a face (TriWork).
    struct Local;
    struct Work;
    struct TriWork;
    /// The candidates of the n points t (a cell's or a face's corners) in the frame of w.c0, set
    /// by the caller: U, the box query, the local features and half-spaces, the rounding scale.
    void localize(const Eigen::Vector3d* t, int n, Local& w, Stats* stats) const;
    /// _partition: the candidates in t's frame and the convex pieces of t with member sets; false
    /// for a flat cell.
    bool partition(const std::array<Eigen::Vector3d, 4>& t, Work& w, Stats* stats) const;
    /// _tri_partition: the same for the face f, its pieces planar rings.
    void tri_partition(const std::array<Eigen::Vector3d, 3>& f, TriWork& w) const;
    /// U: min over all triangles of max over the n points t of d_P.
    double upper_bound(const Eigen::Vector3d* t, int n) const;
    /// Feature indices whose box is within r of [lo, hi], ascending.
    void candidates(
        const Eigen::Vector3d& lo,
        const Eigen::Vector3d& hi,
        double r,
        std::vector<int>& out) const;
};

} // namespace wmtk::components::topological_offset
