#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
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
 *    min. Where they disagree (D) the true side is still one for all of D: the plane test of a
 *    feature nearest at x gives x's true side, so members of opposite verdicts are never nearest
 *    together in D. It is the side of the member nearest at D's centroid: D is dropped, or takes
 *    rule (a)'s min over the members that put it on the offset side, the others being nearest
 *    nowhere in it. Until 2026-10-14 D took vol(D) max(0, min over members of (max over D's
 *    vertices of d_g) - delta), an upper bound that depended on how the cell cut D: MEASURED on
 *    100033 (plates 2.9 thick, delta 2.49), up to 1.24 between two triangulations of one region,
 *    and a swap pass that cycled. The plain min over ALL members' own clipped integrals is no
 *    bound: MEASURED, two plates 0.02 apart, delta 0.2, a cell wholly inside the lower plate (true
 *    value 0): the upper plate's rim edge, whose region held the cell, put it on the offset side,
 *    and that min was -1.32e-5 (the lower plate's face, nearest there, drops it).
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
 * Thread safety: value() and value_grad_hess() are const. Their state is a per-thread scratch
 * partition and value()'s table of results, whose slots are read and written under a lock
 * (value()); concurrent calls are safe. A Partition (below) is the caller's, used by one thread.
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
    /// points out of the solid the triangles bound, -1 if into it, 0 for a triangle that bounds no
    /// solid; sigma = +1 for the band outside that solid, -1 inside it, 0 no clip (as the
    /// two-argument constructor). TopoOffsetTetMesh's solid is its label-1 cells, which the band
    /// is outside of for offset_out and offset_in alike, so it passes +1.
    /// value_slots > 0: value() keeps a table of its results of at least that many slots (the
    /// value table, value()).
    ExactBandIntegral(
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXi& F,
        const std::vector<int8_t>& face_side,
        int sigma,
        size_t value_slots = 0);

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
        /// by their members' planes; parts D where the planes disagree, and of them those on the
        /// offset side (the side of the member nearest at their centroid), kept.
        int64_t side_clipped = 0, side_rule_a_split = 0, side_D = 0, side_D_pos = 0;
        /// Times in seconds, measured only when `timing` is set (the clock reads add a few percent,
        /// mac-mini). t_candidates: the U search, the box query and the local feature setup;
        /// t_clip: every half-space clip; t_closed: every face integral (point and line
        /// distance), INCLUDING t_series, the near-parallel series edges inside them.
        bool timing = false;
        double t_total = 0., t_candidates = 0., t_clip = 0., t_closed = 0., t_series = 0.;
    };

    /**
     * @brief J(t) = int_t (d - delta) dV for the cell with corners t (either orientation),
     * restricted to the offset side when constructed with one (class doc); 0 for a flat cell
     * (|6 vol| <= 64 eps L^3, L its longest edge). With `stats`, adds to it.
     *
     * The value table (constructed with value_slots > 0; not used with `stats`, whose counters
     * count every evaluation): J is a function of its 13 inputs alone -- t's 12 coordinates in
     * their order, delta -- and of the input triangles, fixed. A slot, chosen by a hash of the
     * inputs, holds the last result with its inputs; a call whose inputs equal a slot's, bit for
     * bit, returns its result, which is what the evaluation would return again. Any other call
     * evaluates and takes the slot over: a cell that moved or two cells sharing a slot cost an
     * evaluation, never a different value.
     *
     * Why: the energy rule reads the band cells of an operation's ring before it, and an unchanged
     * cell is read again by every later operation whose ring holds it. MEASURED (Geneva base,
     * inward, exact rule, the turn-1 collapse group, every evaluation of value() logged by its
     * inputs): the collapse before-sums made 8.5M calls (6.2 band cells per sum), every one of
     * them a repeat of an earlier call, 83% of the group's value() time; the after-sums 0.69M,
     * 62% repeats (a collapse refused, retried in a later round). A table taking every result,
     * of 2^18 / 2^20 / 2^22 slots, would have answered 98.2 / 99.5 / 99.9% of the before-sums and
     * 52 / 59 / 61% of the after-sums.
     */
    double value(const std::array<Eigen::Vector3d, 4>& t, double delta, Stats* stats = nullptr)
        const;

    struct Work;
    /**
     * @brief One cell's partition, kept by the caller between value() and value_grad_hess() of
     * that cell: each partitions t (partition(), the candidates and the convex pieces) before
     * integrating, and a partition is a function of t's 12 coordinates, in their order, and of the
     * input alone. Given a Partition, a call whose t equals, bit for bit, the t the Partition was
     * made for (by this integral) reads it; any other call partitions t into it. The value is
     * therefore the one the call would compute without it.
     *
     * Why: the smoother asks for J at x (value(), the line search) and then for its derivatives
     * at the same x (value_grad_hess()), each partitioning every band cell of the ring. MEASURED
     * (Geneva base, inward, the smoothing pass after the split, sampled): partition() was 51% of
     * the busy time, 20% inside value_grad_hess() and 31% inside value().
     */
    class Partition
    {
    public:
        Partition();
        ~Partition();
        Partition(Partition&&) noexcept;
        Partition& operator=(Partition&&) noexcept;

    private:
        friend class ExactBandIntegral;
        std::unique_ptr<Work> m_w;
        uint64_t m_owner = 0; ///< the integral's m_id; 0: none
        double m_key[12] = {};
        bool m_ok = false; ///< false: t is flat
    };
    /// value(t, delta), t's partition read from or written to `keep` (Partition).
    double value(const std::array<Eigen::Vector3d, 4>& t, double delta, Partition& keep) const;
    /// Grow the value table (value()) to at least `slots` slots, emptied; no table, or one that
    /// large already: nothing. Not thread-safe: called between passes. A table sized to the
    /// constructed mesh only answers few calls once the mesh has grown. MEASURED (cube 1e-2 /
    /// 1e-4, front_flux, 15 rounds, table sized at construction, 380k tets late): the smoothing
    /// vetoes' value() calls hit it 1.6-2.3% of the time, the operations' 65%.
    void reserve_values(size_t slots) const;

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
     * them); a disagreeing rule-(a) part D takes the side of the member nearest at its centroid
     * (class doc): dropped, or a rule-(a) part over the members that put it on the offset side.
     * The side can switch where that member does, so J is not smooth there.
     * Flat cells give J = 0 and zero derivatives.
     */
    double value_grad_hess(
        const std::array<Eigen::Vector3d, 4>& t,
        int kv,
        double delta,
        Eigen::Vector3d& grad,
        Eigen::Matrix3d& hess,
        Stats* stats = nullptr) const;
    /// value_grad_hess(), t's partition read from or written to `keep` (Partition).
    double value_grad_hess(
        const std::array<Eigen::Vector3d, 4>& t,
        int kv,
        double delta,
        Eigen::Vector3d& grad,
        Eigen::Matrix3d& hess,
        Partition& keep) const;

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

    // --------------------------------------------------------------------------------------------
    // The front form (EXPERIMENTAL_band_volume_rule "front_flux"; Python prototype front_form.py,
    // d_vertex.py, e_hessian.py, scratchpad front_form/ of 2026-10-13). Per feature k, in its
    // local region, a field G_k with div G_k = d - delta, tangent to the region's planes and 0 on
    // the input (d = 0):
    //     face    G = (d^2/2 - delta d) W        d = W.x - c, W = +-n on each side of its plane
    //     edge    G = (d/3 - delta/2) (x - x_e)  x_e the projection of x on the edge's line
    //     vertex  G = (d/4 - delta/3) (x - v).
    // On a band B made of cells, the divergence theorem cell by cell gives
    //     E(B) = int_B (d - delta) dV = sum over the boundary faces f of B of int_f G.n dA
    //            + sum over the cells t of D_t,
    // D_t = int_t (d - delta) dV - oint_dt G.n dA: the jumps of G.n across the BISECTORS inside t
    // (the sets where two features are nearest), 0 for a cell no bisector crosses. Every piece of
    // the partition (as value()'s) is labelled with its nearest feature exactly: a member whose
    // squared distance is not below another's anywhere on the piece is dropped (the exact range
    // of the difference, a quadratic, over the piece); members that still cross must have a
    // planar bisector (face-face, vertex-vertex, parallel or intersecting edge-edge), and the
    // piece is split by it. A crossing pair with a curved bisector (face-edge, face-vertex,
    // edge-vertex, skew edges) THROWS std::runtime_error naming the pair and the position:
    // there is no approximation in this form (rule (a)'s bound is value()'s only).
    //
    // The offset side (class doc), as value(): a labelled part counts only on the offset side of
    // its label's plane. On a labelled part the label is the nearest feature, whose plane meets it
    // only where d = 0 (the pseudonormal theorem), so the side drops whole parts: a face's far
    // side, and the edge and vertex parts beyond the input (MEASURED, the side cases' cells of
    // the tests: 47 edge parts clipped by their side plane, every one dropped whole, none cut).
    // Where G = 0 (on the input) the
    // dropped parts carry no flux, so the identity holds with the side as without it. The
    // Python prototype used the unsigned d (both sides of every face counted); the two agree on
    // every part on the offset side, and constructed without a side this class is the prototype.

    struct FaceWork;
    /**
     * @brief One face's labelled parts, kept by the caller between front_face() calls on that
     * face, as Partition keeps a cell's partition: the parts are a function of f's 9 coordinates,
     * in their order, and of the input alone (not of delta or kv). A call whose f equals, bit for
     * bit, the f they were made for (by this integral) reads them; any other call makes them anew
     * into it. Why: the smoother asks for the value at x and then for the derivatives at the same
     * x, each labelling every boundary face through x.
     */
    class FaceParts
    {
    public:
        FaceParts();
        ~FaceParts();
        FaceParts(FaceParts&&) noexcept;
        FaceParts& operator=(FaceParts&&) noexcept;

    private:
        friend class ExactBandIntegral;
        std::unique_ptr<FaceWork> m_w;
        uint64_t m_owner = 0; ///< the integral's m_id; 0: none
        double m_key[9] = {};
    };
    /**
     * @brief front_face(): int over the triangle f of G.n dA, n = (f1 - f0) x (f2 - f0) / |.| (f
     * ordered with the normal pointing out of the band). front_face_derivs(): for the motion of
     * corner kv (the others fixed), the gradient and the one-vertex Hessian of E's dependence
     * through f:
     *     grad = n int_f chi (d - delta) phi dA,
     *     hess = n (x) int_f chi phi^2 grad d dA + (int_f chi (d - delta) phi dA / |N|) dN/dx
     *            - sum over cut edges c: n (x) g_c int_c (d - delta) phi^2 ds / |g_c,f|,
     * phi the hat function of corner kv, chi the offset side's indicator, N = (f1 - f0) x
     * (f2 - f0) with column l of dN/dx = e_l x (x_a - x_b) (f ordered (x, x_a, x_b)), and the cut
     * edges the lines where f crosses an input face's plane into the far side (chi (d - delta)
     * jumps by delta there; g_c the plane's normal out of the kept side, g_c,f its part in f's
     * plane). Summed over the band's boundary faces through x these are E(B)'s gradient and
     * Hessian in x: Reynolds' transport theorem on int_B chi (d - delta), the boundary moving
     * with velocity phi; d is Lipschitz, so no medial-axis term and no D_t term. A part of f
     * lying on the input (d = 0 on it) takes grad d from the band side (the side the band is on,
     * the offset side with a side; -n without one), the one-sided limit, as value_grad_hess().
     * Throws on a curved bisector (above) or if f's pieces do not tile it (max_distance()).
     * With `keep`, f's labelled parts are read from or written to it (FaceParts).
     */
    double front_face(
        const std::array<Eigen::Vector3d, 3>& f,
        double delta,
        FaceParts* keep = nullptr) const;
    void front_face_derivs(
        const std::array<Eigen::Vector3d, 3>& f,
        double delta,
        int kv,
        Eigen::Vector3d& grad,
        Eigen::Matrix3d& hess,
        FaceParts* keep = nullptr) const;
    /**
     * @brief D_t = int_t (d - delta) dV - oint_dt G.n dA of the cell t (either orientation): 0
     * exactly when one feature labels every part of t on the offset side (one field); otherwise
     * the flux of each labelled part's field through its faces not on dt, outward from the part,
     * a face on one of the label's own region planes skipped (the field is tangent to it). The
     * faces between two parts of one label cancel, so the sum is the jumps of G.n where the label
     * changes: on the bisectors, and on region planes that coincide with one (MEASURED, Python,
     * cube_out tet 2234: the prism plane between two coplanar triangles of a cube face is the
     * bisector of the two perpendicular faces at the cube edge it contains) or nearly so (see the
     * .cpp). t's partition read from or written to `keep` when given (Partition). Throws on a
     * curved bisector, or as value() if the pieces do not tile t.
     */
    double bisector_jump(
        const std::array<Eigen::Vector3d, 4>& t,
        double delta,
        Partition* keep = nullptr) const;
    /**
     * @brief Builds, from the input alone, where bisectors can be: for every pair of features g, h
     * the polytope L_g cap L_h cap box_g(R) cap box_h(R) (and, with a side, cap both features'
     * offset sides), R = reach = 2 delta, kept where it has volume. A bisector point y of g and
     * h with d(y) <= R lies in it: both are nearest at y, so y is in both regions, on the offset
     * side of both planes, and within d_g(y) = d(y) of a point of g (fact (ii) of the class doc),
     * so in g's box grown by R. Called once, before the integral is shared.
     */
    void prepare_front_flux(double delta);
    /// prepare_front_flux()'s R (0 before it), and the number of polytopes it kept.
    double front_flux_reach() const { return m_ff_reach; }
    size_t front_flux_polytopes() const { return m_ff_boxes.size(); }
    /**
     * @brief Whether the cell t can meet a bisector at points with d <= front_flux_reach(): t's
     * box meets a kept polytope's box and no face plane of that polytope has all four corners
     * strictly outside it (beyond rounding). Conservative: a cell flagged without a bisector gets
     * D_t = 0 up to rounding (exactly 0 when one feature labels all of it). Says nothing about
     * points with d > reach: the caller flags a cell whose bound on d (distance_bound()) exceeds
     * the reach itself (BandVolumeEnergy3D).
     */
    bool front_flux_flag(const std::array<Eigen::Vector3d, 4>& t) const
    {
        return front_flux_flag(t.data(), 4);
    }
    /// The same test for the n points t: their box and their side of each plane. A subset of the
    /// points is flagged only if the points are, so the points of an operation's ring unflagged
    /// leave every cell made from them unflagged.
    bool front_flux_flag(const Eigen::Vector3d* t, int n) const;
    /// d(x), the distance to the input: min over the triangles of the point-triangle distance.
    double distance(const Eigen::Vector3d& x) const { return upper_bound(&x, 1); }
    /// U = min over the input triangles P of max over t's corners of d_P: an upper bound of d on
    /// t (each d_P is convex, and d <= d_P), the bound value()'s candidate search starts from.
    double distance_bound(const std::array<Eigen::Vector3d, 4>& t) const
    {
        return upper_bound(t.data(), 4);
    }
    /// The same bound for the n points t; a subset's bound is never larger.
    double distance_bound(const Eigen::Vector3d* t, const int n) const { return upper_bound(t, n); }

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
    /// normal N oriented out of the solid (face_side) and a point q on it; m_side_has[g] false
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
    struct TriWork;
    /// The candidates of the n points t (a cell's or a face's corners) in the frame of w.c0, set
    /// by the caller: U, the box query, the local features and half-spaces, the rounding scale.
    void localize(const Eigen::Vector3d* t, int n, Local& w, Stats* stats) const;
    /// _partition: the candidates in t's frame and the convex pieces of t with member sets; false
    /// for a flat cell.
    bool partition(const std::array<Eigen::Vector3d, 4>& t, Work& w, Stats* stats) const;
    /// partition() of t (null: t is flat): into `own` with `stats` (which counts every
    /// partition), else read from or written to `keep` when given (Partition), else into a
    /// per-thread scratch partition (its storage reused from call to call).
    const Work* cell_partition(
        const std::array<Eigen::Vector3d, 4>& t,
        Stats* stats,
        Work* own,
        Partition* keep) const;
    /// This integral's number among all constructed, the owner a Partition records (an address
    /// can be reused by a later integral).
    uint64_t m_id = 0;
    /// value() without the value table.
    double evaluate(
        const std::array<Eigen::Vector3d, 4>& t,
        double delta,
        Stats* stats,
        Partition* keep) const;
    /// value(t, delta, stats, keep) through the value table (value()).
    double value_tabled(const std::array<Eigen::Vector3d, 4>& t, double delta, Partition* keep)
        const;
    /// value_grad_hess() with cell_partition()'s `keep`.
    double value_grad_hess_impl(
        const std::array<Eigen::Vector3d, 4>& t,
        int kv,
        double delta,
        Eigen::Vector3d& grad,
        Eigen::Matrix3d& hess,
        Stats* stats,
        Partition* keep) const;
    /// The value table (value()): slots of the 13 inputs and the result; locked by stripes of
    /// kValueStripes slots. Empty: no table.
    struct ValueSlot
    {
        double in[13];
        double out = 0.;
        bool full = false;
    };
    static constexpr size_t kValueStripes = 1024;
    mutable std::vector<ValueSlot> m_values;
    mutable std::unique_ptr<std::mutex[]> m_value_locks;
    /// _tri_partition: the same for the face f, its pieces planar rings.
    void tri_partition(const std::array<Eigen::Vector3d, 3>& f, TriWork& w) const;
    /// U: min over all triangles of max over the n points t of d_P.
    double upper_bound(const Eigen::Vector3d* t, int n) const;
    /// The checks of value() on t's pieces (volumes summed by the caller), and of max_distance()
    /// on f's pieces (areas): throw with the corners in hex floats if the pieces do not tile, or
    /// leave more outside every region, than the partition's tolerances allow.
    void check_cell_pieces(
        const std::array<Eigen::Vector3d, 4>& t,
        double delta,
        const Work& w,
        double vol_sum,
        double empty_vol) const;
    void check_face_pieces(
        const std::array<Eigen::Vector3d, 3>& f,
        const TriWork& w,
        const char* who) const;
    /// front_face()'s labelled parts of f into fw: the partition, each piece resolved to its
    /// nearest feature, the offset side kept.
    void label_face(const std::array<Eigen::Vector3d, 3>& f, FaceWork& fw) const;
    /// f's labelled parts, from `keep` when it holds f's, else made (into keep, or a scratch).
    const FaceWork& face_parts(const std::array<Eigen::Vector3d, 3>& f, FaceParts* keep) const;
    /// prepare_front_flux(): the kept polytopes' boxes and face planes {x : g.x <= h} (global
    /// coordinates), and a tree over the boxes (as m_nodes, over m_ff_order).
    struct FFBox
    {
        Eigen::Vector3d lo, hi;
        std::vector<std::pair<Eigen::Vector3d, double>> planes;
    };
    std::vector<FFBox> m_ff_boxes;
    std::vector<Node> m_ff_nodes;
    std::vector<int> m_ff_order;
    double m_ff_reach = 0.;
    int build_ff_node(int begin, int end);
    /// Feature indices whose box is within r of [lo, hi], ascending.
    void candidates(
        const Eigen::Vector3d& lo,
        const Eigen::Vector3d& hi,
        double r,
        std::vector<int>& out) const;
};

} // namespace wmtk::components::topological_offset
