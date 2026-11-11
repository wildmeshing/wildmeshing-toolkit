#pragma once

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

namespace wmtk::components::topological_offset {

/**
 * @brief The band-area integral of one triangle, in closed form -- the 2D twin of
 * ExactBandIntegral (2026-11-10, the 2D lock-step port):
 *
 *     J(t) = int_t (d(x) - delta) dA,   d = Euclidean distance to the input,
 *
 * the input a complex of segments and points in the plane (a 2D offset's input complex: the
 * boundary of a tagged region, open curves, isolated points). The algorithm, its constants and
 * its tolerances are ExactBandIntegral's one dimension down; this doc states where they differ.
 *
 * Features and local regions. The features are the segments' interiors (edges) and the vertices
 * (segment ends and points, merged by exact coordinates and sorted, as 3D's). An edge's region
 * is its slab between the lines through its ends perpendicular to it; a vertex's region is the
 * cone {x : (x - v).(w - v) <= 0 for every edge (v, w)}, all of the plane for an isolated point.
 * d = min over {g : x in L_g} of d_g is exact, d_g the line or point distance of g, by 3D's two
 * facts. Where 2D differs: an edge's line distance |n.(x - a)| is affine on each side of its line,
 * so a 2D edge takes the role of a 3D face in every integral and split (3D's line distance has
 * no 2D counterpart), and the slab is not cut by incident faces (a segment has none).
 *
 * Per triangle, in its own frame (origin the centroid, as 3D's cell; the same reason):
 *  0. Candidates: U = min over the primitives P (segments, points) of max over t's corners of
 *     d_P bounds d on t (d_P convex); every feature whose box is within U of t's box is kept.
 *  1. t is cut by the candidates' regions into convex pieces of constant member set; members
 *     that cannot attain the minimum (lower bound >= another member's upper bound) are dropped.
 *  2. Exact: one member; all members edges, split by their lines and then by the bisectors of
 *     their (then affine) distances, as 3D splits a piece whose members are all faces -- two lines
 *     in the plane meet or are parallel, so every edge pair is exact here, unlike 3D's skew edges;
 *     all members vertices, split by their perpendicular bisectors.
 *  3. Otherwise -- an edge and a vertex not its own, whose bisector is a parabola -- "rule (a)":
 *     the piece takes min over its members g of int_piece d_g >= int_piece d, an upper bound, as
 *     3D's curved bisectors do. Stats::rule_a_area reports its area.
 * The pieces must tile t and leave no more of it outside every region than the partition's
 * tolerances allow; value() throws otherwise, with t in hex floats (3D's check_cell_pieces()).
 *
 * One-feature integrals: an affine distance is area times its centroid value; a point distance
 * by Euler's theorem for degree-1 homogeneous integrands, int_Q |x - v| dA = 1/3 sum over Q's
 * edges e of h_e int_e |x - v| ds, h_e the signed distance from v to e's line, the edge integral
 * in 3D's rationalized form (int_sqrt()). 3D's conditioning remark holds: a piece of size L at
 * distance D from its vertex loses about eps (D/L)^2 in the Euler sum.
 *
 * The offset side, as 3D's: constructed with per-segment sides (+1 if the segment's normal
 * (b - a) rotated clockwise points out of the solid the segments bound, -1 into it, 0 no solid)
 * and sigma (+1: the band is outside the solid, -1 inside, 0: no clip), value() integrates only
 * the offset side. The side of x is decided by its nearest feature g, by the sign of
 * (x - q_g).N_g: N_g the segment's outward normal (edge), the sum of its two segments' unit
 * outward normals (vertex, the plane's pseudonormal; 3D's angle weights are equal in the plane).
 * A vertex has a side only with exactly two segments, both sided, as a 3D edge only with exactly
 * two faces. Where 2D differs: 3D's segments bound no solid and never have a side; a 2D segment
 * is a 3D triangle's twin here. The clip rules are 3D's (class doc there): a one-member part is
 * clipped by its member's line, an edges-only part kept or dropped whole, a rule-(a) piece split
 * by its members' lines with a disagreeing part taking the side of the member nearest at its
 * centroid. With nothing clipped J = int_t d - delta area(t), as 3D keeps.
 *
 * value() sums the elementary pieces (pieces on which the integrand is one smooth function) that
 * value_grad_hess() differentiates, so the two agree bit for bit. Where 2D differs: 3D's
 * evaluate() integrates the pieces directly and captures the elementary pieces into a Partition
 * for the derivative call that follows (a speed device MEASURED on 3D cells); one build serves
 * both here until a 2D measurement asks for the capture.
 *
 * Thread safety: every member is const and keeps no state between calls.
 */
class ExactBandIntegral2D
{
public:
    /// V (n x 2 or n x 3, the first two columns read), E (m x 2, rows of V) the segments, P the
    /// points (rows of V); a segment whose ends merged is a point. segment_side: per row of E (see
    /// the class doc), required (one per row) when sigma != 0; sigma the band's side of the solid,
    /// 0 for no clip.
    ExactBandIntegral2D(
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXi& E,
        const std::vector<int>& P,
        const std::vector<int8_t>& segment_side = {},
        int sigma = 0);

    /// Counters and areas, ADDED to by every value() and value_grad_hess() call that is given one.
    struct Stats
    {
        int64_t cells = 0; ///< calls with a non-flat triangle
        int64_t candidates = 0; ///< candidate features
        int64_t pieces = 0; ///< convex pieces of the partition
        int64_t rule_a_pieces = 0;
        double area = 0.; ///< triangle area
        double rule_a_area = 0.; ///< area of the rule-(a) pieces (an upper bound there)
        double empty_area = 0.; ///< area of pieces with no member (must be rounding)
        int64_t side_clipped = 0; ///< calls with anything clipped by the offset side
    };

    /// J(t) for the triangle with corners t (either orientation), on the offset side when
    /// constructed with one; 0 for a flat triangle (|2 area| <= 64 eps L^2, L its longest edge).
    double value(const std::array<Eigen::Vector2d, 3>& t, double delta, Stats* stats = nullptr)
        const;

    /**
     * @brief J and its gradient and Hessian with respect to corner `kv` of t, the other two fixed:
     * 3D's value_grad_hess() one dimension down. Reynolds' transport theorem on each elementary
     * piece Q, a boundary integral over the two edges of t through x = t[kv] (outward unit normal
     * n, length |N|, lambda the hat function of x):
     *     grad += n int_{Q cap e} lambda phi ds,
     *     hess += n (x) int_{Q cap e} lambda^2 grad phi ds + (int_{Q cap e} lambda phi ds / |N|) dN/dx
     *             - sum over the ends y of Q cap e that are not corners of t:
     *               n (x) g lambda(y)^2 phi(y) / |g - (g.n) n|,
     * phi the piece's integrand, g the outward normal of Q's edge through y ("cut points", 3D's
     * cut edges: they cancel in pairs where phi is continuous and leave its jump where it is not).
     * A piece touching e at one vertex only adds that vertex's two cut terms (3D's contact term).
     * Where the derivative is not the derivative of J, as in 3D: on rule-(a) pieces it is that of
     * the bound (the chosen member's integral); with a kink of d on an edge of t through x, the
     * one-sided limit for x moving into t. Flat triangles give J = 0 and zero derivatives.
     * Layout and orientation as 3D's: one corner at a time, grad its 2-vector, hess its 2 x 2
     * block; t in either orientation.
     */
    double value_grad_hess(
        const std::array<Eigen::Vector2d, 3>& t,
        int kv,
        double delta,
        Eigen::Vector2d& grad,
        Eigen::Matrix2d& hess,
        Stats* stats = nullptr) const;
    /// value_grad_hess()'s gradient and Hessian alone (J's integration over the pieces skipped),
    /// as 3D's grad_hess(): the energy reads J from value() and the derivatives from here.
    void grad_hess(
        const std::array<Eigen::Vector2d, 3>& t,
        int kv,
        double delta,
        Eigen::Vector2d& grad,
        Eigen::Matrix2d& hess) const;

    /**
     * @brief The band integral's dependence through one boundary edge of the band, for the
     * motion of its corner kv (the other fixed): the 2D twin of ExactBandIntegral::
     * front_face_derivs() under a CurvedBisectorRuleA, EXPERIMENTAL_band_volume_rule
     * "exact_swept"'s search model (BandVolumeEnergy2D::set_swept()). f = (f0, f1) ordered so
     * that its normal n = rot_cw(f1 - f0) / |.| points out of the band, x = f[kv]:
     *     grad = n int_f chi (d - delta) phi ds,
     *     hess = n (x) int_f chi phi^2 grad d ds + (int_f chi (d - delta) phi ds / |N|) dN/dx
     *            - sum over cut points c: n (x) g_c (d - delta)(c) phi(c)^2 / |g_c,f|,
     *     hess_swept = (n (x) Q + Q (x) n) / 2,
     *            Q = int_f chi phi^2 grad d ds - sum over c: g_c (d - delta)(c) phi(c)^2 / |g_c,f|,
     * phi the hat function of x along f, chi the offset side's indicator, N = rot_cw(f1 - f0),
     * the cut points the ends of f's parts that are not ends of f, each with its part's own
     * integrand (g_c the normal of the line that cut f there, out of the part, g_c,f its part
     * along f). As in 3D: summed over the band's boundary edges through x, grad and hess are the
     * band integral's gradient and Hessian in x (Reynolds' transport theorem, no medial-axis
     * term, d being Lipschitz); hess_swept is the Hessian at w = 0 of the integral over the swept
     * triangle conv(x, x + w, f[1 - kv]) signed by w.n, its labels f's; a part of f on the input
     * takes grad d from the band side (-n) unless it is the offset side of a sided segment.
     *
     * f's parts, labelled as value() labels the pieces of a thin swept triangle over f (3D's
     * label_face()): f cut by the candidates' regions into spans with member sets (inside first
     * where f lies on a region's line, as 3D's face partition), the members pruned on the span's
     * ends, then one member, all edges (split by their lines, then by their bisectors), all
     * vertices (split by their perpendicular bisectors), or rule (a): the span split by the
     * members' side lines, each part labelled by the member of smallest int_part d_g phi ds (the
     * first on a tie), a part where the lines disagree taking the side of the member nearest at
     * its centroid in that limit (below). So summed over the boundary edges, grad at x is the
     * exact derivative at x of the band change "exact_swept" judges a move from x on, where
     * value() and these labels cut alike. `rule_a`: the number of parts labelled by rule (a).
     *
     * Where 2D differs, and why:
     *  - a curved bisector never throws: the parts are always labelled by rule (a), 3D's
     *    behaviour under a CurvedBisectorRuleA, which "exact_swept"'s search model always opens;
     *    3D's throwing mode serves "front_flux", which has no 2D port. So no CurvedBisectorRuleA:
     *    the corner whose hat function weighs rule (a)'s integrals is kv;
     *  - the centroid that decides a part whose members' side lines disagree is the thin swept
     *    triangle's in the limit, the phi-weighted centroid of the span, where 3D's label_face()
     *    takes the face part's own area centroid: value() decides by the swept part's centroid,
     *    which lies over the phi-weighted one;
     *  - no labelled parts are kept between calls (3D's FaceParts) and there is no front_face():
     *    the 2D integral keeps no partitions (class doc) and has no front form;
     *  - a span lying on a bisector line takes its lower member, as value() breaks a tie between
     *    parallel lines (3D's resolve() keeps both sides of a ring on a bisector plane).
     * Throws (f in hex floats) if the parts do not tile f, or leave more of f outside every
     * region, than the tolerances allow (3D's check_face_pieces(), on lengths).
     */
    void front_edge_derivs(
        const std::array<Eigen::Vector2d, 2>& f,
        double delta,
        int kv,
        Eigen::Vector2d& grad,
        Eigen::Matrix2d& hess,
        int* rule_a = nullptr,
        Eigen::Matrix2d* hess_swept = nullptr) const;

    /// d(x), the distance to the nearest segment or point.
    double distance(const Eigen::Vector2d& x) const { return upper_bound(&x, 1); }
    /// U(t): min over the primitives of the max of their distance over t's corners, >= d on t.
    double distance_bound(const std::array<Eigen::Vector2d, 3>& t) const
    {
        return upper_bound(t.data(), 3);
    }
    /// The band term's corner bound (EXPERIMENTAL_band_volume_rule "corner_bound",
    /// TopoOffsetTriMesh::band_cell_corner_bound()): min over ALL primitives P of (1/n) sum_i
    /// d_P(t_i), n points, and the minimiser in `best`, the lowest index on a tie (segments, then
    /// points, in row order). 3D's min_mean_distance(), whose sum, order and comparison it keeps;
    /// where 2D differs: 3D searches the input envelope's primitives in a grown box, the 2D input
    /// has no envelope over its primitives, and this scans them all as upper_bound() does. The
    /// result is the same min either way.
    double min_mean_distance(const Eigen::Vector2d* t, int n, int64_t& best) const;
    /// d_P(x), primitive P alone (min_mean_distance()'s index and arithmetic), with its gradient
    /// and Hessian in x: Envelope::primitive_distance()'s 2D derivatives, (x - foot) / d, and 0
    /// on a segment's interior (d affine there), (I - u u^T) / d at an end or a point; both 0 at
    /// d = 0. The corner bound's derivatives (BandVolumeEnergy2D) read them, as 3D's read the
    /// input envelope's.
    double primitive_distance(
        int64_t P,
        const Eigen::Vector2d& x,
        Eigen::Vector2d* grad = nullptr,
        Eigen::Matrix2d* hess = nullptr) const;

    size_t num_segments() const { return size_t(m_nE); }
    size_t num_vertices() const { return m_feat.size() - size_t(m_nE); }

private:
    /// A feature: kind 1 edge (p = its first end, dir = its unit normal, (b - a) rotated
    /// counter-clockwise), 2 vertex (p). Its local region is {x : g.(x - q) <= 0} over (g, q) in
    /// m_hs[h_begin, h_end). Kinds numbered as 3D's (0, a face, has no 2D counterpart).
    struct Feature
    {
        int kind = 1;
        Eigen::Vector2d p, dir;
        int h_begin = 0, h_end = 0;
        Eigen::Vector2d lo, hi; ///< bounding box
    };
    std::vector<Feature> m_feat; ///< edges, then vertices
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> m_hs; ///< (unit normal g, point q)
    int m_nE = 0;
    /// The primitives, for d_P: the segments' ends, then the points' (both ends equal).
    std::vector<Eigen::Vector2d> m_SA, m_SB;
    /// The offset side (class doc): sigma (0: none), and per feature its side line, the unit
    /// normal N out of the solid and a point q on it; m_side_has[g] false where it has none.
    int m_sigma = 0;
    std::vector<char> m_side_has;
    std::vector<Eigen::Vector2d> m_side_n, m_side_q;

    struct Work;
    /// The candidates in t's frame and the convex pieces of t with member sets; false: t is flat.
    bool partition(const std::array<Eigen::Vector2d, 3>& t, Work& w, Stats* stats) const;
    struct EdgeWork;
    /// front_edge_derivs()'s labelled parts of f, rule (a)'s integrals weighted by the hat
    /// function of corner kv (3D's tri_partition() and label_face()); nothing for a point.
    void label_edge(const std::array<Eigen::Vector2d, 2>& f, int kv, EdgeWork& w) const;
    /// value(), and with kv >= 0 value_grad_hess() (want_value false: grad_hess(), J not summed).
    double evaluate(
        const std::array<Eigen::Vector2d, 3>& t,
        double delta,
        int kv,
        Eigen::Vector2d* grad,
        Eigen::Matrix2d* hess,
        bool want_value,
        Stats* stats) const;
    /// U: min over all primitives of max over the n points t of d_P.
    double upper_bound(const Eigen::Vector2d* t, int n) const;
};

} // namespace wmtk::components::topological_offset
