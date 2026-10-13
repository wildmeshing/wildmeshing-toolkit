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
 * (the band cell term).
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
 * Thread safety: value() is const and keeps no state; concurrent calls are safe.
 */
class ExactBandIntegral
{
public:
    /// V (n x 3) and F (m x 3): the input triangles. Vertices are merged by exact coordinates and
    /// sorted (numpy.unique, as the Python Mesh does), which fixes the order the features are
    /// visited in -- the rule-(a) pieces depend on it.
    ExactBandIntegral(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);

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
        int64_t point_faces = 0; ///< faces integrated against a point distance
        int64_t line_faces_radial = 0; ///< ... against a line distance, radial closed form
        int64_t line_faces_parallel = 0; ///< ... line exactly parallel to the face
        int64_t line_faces_series = 0; ///< ... near-parallel series
        int64_t closed_edges = 0; ///< polygon edges integrated by a closed form
        int64_t series_short = 0; ///< near-parallel edges, Taylor series along the edge
        int64_t series_long = 0; ///< near-parallel edges, Taylor series in eta
        /// Times in seconds, measured only when `timing` is set (the clock reads add a few percent,
        /// mac-mini). t_candidates: the U search, the box query and the local feature setup;
        /// t_clip: every half-space clip; t_closed: every face integral (point and line
        /// distance), INCLUDING t_series, the near-parallel series edges inside them.
        bool timing = false;
        double t_total = 0., t_candidates = 0., t_clip = 0., t_closed = 0., t_series = 0.;
    };

    /// J(t) = int_t (d - delta) dV for the cell with corners t (either orientation); 0 for a
    /// flat cell (|6 vol| <= 64 eps L^3, L its longest edge). With `stats`, adds to it.
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
     * Flat cells give J = 0 and zero derivatives.
     */
    double value_grad_hess(
        const std::array<Eigen::Vector3d, 4>& t,
        int kv,
        double delta,
        Eigen::Vector3d& grad,
        Eigen::Matrix3d& hess,
        Stats* stats = nullptr) const;

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

    /// Per-call scratch of the partition (defined in the .cpp).
    struct Work;
    /// _partition: the candidates in t's frame and the convex pieces of t with member sets; false
    /// for a flat cell.
    bool partition(const std::array<Eigen::Vector3d, 4>& t, Work& w, Stats* stats) const;
    /// U: min over all triangles of max over the corners of d_P.
    double upper_bound(const std::array<Eigen::Vector3d, 4>& t) const;
    /// Feature indices whose box is within r of [lo, hi], ascending.
    void candidates(
        const Eigen::Vector3d& lo,
        const Eigen::Vector3d& hi,
        double r,
        std::vector<int>& out) const;
};

} // namespace wmtk::components::topological_offset
