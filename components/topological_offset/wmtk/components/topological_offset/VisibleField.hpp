#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

namespace wmtk::components::topological_offset {

/**
 * @brief What the visibility walk reads from a tet mesh.
 *
 * Each cell's four vertex ids, a vertex's position, a cell's kind, its neighbour across a face,
 * and the cells around an edge or a vertex. TopoOffsetTetMesh implements it on the mesh as it
 * is, on the mesh with one vertex at a trial position (the front smoother's Newton solve), and on
 * the mesh with a swap's candidate cells in place of the cells they would replace.
 */
class VisibilityCells
{
public:
    enum class Kind { Band, Input, Other };
    virtual ~VisibilityCells() = default;
    virtual std::array<int64_t, 4> vertices(int64_t cell) const = 0;
    virtual Eigen::Vector3d position(int64_t vertex) const = 0;
    virtual Kind kind(int64_t cell) const = 0;
    /// The cell across the face of `cell` opposite its local vertex j, -1 when there is none.
    virtual int64_t neighbor(int64_t cell, int j) const = 0;
    virtual void cells_around_edge(int64_t a, int64_t b, std::vector<int64_t>& out) const = 0;
    virtual void cells_around_vertex(int64_t v, std::vector<int64_t>& out) const = 0;
};

/**
 * @brief The band cells a query point lies on.
 *
 * Each with the mask of its local faces whose planes pass through the point (bit j: the face
 * opposite local vertex j): one face for a point inside a front face, the two faces through an
 * edge for a point on a front edge, the three faces through a vertex for a front vertex. The
 * point's position is rounded; these faces are its exact support, so the walk never asks which
 * side of them the point is on.
 */
struct VisibleStart
{
    std::vector<std::pair<int64_t, uint8_t>> cells;
    /// The point's exact position, when the caller has it: its support's vertices and their
    /// barycentric weights (renormalised exactly to sum 1). The point then lies exactly on its
    /// support, which its rounded coordinates do not; the walks that run on exact rationals
    /// start from it. Empty: the rounded coordinates are taken as exact.
    std::vector<std::pair<int64_t, double>> support;
};

/**
 * @brief EXPERIMENTAL_visible_distance: the distance from a point of the front to the nearest
 * input point it SEES through the band.
 *
 *     d_vis(p) = min |p - q| over input points q such that the segment [p, q] lies in closed
 *                band cells, except that it may end in input cells (the mesh's input cells
 *                stand off the input triangles by up to the input envelope)
 *
 * Wherever the segment to the nearest input point stays in the band, d_vis is the euclidean
 * distance d. Where it does not -- a front vertex of one side of a slot that has crossed the
 * slot's midline is nearest the OTHER wall, through cells that are not band -- d_vis measures to
 * the wall the vertex's band is attached to.
 *
 * Search: the input triangles' own AABB tree, branch and bound on distance, plus two exact
 * tests that discard whole boxes and triangles: (1) the cone test -- a box or triangle lying
 * strictly outside, for every start cell, one of that cell's faces through p, cannot be reached
 * by a segment that starts into the band; (2) the side test -- a triangle bounding the input
 * solid is reached only from its outer side. A triangle that passes both has its nearest point
 * q walked to from p, cell by cell (exact orientation predicates on the points as given). The
 * nearest triangle whose q is visible gives d_vis.
 *
 * A triangle whose q is hidden but which passed both tests has its nearest point among the
 * directions that enter the start cells computed instead (the triangle clipped by the start
 * cones, in exact rationals): every visible point of the triangle is among them, so when that
 * point is visible it is the triangle's nearest visible point (the first step was what hid q).
 *
 * HIDDEN FURTHER ALONG. If that point is hidden too -- by cells further along the segment, not at
 * its first step -- part of the triangle may still be visible, closer than every point proven
 * visible. d_vis then lies between two known values: the distance to that point (a lower bound:
 * every visible point of the triangle lies among the directions entering the start cells) and the
 * nearest point proven visible (an upper bound). The query returns the lower bound (how 3).
 * Measured on the slot at stage 1 (target 0.15): lower bound 0.416, exact 0.419 (floating-point
 * scan of the wall), proven visible 0.558.
 *
 * NOTHING VISIBLE. A point whose segments all leave the band (the far end of a tentacle of band
 * curving away from the input) has no d_vis. It takes the euclidean nearest input point instead,
 * so that it is still pulled toward the input; counted.
 *
 * ASSUMPTIONS -- where the returned value or its derivatives are not d_vis's, each counted where
 * it fires (Counts):
 *  1. Nothing visible: the euclidean nearest point (how 2). Uday, 2026-10-07, "for now".
 *  2. Hidden further along: the lower bound above (how 3), not the exact nearest visible point;
 *     the error is at most the gap to the nearest point proven visible, logged per turn (max gap;
 *     unbounded when nothing is proven visible). Uday, 2026-10-07. The alternative, exact
 *     shadows (the triangle minus the shadows of the cells the segments cross), is not built.
 *     The lower bound is NOT the euclidean distance to the whole input: a triangle proven hidden
 *     (at the first step, or by the side test) never supplies it -- that would be the other wall
 *     of a slot again.
 *  3. Derivatives (gradient(), hessian()) treat the foot as a fixed point of the triangle feature
 *     it lies on. Exact for how 0. For a foot found among the start cones (how 1, how 3) the
 *     motion of the cone planes with a moving front vertex is ignored, and the hessian is the
 *     feature's (0 inside a triangle), not that of |p - foot| -- although the gradient is.
 *  4. The caller's: the operations keep their local energy rule (see TopoOffsetTetMesh.h,
 *     m_visible_field), and the march keeps the euclidean distance.
 *
 * A walk that cannot be completed on exact rationals (no exit face found; a defect, or a trial
 * position whose cells are inverted) is not an assumption: the query reports Stop.
 */
class VisibleField
{
public:
    /// The input triangles (the complex's boundary and isolated triangles), and per triangle the
    /// sign of orient3d(a, b, c, x) for x inside the input solid, 0 for a triangle that bounds
    /// no solid (reachable from both sides). delta: target_distance, the field's unit.
    VisibleField(
        std::vector<Eigen::Vector3d> V,
        std::vector<Eigen::Vector3i> F,
        std::vector<int8_t> inner_sign,
        double delta);

    struct Feature
    {
        enum class Status { Found, NoneVisible, Stop };
        Status status = Status::NoneVisible;
        Eigen::Vector3d foot = Eigen::Vector3d::Zero();
        int dim = -1; ///< 2 triangle interior, 1 edge interior, 0 vertex
        Eigen::Vector3d dir = Eigen::Vector3d::Zero(); ///< the edge's unit direction (dim 1)
        double d = 0.;
        int64_t tri = -1; ///< the triangle (Stop: the one whose walk failed)
        /// How the point was found: 0 the triangle's own nearest point, 1 its nearest point
        /// among the directions entering the start cells, 2 nothing visible: euclidean,
        /// 3 hidden further along: the lower bound (assumption 2).
        int how = 0;
    };

    Feature nearest(
        const Eigen::Vector3d& p,
        const VisibleStart& start,
        const VisibilityCells& cells) const;

    /// The walk alone: whether the segment from p to q, q on triangle t, is visible.
    bool visible(
        const Eigen::Vector3d& p,
        const Eigen::Vector3d& q,
        int64_t t,
        const VisibleStart& start,
        const VisibilityCells& cells) const;

    /// The field from a Found feature, in EuclideanOffsetPotential's units (value d / delta,
    /// level 1): (d - delta) / delta, grad (d / delta), hess (d / delta) -- the same formulas,
    /// cased on the feature kind.
    double relative_residual(const Eigen::Vector3d& p, const Feature& f) const;
    Eigen::Vector3d gradient(const Eigen::Vector3d& p, const Feature& f) const;
    Eigen::Matrix3d hessian(const Eigen::Vector3d& p, const Feature& f) const;

    double delta() const { return m_delta; }
    size_t n_triangles() const { return m_F.size(); }

    /// Query counts since the last reset_counts(): all, resolved through the start cones (how 1),
    /// nothing visible (how 2, euclidean), hidden further along (how 3) with the largest gap to
    /// the nearest point proven visible and how many had none proven visible, stops.
    struct Counts
    {
        size_t queries = 0, first_step = 0, none_visible = 0, hidden_further = 0,
               hidden_further_unbounded = 0, stops = 0;
        double hidden_further_max_gap = 0.;
    };
    Counts counts() const { return m_counts; }
    void reset_counts() const { m_counts = Counts(); }

private:
    struct Node
    {
        Eigen::Vector3d lo, hi;
        int left = -1, right = -1; ///< children, -1 for a leaf
        int64_t tri = -1; ///< a leaf's triangle
    };
    int build(std::vector<int64_t>& ids, size_t b, size_t e);

    std::vector<Eigen::Vector3d> m_V;
    std::vector<Eigen::Vector3i> m_F;
    std::vector<int8_t> m_inner;
    double m_delta;
    std::vector<Node> m_nodes;
    int m_root = -1;
    mutable Counts m_counts; ///< serial only, as the key requires
    /// The euclidean nearest point of the input, for a point that sees none.
    Feature euclidean_nearest(const Eigen::Vector3d& p) const;
};

} // namespace wmtk::components::topological_offset
