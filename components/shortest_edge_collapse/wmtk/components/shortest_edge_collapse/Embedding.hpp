#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstddef>
#include <vector>

namespace wmtk::components::shortest_edge_collapse {

/**
 * @brief Do the closed triangles A and B meet anywhere but in the vertices and the edge they
 * share? Exact: orient3d and orient2d signs only (wmtk/utils/predicates.hpp), no arithmetic on
 * the coordinates.
 *
 * Vertices are shared by id (ia, ib), not by position: two vertices at one position are two
 * points of the mesh that coincide, which this reports as a meeting. Both triangles must be
 * non-degenerate (predicates::is_degenerate false); a degenerate one is not embedded on its own
 * and is the caller's to count.
 *
 * By the number of shared vertices:
 * - none: they may not meet at all. Two closed triangles meet iff an edge of one meets the
 *   other, so six closed segment-triangle tests decide it; coplanar ones in a coordinate plane
 *   the triangle projects onto without degenerating.
 * - one, v: they may meet only in v. Iff they meet elsewhere, a point of the boundary of one,
 *   other than v, lies in the other: on the edge opposite v (a segment-triangle test, which
 *   cannot hit v, as the triangle is not degenerate), or on an edge out of v, which then runs
 *   from v into the other triangle -- coplanar with it, and inside its corner at v.
 * - two, an edge: they may meet only in that edge. Iff they meet elsewhere, they are coplanar
 *   with the two other vertices on one side of the edge (a flat fold).
 * - three: the same triangle twice, which is not embedded.
 */
bool triangles_meet_outside_shared(
    const std::array<size_t, 3>& ia,
    const std::array<Eigen::Vector3d, 3>& A,
    const std::array<size_t, 3>& ib,
    const std::array<Eigen::Vector3d, 3>& B);

/// What keeps a triangle mesh from being embedded.
struct Crossings
{
    /// pairs of non-degenerate triangles that meet outside the vertices and the edge they share
    size_t pairs = 0;
    /// triangles whose three corners are collinear, left out of the pairs
    size_t degenerate = 0;
};

/**
 * @brief The crossings of the triangles F on the vertices V, exactly: every pair of triangles
 * whose boxes meet (BoxTree, so none is missed) is decided by triangles_meet_outside_shared().
 * The triangles are split among num_threads threads, the rule of threading::parallel_for.
 */
Crossings count_crossings(
    const std::vector<Eigen::Vector3d>& V,
    const std::vector<std::array<size_t, 3>>& F,
    int num_threads);

/**
 * @brief Boxes of the items 0..n-1, in a bounding volume hierarchy, for finding the items whose
 * boxes meet a box.
 *
 * Exact: a node's box is the componentwise min and max of its children's, and boxes are compared
 * with <=, so no item whose box meets the query box is ever missed -- only the tree's shape,
 * which no answer depends on, is chosen in floating point (median splits of the box centres).
 */
class BoxTree
{
public:
    using Box = Eigen::AlignedBox3d;

    BoxTree() = default;
    explicit BoxTree(const std::vector<Box>& boxes);

    /// Calls visit(item) for the items whose boxes meet `q`, closed boxes, until one call
    /// returns true; whether one did. An empty Box() meets nothing.
    template <typename Visit>
    bool any(const Box& q, Visit&& visit) const
    {
        return !m_nodes.empty() && any(0, q, visit);
    }

private:
    struct Node
    {
        Box box;
        int left = -1; // -1: a leaf, of `item`
        int right = -1;
        size_t item = 0;
    };
    std::vector<Node> m_nodes; // the root first

    int build(const std::vector<Box>& boxes, size_t* begin, size_t* end);

    template <typename Visit>
    bool any(const int n, const Box& q, Visit& visit) const
    {
        const Node& node = m_nodes[n];
        if (!node.box.intersects(q)) return false;
        if (node.left < 0) return visit(node.item);
        return any(node.left, q, visit) || any(node.right, q, visit);
    }
};

} // namespace wmtk::components::shortest_edge_collapse
