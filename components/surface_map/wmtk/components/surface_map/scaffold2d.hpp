#pragma once

#include <Eigen/Core>

#include <array>
#include <vector>

namespace wmtk::components::surface_map {

/**
 * @brief The scaffold of a joint flattening (Jiang et al. 2017, as BijectiveRemesh uses it): the
 * region between a simple polygon and a box around it, triangulated with every edge of the
 * polygon an edge of the triangulation.
 *
 * A constrained Delaunay triangulation of the polygon's vertices and the box's corners, from
 * VolumeRemesher's 2D arrangement: Delaunay with exact predicates, then each polygon edge forced
 * in by flipping the edges it crosses. The polygon being simple, nothing crosses it, so the only
 * vertices added are corners: the box's, and the four the arrangement adds a tenth beyond the
 * points' bounding box. The scaffold is the triangles reached from those without crossing the
 * polygon.
 *
 * @param polygon  the polygon's vertices, in order around it
 * @param box_min, box_max  a box holding the polygon strictly inside
 * @param[out] corners  the vertices the triangulation adds
 * @param[out] triangles  the scaffold's triangles, positively oriented: a vertex id below
 *                        polygon.size() is a vertex of the polygon, the others index `corners`
 *                        after them
 * @return false if the polygon is not simple: two vertices coincide, or edges cross
 */
bool box_scaffold(
    const std::vector<Eigen::Vector2d>& polygon,
    const Eigen::Vector2d& box_min,
    const Eigen::Vector2d& box_max,
    std::vector<Eigen::Vector2d>& corners,
    std::vector<std::array<int, 3>>& triangles);

} // namespace wmtk::components::surface_map
