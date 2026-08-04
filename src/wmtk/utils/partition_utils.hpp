#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <functional>
#include <vector>

namespace wmtk {

/**
 * @brief Assign each vertex a partition id by sorting the vertices along a Morton curve
 * and cutting the resulting order into `num_partition` equal runs.
 *
 * Vertices close in space end up in the same partition, which is what the parallel
 * scheduler needs to keep its two-ring locks from colliding.
 *
 * 2D callers pass their positions as (x, y, 0): the z terms then contribute nothing to the
 * bounding box, to the scale or to the Morton code, so the result is exactly what a
 * 2D-specific implementation produces.
 *
 * @param vert_size     number of vertices (ids 0..vert_size-1)
 * @param pos           position of vertex i
 * @param num_partition number of partitions, also the thread count used internally
 * @param[out] result   partition id per vertex; resized to vert_size
 */
void partition_vertex_morton(
    size_t vert_size,
    const std::function<Eigen::Vector3d(size_t)>& pos,
    int num_partition,
    std::vector<size_t>& result);

} // namespace wmtk
