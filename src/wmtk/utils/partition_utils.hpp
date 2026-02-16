#pragma once

#include <wmtk/TetMesh.h>
#include <Eigen/Core>

namespace wmtk {
void partition_vertex_morton(
    size_t vert_size,
    const std::function<Eigen::Vector3d(size_t)>& pos,
    int num_partition,
    std::vector<size_t>&);
}