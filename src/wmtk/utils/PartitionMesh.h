#pragma once
#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>
#include <wmtk/utils/Morton.h>
#include <wmtk/utils/Partitioning.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>

namespace wmtk {

std::vector<size_t> partition_TriMesh(const wmtk::TriMesh& m, int num_partition);
std::vector<size_t> partition_TetMesh(wmtk::TetMesh& m, int num_partition);
std::vector<size_t> partition_TriMesh_morton(const wmtk::TriMesh& m, int num_partition);
std::vector<size_t> partition_TetMesh_morton(wmtk::TetMesh& m, int num_partition);
std::vector<size_t> partition_morton(std::vector<Eigen::Vector3d> vertex_position, int NUM_THREADS);

void resorting(
    const std::vector<Eigen::Vector3d>& Vori,
    const std::vector<Eigen::Vector3i>& F,
    std::vector<Eigen::Vector3i>& fnew,
    int NUM_THREADS);

} // namespace wmtk