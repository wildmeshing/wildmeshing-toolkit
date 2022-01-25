#pragma once
#include <tbb/concurrent_vector.h>
#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>
#include <wmtk/utils/Partitioning.h>

namespace wmtk {

tbb::concurrent_vector<size_t> partition_TriMesh(wmtk::TriMesh& m, int num_partition);

tbb::concurrent_vector<size_t> partition_TetMesh(wmtk::TetMesh& m, int num_partition);


} // namespace wmtk