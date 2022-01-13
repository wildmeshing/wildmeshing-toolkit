#pragma once
#include <tbb/concurrent_vector.h>
#include <wmtk/TriMesh.h>
#include <wmtk/utils/Partitioning.h>

namespace wmtk {

tbb::concurrent_vector<int> partition_TriMesh(wmtk::TriMesh m, int num_partition);


} // namespace wmtk