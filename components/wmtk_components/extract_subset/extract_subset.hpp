#pragma once

#include <wmtk/TriMesh.hpp>
#include <vector>

namespace wmtk::components{
    wmtk::TriMesh extract_subset(long dimension, const wmtk::TriMesh& m, std::vector<size_t> tag);
} // namespace wmtk::components