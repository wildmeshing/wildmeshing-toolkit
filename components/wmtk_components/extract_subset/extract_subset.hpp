#pragma once

#include <vector>
#include "wmtk/Mesh.hpp"
#include <wmtk/TriMesh.hpp>
#include "wmtk/Primitive.hpp"
#include "internal/extract_subset_2d.hpp"

namespace wmtk::components{
    wmtk::TriMesh extract_subset(long dimension, const wmtk::TriMesh& m, std::vector<size_t> tag);
} // namespace wmtk::components