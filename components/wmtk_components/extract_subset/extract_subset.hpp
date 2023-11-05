#pragma once

#include <vector>
#include "wmtk/Mesh.hpp"
#include <wmtk/TriMesh.hpp>
#include "wmtk/Primitive.hpp"
#include "internal/extract_subset_2d.hpp"
#include <wmtk/attribute/AttributeHandle.hpp>

namespace wmtk{

namespace components{
    // wmtk::TriMesh extract_subset(const wmtk::TriMesh& m, std::vector<size_t> tag, long dimension);
    wmtk::TriMesh extract_subset(wmtk::TriMesh m, wmtk::MeshAttributeHandle<long> tag_handle, long dimension);
}
} // namespace wmtk::components