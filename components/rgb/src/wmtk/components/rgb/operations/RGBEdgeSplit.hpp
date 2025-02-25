
#pragma once

#include <wmtk/operations/EdgeSplit.hpp>

namespace wmtk::components::rgb::operations {
class RGBEdgeSplit : public wmtk::operations::EdgeSplit
{
    RGBEdgeSplit(
        Mesh& m,
        const attribute::MeshAttributeHandle& tri_level,
        const attribute::MeshAttributeHandle& edge_level);
};
} // namespace wmtk::components::rgb::operations
