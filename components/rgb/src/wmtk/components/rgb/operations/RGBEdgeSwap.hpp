#pragma once
#include <wmtk/operations/composite/TriEdgeSwap.hpp>

namespace wmtk::components::rgb::operations {
class RGBTriEdgeSwap : public wmtk::operations::composite::TriEdgeSwap
{
public:
    RGBTriEdgeSwap(
        Mesh& m,
        const attribute::MeshAttributeHandle& tri_level,
        const attribute::MeshAttributeHandle& edge_level);

private:
};
} // namespace wmtk::components::rgb::operations
