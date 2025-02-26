#pragma once
#include <wmtk/operations/composite/TriEdgeSwap.hpp>

namespace wmtk::components::rgb::operations {
class RGBTriEdgeSwap : public wmtk::operations::composite::TriEdgeSwap
{
public:
    RGBTriEdgeSwap(
        TriMesh& m,
        const attribute::MeshAttributeHandle& tri_level,
        const attribute::MeshAttributeHandle& edge_level);
    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;

private:
    TriMesh& mesh();
    const TriMesh& mesh() const;
    attribute::MeshAttributeHandle m_triangle_level_handle;
    attribute::MeshAttributeHandle m_edge_level_handle;
};
} // namespace wmtk::components::rgb::operations
