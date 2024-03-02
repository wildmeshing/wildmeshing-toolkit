#pragma once

#include <wmtk/attribute/Accessor.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>

namespace wmtk::operations::composite {

class RGRefine : public Operation
{
public:
    RGRefine(Mesh& m, const attribute::MeshAttributeHandle& edge_length_handle);

    PrimitiveType primitive_type() const override { return PrimitiveType::Edge; }

    inline EdgeSplit& split() { return m_split; }
    inline EdgeSplit& second_split() { return m_second_split; }
    inline TriEdgeSwap& swap() { return m_swap; }

protected:
    std::vector<simplex::Simplex> unmodified_primitives(
        const simplex::Simplex& simplex) const override;
    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;

private:
    EdgeSplit m_split;
    EdgeSplit m_second_split;
    TriEdgeSwap m_swap;
    attribute::MeshAttributeHandle m_edge_length_handle;
};

} // namespace wmtk::operations::composite
