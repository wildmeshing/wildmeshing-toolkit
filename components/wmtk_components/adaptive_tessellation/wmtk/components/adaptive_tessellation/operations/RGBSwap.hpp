#pragma once

#include <wmtk/attribute/Accessor.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>

#include <wmtk/operations/composite/TriEdgeSwap.hpp>

namespace wmtk::operations::composite {

class RGBSwap : public Operation
{
public:
    RGBSwap(
        Mesh& m,
        attribute::MeshAttributeHandle& triangle_rgb_state_handle,
        attribute::MeshAttributeHandle& edge_rgb_state_handle,
        attribute::MeshAttributeHandle& edge_todo_tag_handle);

    PrimitiveType primitive_type() const override { return PrimitiveType::Edge; }

    inline TriEdgeSwap& swap() { return m_swap; }

protected:
    std::vector<simplex::Simplex> unmodified_primitives(
        const simplex::Simplex& simplex) const override;
    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;

private:
    TriEdgeSwap m_swap;
    attribute::MeshAttributeHandle m_triangle_rgb_state_handle;
    attribute::MeshAttributeHandle m_edge_rgb_state_handle;
    attribute::MeshAttributeHandle m_edge_todo_tag_handle;
    attribute::Accessor<int64_t> m_triangle_rgb_state_accessor;
    attribute::Accessor<int64_t> m_edge_rgb_state_accessor;
    attribute::Accessor<int64_t> m_edge_todo_tag_accessor;
};

} // namespace wmtk::operations::composite
