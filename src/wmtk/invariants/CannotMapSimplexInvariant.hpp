#pragma once

#include <wmtk/invariants/Invariant.hpp>

namespace wmtk::invariants {
/**
 * This invariant returns true if the given simplex, or one of its neighbors can be mapped to a
 * child mesh.
 */
class CannotMapSimplexInvariant : public Invariant
{
public:
    CannotMapSimplexInvariant(const Mesh& parent_mesh, const Mesh& child_mesh);
    CannotMapSimplexInvariant(
        const Mesh& parent_mesh,
        const Mesh& child_mesh,
        PrimitiveType mapped_simplex_type);
    bool before(const simplex::Simplex& s) const override;
    bool before_same_dimension(const simplex::Simplex& s) const;
    bool before_default(const simplex::Simplex& s) const;
    bool before_same_dimension_vertex(const simplex::Simplex& s) const;
    bool before_same_dimension_edge(const simplex::Simplex& s) const;

private:
    const Mesh& m_child_mesh;
    // the type of primitive we are blocking if one exists nearby
    PrimitiveType m_mapped_simplex_type;
};
} // namespace wmtk::invariants
