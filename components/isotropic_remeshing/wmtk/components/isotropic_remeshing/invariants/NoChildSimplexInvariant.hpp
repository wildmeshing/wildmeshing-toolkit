#pragma once

#include <wmtk/invariants/Invariant.hpp>

namespace wmtk::components::isotropic_remeshing::invariants {
/**
 * This invariant returns true if the given simplex is not mapping to any child mesh.
 */
class NoChildSimplexInvariant : public Invariant
{
public:
    NoChildSimplexInvariant(const Mesh& parent_mesh, const Mesh& child_mesh);
    NoChildSimplexInvariant(const Mesh& parent_mesh, const Mesh& child_mesh, PrimitiveType pt);
    bool before(const simplex::Simplex& s) const override;
    bool before_same(const simplex::Simplex& s) const;
    bool before_lower(const simplex::Simplex& s) const;

private:
    const Mesh& m_child_mesh;
    // the type of primitive we are blocking if one exists nearby
    PrimitiveType m_primitive_type;
};
} // namespace wmtk::components::isotropic_remeshing::invariants
