#pragma once

#include <optional>
#include <wmtk/attribute/TypedAttributeHandle.hpp>
#include "Invariant.hpp"

namespace wmtk::invariants {
class FrozenVertexInvariant : public Invariant
{
public:
    FrozenVertexInvariant(const Mesh& m, const TypedAttributeHandle<int64_t>& frozen_vertex_handle);
    bool before(const simplex::Simplex& t) const override;

private:
    const TypedAttributeHandle<int64_t> m_frozen_vertex_handle;
};

class FrozenOppVertexInvariant : public Invariant
{
public:
    FrozenOppVertexInvariant(
        const Mesh& m,
        const TypedAttributeHandle<int64_t>& frozen_vertex_handle);
    bool before(const simplex::Simplex& t) const override;

private:
    const TypedAttributeHandle<int64_t> m_frozen_vertex_handle;
};


} // namespace wmtk::invariants
