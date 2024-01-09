#pragma once

#include <wmtk/TriMesh.hpp>
#include <wmtk/attribute/AttributeHandle.hpp>
#include "Invariant.hpp"

namespace wmtk {
namespace invariants {
class SeamlessCollapseInvariant : public Invariant
{
public:
    using Invariant::Invariant;
    SeamlessCollapseInvariant(
        const TriMesh& m,
        std::shared_ptr<TriMesh> uv_mesh,
        const TypedAttributeHandle<double>& uv_handle);
    bool before(const simplex::Simplex& t) const override;

private:
    std::shared_ptr<TriMesh> m_uv_mesh;
    const TypedAttributeHandle<double> m_uv_handle;
};
} // namespace invariants
} // namespace wmtk
