#include "SeamlessCollapseInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/utils/SeamlessConstraints.hpp>
namespace wmtk::invariants {
SeamlessCollapseInvariant::SeamlessCollapseInvariant(
    const TriMesh& m,
    std::shared_ptr<TriMesh> uv_mesh,
    const TypedAttributeHandle<double>& uv_handle)
    : Invariant(m)
    , m_uv_mesh(uv_mesh)
    , m_uv_handle(uv_handle)
{}
bool SeamlessCollapseInvariant::before(const simplex::Simplex& t) const
{
    // TODO: implement this
    return true;
}
} // namespace wmtk::invariants
