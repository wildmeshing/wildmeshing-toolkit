#include "HybridRationalIsRationalInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/attribute/utils/HybridRationalAccessor.hpp>
#include <wmtk/simplex/Simplex.hpp>
namespace wmtk {
HybridRationalIsRationalInvariant::HybridRationalIsRationalInvariant(
    const attribute::MeshAttributeHandle& coordinate)
    : Invariant(coordinate.mesh(), true, false, false)
    , m_coordinate_handle(coordinate)
{}
HybridRationalIsRationalInvariant::HybridRationalIsRationalInvariant(
    const Mesh& m,
    const attribute::utils::HybridRationalAttribute<>& coordinate)
    : HybridRationalIsRationalInvariant(attribute::MeshAttributeHandle(m, coordinate))
{}

bool HybridRationalIsRationalInvariant::before(const simplex::Simplex& t) const
{
    return !attribute::utils::HybridRationalAccessor(m_coordinate_handle).is_rounded(t.tuple());
}
} // namespace wmtk
