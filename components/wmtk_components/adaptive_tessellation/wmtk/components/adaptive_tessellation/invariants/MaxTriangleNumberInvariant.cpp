#include "MaxTriangleNumberInvariant.hpp"


namespace wmtk {

MaxTriangleNumberInvariant::MaxTriangleNumberInvariant(
    const Mesh& m,
    const int64_t max_triangle_number)
    : Invariant(m)
    , m_max_triangle_number(max_triangle_number)
{}

bool MaxTriangleNumberInvariant::before(const simplex::Simplex& t) const
{
    // assumes the mesh is consolidated after swap and collapse
    return mesh().capacity(PrimitiveType::Triangle) < m_max_triangle_number;
}
} // namespace wmtk