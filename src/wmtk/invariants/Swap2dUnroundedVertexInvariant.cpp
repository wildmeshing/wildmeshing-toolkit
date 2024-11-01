#include "Swap2dUnroundedVertexInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/function/utils/amips.hpp>
#include <wmtk/utils/orient.hpp>

namespace wmtk {
Swap2dUnroundedVertexInvariant::Swap2dUnroundedVertexInvariant(
    const Mesh& m,
    const attribute::TypedAttributeHandle<Rational>& coordinate)
    : Invariant(m, true, false, false)
    , m_coordinate_handle(coordinate)
{}

bool Swap2dUnroundedVertexInvariant::before(const simplex::Simplex& t) const
{
    constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Triangle;

    auto accessor = mesh().create_const_accessor(m_coordinate_handle);
    int dim = mesh().get_attribute_dimension(m_coordinate_handle);

    // get the coords of the vertices
    // input face end points
    const std::array<Tuple, 4> vs = {
        {t.tuple(),
         mesh().switch_tuple(t.tuple(), PV),
         mesh().switch_tuples(t.tuple(), {PE, PV}),
         mesh().switch_tuples(t.tuple(), {PF, PE, PV})}};

    for (const auto& v : vs) {
        for (int i = 0; i < dim; ++i) {
            if (!accessor.const_vector_attribute(v)[i].is_rounded()) {
                return false;
            }
        }
    }

    return true;
}

} // namespace wmtk
