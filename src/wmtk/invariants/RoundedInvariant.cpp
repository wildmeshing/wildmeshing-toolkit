#include "RoundedInvariant.hpp"
#include <wmtk/Mesh.hpp>

#include <wmtk/simplex/k_ring.hpp>

namespace wmtk {

RoundedInvariant::RoundedInvariant(
    const Mesh& m,
    const attribute::TypedAttributeHandle<Rational>& coordinate)
    : Invariant(m)
    , m_coordinate_handle(coordinate)
{}

bool RoundedInvariant::before(const simplex::Simplex& t) const
{
    const std::array<Tuple, 2> vs = {
        {t.tuple(), mesh().switch_tuple(t.tuple(), PrimitiveType::Vertex)}};

    auto accessor = mesh().create_const_accessor(m_coordinate_handle);

    // check endpoints
    const auto& p1 = accessor.const_vector_attribute(vs[0]);
    const auto& p2 = accessor.const_vector_attribute(vs[1]);

    int dim = mesh().get_attribute_dimension(m_coordinate_handle);

    for (int i = 0; i < dim; ++i) {
        if (!p1[i].is_rounded() || !p2[i].is_rounded()) return false;
    }

    // check one ring
    for (int i = 0; i < 2; ++i) {
        const auto one_ring_v = simplex::k_ring(mesh(), simplex::Simplex::vertex(vs[i]), 1)
                                    .simplex_vector(PrimitiveType::Vertex);
        for (const auto& v : one_ring_v) {
            const auto& pv = accessor.const_vector_attribute(v.tuple());
            for (int d = 0; d < dim; ++d) {
                if (!pv[d].is_rounded()) return false;
            }
        }
    }

    return true;
}

} // namespace wmtk