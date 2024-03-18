#include "RoundRationalToDouble.hpp"
#include <wmtk/Mesh.hpp>

namespace wmtk::operations::utils {

PrimitiveType RoundRationalToDouble::primitive_type() const
{
    return m_hybrid_accessor.primitive_type();
}
std::vector<simplex::Simplex> RoundRationalToDouble::execute(const simplex::Simplex& simplex)
{
    assert(simplex.primitive_type() == primitive_type());
    m_hybrid_accessor.round(simplex.tuple());
    return AttributesUpdate::execute(simplex);
}
} // namespace wmtk::operations::utils
