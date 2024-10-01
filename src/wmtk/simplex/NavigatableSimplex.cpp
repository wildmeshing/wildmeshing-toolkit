
#include "NavigatableSimplex.hpp"

#include <wmtk/Mesh.hpp>
namespace wmtk::simplex {

NavigatableSimplex::NavigatableSimplex(const Mesh& m, const Simplex& s)
    : NavigatableSimplex(
          s.primitive_type(),
          s.tuple(),
          IdSimplex::id(m, s.primitive_type(), s.tuple()))
{}
NavigatableSimplex::NavigatableSimplex(const Mesh& m, const PrimitiveType& ptype, const Tuple& t)
    : NavigatableSimplex(ptype, t, IdSimplex::id(m, ptype, t))
{}

NavigatableSimplex::operator Simplex() const
{
    return {primitive_type(), tuple()};
}
} // namespace wmtk::simplex
