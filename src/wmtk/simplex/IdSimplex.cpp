#include "IdSimplex.hpp"
#include <wmtk/Mesh.hpp>
#include "NavigatableSimplex.hpp"
namespace wmtk::simplex {

IdSimplex::IdSimplex(const NavigatableSimplex& s)
    : IdSimplex(s.primitive_type(), s.index())
{}

int64_t IdSimplex::id(const Mesh& m, PrimitiveType pt, const Tuple& t)
{
    return m.id(t, pt);
}
} // namespace wmtk::simplex
