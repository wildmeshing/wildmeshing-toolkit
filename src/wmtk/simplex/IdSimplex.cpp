#include "IdSimplex.hpp"
#include "NavigatableSimplex.hpp"
namespace wmtk::simplex {

IdSimplex::IdSimplex(const NavigatableSimplex& s)
    : IdSimplex(s.primitive_type(), s.index())
{}
} // namespace wmtk::simplex
