#include "simplex_boundary_iterable.hpp"

namespace wmtk::simplex {

SimplexBoundaryIterable simplex_boundary_iterable(const Mesh& mesh, const Simplex& simplex)
{
    return SimplexBoundaryIterable(mesh, simplex);
}

} // namespace wmtk::simplex