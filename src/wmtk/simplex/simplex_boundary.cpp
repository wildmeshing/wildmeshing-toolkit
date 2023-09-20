#include "simplex_boundary.hpp"

#include "internal/SimplexBoundary.hpp"

namespace wmtk::simplex {

SimplexCollection simplex_boundary(const Mesh& mesh, const Simplex& simplex)
{
    return internal::SimplexBoundary(mesh, simplex);
}

} // namespace wmtk::simplex