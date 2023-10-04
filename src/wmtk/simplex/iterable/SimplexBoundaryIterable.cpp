#include "SimplexBoundaryIterable.hpp"

#include <wmtk/simplex/simplex_boundary.hpp>

namespace wmtk::simplex {


SimplexBoundaryIterable::SimplexBoundaryIterable(const Mesh& mesh, const Simplex& simplex)
    : m_collection(simplex_boundary(mesh, simplex))
{}

} // namespace wmtk::simplex