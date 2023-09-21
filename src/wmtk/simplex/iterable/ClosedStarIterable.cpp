#include "ClosedStarIterable.hpp"

#include <wmtk/simplex/closed_star.hpp>

namespace wmtk::simplex {


ClosedStarIterable::ClosedStarIterable(const Mesh& mesh, const Simplex& simplex)
    : m_collection(closed_star(mesh, simplex))
{}

} // namespace wmtk::simplex