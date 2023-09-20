#include "OpenStarIterable.hpp"

#include "open_star.hpp"

namespace wmtk::simplex {


OpenStarIterable::OpenStarIterable(const Mesh& mesh, const Simplex& simplex)
    : m_collection(open_star(mesh, simplex))
{}

} // namespace wmtk::simplex