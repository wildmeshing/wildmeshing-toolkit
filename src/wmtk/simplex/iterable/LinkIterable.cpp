#include "LinkIterable.hpp"

#include <wmtk/simplex/link.hpp>

namespace wmtk::simplex {


LinkIterable::LinkIterable(const Mesh& mesh, const Simplex& simplex)
    : m_collection(link(mesh, simplex))
{}

} // namespace wmtk::simplex