#include "FacesIterable.hpp"

#include <wmtk/simplex/faces.hpp>

namespace wmtk::simplex {


FacesIterable::FacesIterable(const Mesh& mesh, const Simplex& simplex)
    : m_collection(faces(mesh, simplex))
{}

} // namespace wmtk::simplex
