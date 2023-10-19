#include "faces_iterable.hpp"

namespace wmtk::simplex {

FacesIterable faces_iterable(const Mesh& mesh, const Simplex& simplex)
{
    return FacesIterable(mesh, simplex);
}

} // namespace wmtk::simplex
