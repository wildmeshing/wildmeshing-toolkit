#include "link_iterable.hpp"

namespace wmtk::simplex {

LinkIterable link_iterable(const Mesh& mesh, const Simplex& simplex)
{
    return LinkIterable(mesh, simplex);
}

} // namespace wmtk::simplex