#include "open_star_iterable.hpp"

namespace wmtk::simplex {

OpenStarIterable open_star_iterable(const Mesh& mesh, const Simplex& simplex)
{
    return OpenStarIterable(mesh, simplex);
}

} // namespace wmtk::simplex