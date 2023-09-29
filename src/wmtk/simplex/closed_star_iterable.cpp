#include "closed_star_iterable.hpp"

namespace wmtk::simplex {

ClosedStarIterable closed_star_iterable(const Mesh& mesh, const Simplex& simplex)
{
    return ClosedStarIterable(mesh, simplex);
}

} // namespace wmtk::simplex