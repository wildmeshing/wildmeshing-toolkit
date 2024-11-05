#include "half_closed_star_iterable.hpp"

namespace wmtk::simplex {

HalfClosedStarIterable half_closed_star_iterable(const Mesh& mesh, const Tuple& tuple)
{
    return HalfClosedStarIterable(mesh, tuple);
}

} // namespace wmtk::simplex