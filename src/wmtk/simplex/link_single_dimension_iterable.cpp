#include "link_single_dimension_iterable.hpp"

namespace wmtk::simplex {

LinkSingleDimensionIterable link_single_dimension_iterable(
    const Mesh& mesh,
    const Simplex& simplex,
    const PrimitiveType link_type)
{
    return LinkSingleDimensionIterable(mesh, simplex, link_type);
}

} // namespace wmtk::simplex