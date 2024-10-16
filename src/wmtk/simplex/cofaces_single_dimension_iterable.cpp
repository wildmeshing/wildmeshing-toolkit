#include "cofaces_single_dimension_iterable.hpp"

namespace wmtk::simplex {

CofacesSingleDimensionIterable cofaces_single_dimension_iterable(
    const Mesh& mesh,
    const Simplex& simplex,
    const PrimitiveType cofaces_type)
{
    return CofacesSingleDimensionIterable(mesh, simplex, cofaces_type);
}

} // namespace wmtk::simplex