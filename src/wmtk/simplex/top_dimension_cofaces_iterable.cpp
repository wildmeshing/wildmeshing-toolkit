#include "top_dimension_cofaces_iterable.hpp"

namespace wmtk::simplex {

TopDimensionCofacesIterable top_dimension_cofaces_iterable(const Mesh& mesh, const Simplex& simplex)
{
    return TopDimensionCofacesIterable(mesh, simplex);
}

} // namespace wmtk::simplex