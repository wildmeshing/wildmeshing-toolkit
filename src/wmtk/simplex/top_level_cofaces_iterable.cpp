#include "top_level_cofaces_iterable.hpp"

namespace wmtk::simplex {

TopLevelCofacesIterable top_level_cofaces_iterable(const Mesh& mesh, const Simplex& simplex)
{
    return TopLevelCofacesIterable(mesh, simplex);
}

} // namespace wmtk::simplex