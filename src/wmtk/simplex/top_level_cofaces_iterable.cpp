#include "top_level_cofaces_iterable.hpp"

namespace wmtk::simplex {

CofaceCellsIterable top_level_cofaces_iterable(const Mesh& mesh, const Simplex& simplex)
{
    return CofaceCellsIterable(mesh, simplex);
}

} // namespace wmtk::simplex