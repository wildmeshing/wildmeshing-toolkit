#include "coface_cells_iterable.hpp"

namespace wmtk::simplex {

CofaceCellsIterable coface_cells_iterable(const Mesh& mesh, const Simplex& simplex)
{
    return CofaceCellsIterable(mesh, simplex);
}

} // namespace wmtk::simplex