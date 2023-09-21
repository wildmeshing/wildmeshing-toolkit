#include "CofaceCellsIterable.hpp"

#include <wmtk/simplex/coface_cells.hpp>

namespace wmtk::simplex {


CofaceCellsIterable::CofaceCellsIterable(const Mesh& mesh, const Simplex& simplex)
    : m_collection(coface_cells(mesh, simplex))
{}

} // namespace wmtk::simplex