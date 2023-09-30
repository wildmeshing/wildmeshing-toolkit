#include "CofaceCellsIterable.hpp"

#include <wmtk/simplex/top_level_cofaces.hpp>

namespace wmtk::simplex {


CofaceCellsIterable::CofaceCellsIterable(const Mesh& mesh, const Simplex& simplex)
    : m_collection(top_level_cofaces(mesh, simplex))
{}

} // namespace wmtk::simplex