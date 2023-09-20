#include "coface_cells.hpp"

#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include "internal/CofaceCells.hpp"

namespace wmtk::simplex {

SimplexCollection coface_cells(const Mesh& mesh, const Simplex& simplex)
{
    switch (mesh.top_simplex_type()) {
    case PrimitiveType::Face:
        return internal::CofaceCells(static_cast<const TriMesh&>(mesh), simplex);
        break;
    case PrimitiveType::Tetrahedron:
        return internal::CofaceCells(static_cast<const TetMesh&>(mesh), simplex);
        break;
    default: assert(false); break;
    }

    return SimplexCollection(mesh);
}

} // namespace wmtk::simplex