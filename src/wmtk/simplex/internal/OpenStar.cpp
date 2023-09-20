#include "OpenStar.hpp"

#include <queue>
#include <set>

#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include "CofaceCells.hpp"
#include "SimplexBoundary.hpp"

namespace wmtk::simplex::internal {


OpenStar::OpenStar(const Mesh& mesh, const Simplex& simplex, const bool sort)
    : SimplexCollection(mesh)
{
    if (mesh.top_simplex_type() != PrimitiveType::Face) {
        throw "only testet for TriMesh";
    }

    add(simplex);

    CofaceCells coface_cells = mesh.top_simplex_type() == PrimitiveType::Face
                                   ? CofaceCells(static_cast<const TriMesh&>(mesh), simplex, false)
                                   : CofaceCells(static_cast<const TetMesh&>(mesh), simplex, false);

    for (const Simplex& coface_cell : coface_cells.simplex_vector()) {
        add(coface_cell);

        SimplexBoundary cell_boundary(mesh, coface_cell);
        for (const Simplex& boundary_simplex : cell_boundary.simplex_vector()) {
            SimplexBoundary bdbd(mesh, boundary_simplex);
            if (bdbd.contains(simplex)) {
                add(boundary_simplex);
            }
        }
    }

    if (sort) {
        sort_and_clean();
    }
}

} // namespace wmtk::simplex::internal
