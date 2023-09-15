#include "OpenStar.hpp"

#include <queue>
#include <set>

#include "../TetMesh.hpp"
#include "../TriMesh.hpp"
#include "CofaceCells.hpp"
#include "SimplexBoundary.hpp"

namespace wmtk::simplex {


OpenStar::OpenStar(const Mesh& mesh, const Simplex& simplex, const bool sort)
    : SimplexCollection(mesh)
{
    if (!dynamic_cast<const TriMesh*>(&mesh)) {
        throw "only testet for TriMesh";
    }

    add(simplex);

    CofaceCells coface_cells = dynamic_cast<const TriMesh*>(&mesh)
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

} // namespace wmtk::simplex
