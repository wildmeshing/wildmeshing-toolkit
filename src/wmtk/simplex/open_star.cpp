#include "open_star.hpp"

#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include "coface_cells.hpp"
#include "simplex_boundary.hpp"

namespace wmtk::simplex {

SimplexCollection open_star(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    SimplexCollection collection(mesh);

    collection.add(simplex);

    const SimplexCollection coface_cells_collection =
        mesh.top_simplex_type() == PrimitiveType::Face
            ? coface_cells(static_cast<const TriMesh&>(mesh), simplex, false)
            : coface_cells(static_cast<const TetMesh&>(mesh), simplex, false);

    for (const Simplex& coface_cell : coface_cells_collection.simplex_vector()) {
        collection.add(coface_cell);

        const SimplexCollection cell_boundary = simplex_boundary(mesh, coface_cell);
        for (const Simplex& boundary_simplex : cell_boundary.simplex_vector()) {
            const SimplexCollection bdbd = simplex_boundary(mesh, boundary_simplex);
            if (bdbd.contains(simplex)) {
                collection.add(boundary_simplex);
            }
        }
    }

    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

} // namespace wmtk::simplex