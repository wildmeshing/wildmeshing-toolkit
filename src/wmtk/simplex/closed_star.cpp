#include "closed_star.hpp"

#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include "coface_cells.hpp"
#include "simplex_boundary.hpp"

namespace wmtk::simplex {

SimplexCollection closed_star(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    if (mesh.top_simplex_type() != PrimitiveType::Face) {
        throw "only testet for TriMesh";
    }

    SimplexCollection collection(mesh);

    collection.add(simplex);

    const SimplexCollection coface_cells_collection =
        mesh.top_simplex_type() == PrimitiveType::Face
            ? coface_cells(static_cast<const TriMesh&>(mesh), simplex, false)
            : coface_cells(static_cast<const TetMesh&>(mesh), simplex, false);

    for (const Simplex& coface_cell : coface_cells_collection.simplex_vector()) {
        collection.add(coface_cell);
        const SimplexCollection cell_boundary = simplex_boundary(mesh, coface_cell);
        collection.add(cell_boundary);
    }

    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

} // namespace wmtk::simplex