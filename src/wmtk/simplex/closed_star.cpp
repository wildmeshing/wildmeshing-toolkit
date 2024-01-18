#include "closed_star.hpp"

#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include "faces.hpp"
#include "top_dimension_cofaces.hpp"

namespace wmtk::simplex {

SimplexCollection closed_star(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    assert(mesh.is_valid_slow(simplex.tuple()));
    SimplexCollection collection(mesh);

    collection.add(simplex);

    const SimplexCollection top_dimension_cofaces_collection =
        top_dimension_cofaces(mesh, simplex, false);

    for (const Simplex& coface_cell : top_dimension_cofaces_collection.simplex_vector()) {
        collection.add(coface_cell);
        const SimplexCollection cell_boundary = faces(mesh, coface_cell, false);
        collection.add(cell_boundary);
    }

    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

} // namespace wmtk::simplex
