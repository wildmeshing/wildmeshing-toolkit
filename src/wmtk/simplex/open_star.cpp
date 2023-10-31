#include "open_star.hpp"

#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include "top_dimension_cofaces.hpp"
#include "faces.hpp"

namespace wmtk::simplex {

SimplexCollection open_star(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    SimplexCollection collection(mesh);

    collection.add(simplex);

    const SimplexCollection top_dimension_cofaces_collection =
            top_dimension_cofaces(mesh, simplex, false);

    for (const Simplex& coface_cell : top_dimension_cofaces_collection.simplex_vector()) {
        collection.add(coface_cell);

        const SimplexCollection cell_boundary = faces(mesh, coface_cell);
        for (const Simplex& boundary_simplex : cell_boundary.simplex_vector()) {
            const SimplexCollection bdbd = faces(mesh, boundary_simplex);
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
