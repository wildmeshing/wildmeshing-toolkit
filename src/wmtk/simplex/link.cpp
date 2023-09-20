#include "link.hpp"

#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include "closed_star.hpp"
#include "simplex_boundary.hpp"

namespace wmtk::simplex {

SimplexCollection link(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    SimplexCollection collection(mesh);

    SimplexCollection cs = closed_star(mesh, simplex, sort_and_clean);

    for (const Simplex& s : cs.simplex_vector()) {
        SimplexCollection bd = simplex_boundary(mesh, s, false);
        bd.add(s);
        bd.sort_and_clean();
        if (!bd.contains(simplex)) {
            collection.add(s);
        }
    }

    return collection;
}

} // namespace wmtk::simplex