#include "link.hpp"

#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include "closed_star.hpp"
#include "faces.hpp"

namespace wmtk::simplex {

SimplexCollection link(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    SimplexCollection collection(mesh);

    SimplexCollection cs = closed_star(mesh, simplex, sort_and_clean);

    SimplexCollection simplex_w_bd = faces(mesh, simplex, false);
    simplex_w_bd.add(simplex);
    simplex_w_bd.sort_and_clean();

    for (const Simplex& s : cs.simplex_vector()) {
        SimplexCollection bd = faces(mesh, s, false);
        bd.add(s);
        bd.sort_and_clean();
        SimplexCollection intersection = SimplexCollection::get_intersection(simplex_w_bd, bd);
        if (intersection.simplex_vector().empty()) {
            collection.add(s);
        }
    }

    return collection;
}

} // namespace wmtk::simplex
