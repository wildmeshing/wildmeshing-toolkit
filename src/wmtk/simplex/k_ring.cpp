#include "k_ring.hpp"

#include "link.hpp"

namespace wmtk::simplex {
SimplexCollection k_ring(const Mesh& mesh, const Simplex& simplex, long k)
{
    if (k < 1) return SimplexCollection(mesh);


    SimplexCollection sc = link(mesh, simplex);

    for (long i = 2; i <= k; ++i) {
        const auto simplices = sc.simplex_vector();
        for (const Simplex& s : simplices) {
            SimplexCollection sc_or = link(mesh, s);
            sc.add(sc_or);
        }

        sc.sort_and_clean();
    }

    return sc;
}
} // namespace wmtk::simplex
