#include "k_ring.hpp"

#include "link.hpp"

namespace wmtk::simplex {
SimplexCollection k_ring(const Mesh& mesh, const Simplex& simplex, int64_t k)
{
    if (k < 1) return SimplexCollection(mesh);


    SimplexCollection sc = link(mesh, simplex);

    for (int64_t i = 2; i <= k; ++i) {
        const auto& simplices = sc.simplex_vector();
        for (const IdSimplex& s : simplices) {
            SimplexCollection sc_or = link(mesh, mesh.get_simplex(s));
            sc.add(sc_or);
        }

        sc.sort_and_clean();
    }

    return sc;
}
} // namespace wmtk::simplex
