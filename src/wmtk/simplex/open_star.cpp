#include "open_star.hpp"

#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include "faces.hpp"
#include "top_dimension_cofaces.hpp"

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

SimplexCollection open_star(const TriMesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    // make use of the fact that the top dimension coface tuples always contain the simplex itself
    const std::vector<Tuple> cell_tuples = top_dimension_cofaces_tuples(mesh, simplex);

    std::vector<Simplex> all_cofaces;
    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex:
        all_cofaces.reserve(cell_tuples.size() * 3 + 1);
        for (const Tuple& t : cell_tuples) {
            all_cofaces.emplace_back(Simplex::face(t));
            all_cofaces.emplace_back(Simplex::edge(t));
            all_cofaces.emplace_back(Simplex::edge(mesh.switch_edge(t)));
        }
        break;
    case PrimitiveType::Edge:
        all_cofaces.reserve(cell_tuples.size() + 1);
        for (const Tuple& t : cell_tuples) {
            all_cofaces.emplace_back(Simplex::face(t));
        }
        break;
    case PrimitiveType::Face: all_cofaces.reserve(1); break;
    case PrimitiveType::Tetrahedron:
    case PrimitiveType::HalfEdge:
    default: break;
    }
    all_cofaces.emplace_back(simplex);

    SimplexCollection collection(mesh, std::move(all_cofaces));


    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

} // namespace wmtk::simplex
