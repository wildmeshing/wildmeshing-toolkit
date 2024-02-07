#include "open_star.hpp"

#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include "faces.hpp"
#include "top_dimension_cofaces.hpp"

namespace wmtk::simplex {

SimplexCollection open_star(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    switch (mesh.top_simplex_type()) {
    case PrimitiveType::Face:
        return open_star(static_cast<const TriMesh&>(mesh), simplex, sort_and_clean);
    case PrimitiveType::Tetrahedron:
        return open_star(static_cast<const TetMesh&>(mesh), simplex, sort_and_clean);
    case PrimitiveType::Vertex:
    case PrimitiveType::Edge:
    case PrimitiveType::HalfEdge:
    default: return open_star_slow(mesh, simplex, sort_and_clean); break;
    }
}

SimplexCollection open_star(const TriMesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    // make use of the fact that the top dimension coface tuples always contain the simplex itself
    const std::vector<Tuple> cell_tuples = top_dimension_cofaces_tuples(mesh, simplex);

    SimplexCollection collection(mesh);
    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex:
        collection.reserve(cell_tuples.size() * 3 + 1);
        for (const Tuple& t : cell_tuples) {
            collection.add(Simplex::face(t));
            collection.add(Simplex::edge(t));
            collection.add(Simplex::edge(mesh.switch_edge(t)));
        }
        break;
    case PrimitiveType::Edge:
        collection.reserve(cell_tuples.size() + 1);
        for (const Tuple& t : cell_tuples) {
            collection.add(Simplex::face(t));
        }
        break;
    case PrimitiveType::Face: collection.reserve(1); break;
    case PrimitiveType::Tetrahedron:
    case PrimitiveType::HalfEdge:
    default: break;
    }
    collection.add(simplex);


    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

SimplexCollection open_star(const TetMesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    // make use of the fact that the top dimension coface tuples always contain the simplex itself
    const std::vector<Tuple> cell_tuples = top_dimension_cofaces_tuples(mesh, simplex);

    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;
    constexpr PrimitiveType PF = PrimitiveType::Face;
    constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

    SimplexCollection collection(mesh);
    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex:
        collection.reserve(cell_tuples.size() * 7 + 1);
        for (Tuple t : cell_tuples) {
            collection.add(Simplex::tetrahedron(t));
            collection.add(Simplex::face(t));
            collection.add(Simplex::edge(t));
            t = mesh.switch_tuples(t, {PE, PF});
            collection.add(Simplex::face(t));
            collection.add(Simplex::edge(t));
            t = mesh.switch_tuples(t, {PE, PF});
            collection.add(Simplex::face(t));
            collection.add(Simplex::edge(t));
        }
        break;
    case PrimitiveType::Edge:
        collection.reserve(cell_tuples.size() * 3 + 1);
        for (const Tuple& t : cell_tuples) {
            collection.add(Simplex::tetrahedron(t));
            collection.add(Simplex::face(t));
            collection.add(Simplex::face(mesh.switch_face(t)));
        }
        break;
    case PrimitiveType::Face:
        collection.reserve(3);
        assert(cell_tuples.size() <= 2);
        for (const Tuple& t : cell_tuples) {
            collection.add(Simplex::tetrahedron(t));
        }
        break;
    case PrimitiveType::Tetrahedron: collection.reserve(1); break;
    case PrimitiveType::HalfEdge:
    default: log_and_throw_error("Unknown primitive type in open_star."); break;
    }
    collection.add(simplex);


    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

SimplexCollection
open_star_slow(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean)
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
