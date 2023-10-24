#include "boundary.hpp"
#include "SimplexCollection.hpp"

namespace wmtk::simplex {
// returns all simplices that lie on the boundary of the input simplex (i.e all cofaces)
// This does not include itself
SimplexCollection boundary(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    SimplexCollection collection(mesh);

    const Mesh& m = mesh;
    const Tuple& t = simplex.tuple();

    constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Face;
    constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;
    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex: {
        // vertex does not have a boundary
        break;
    }
    case PrimitiveType::Edge: {
        for (const auto& s : {Simplex::vertex(t), Simplex::vertex(m.switch_vertex(t))}) {
            collection.add(s);
        }
        break;
    }
    case PrimitiveType::Face: {
        for (const auto& s :
             {Simplex::edge(t), //
              Simplex::edge(m.switch_tuples(t, {PE})),
              Simplex::edge(m.switch_tuples(t, {PV, PE}))}) {
            collection.add(s);
        }
        break;
    }
    case PrimitiveType::Tetrahedron: {
        for (const auto& s :
             {Simplex::face(t), //
              Simplex::face(m.switch_tuples(t, {PF})), //
              Simplex::face(m.switch_tuples(t, {PE, PF})), //
              Simplex::face(m.switch_tuples(t, {PV, PE, PF}))}) {
            collection.add(s);
        }
        break;
    }
    default: assert(false); break;
    }

    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}
} // namespace wmtk::simplex
