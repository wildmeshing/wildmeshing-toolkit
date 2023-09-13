#include "SimplexBoundary.hpp"

namespace wmtk::simplex {

SimplexBoundary::SimplexBoundary(const Mesh& mesh, const Simplex& simplex)
    : SimplexCollection(mesh)
{
    const Mesh& m = m_mesh;
    const Tuple& t = simplex.tuple();

    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex: {
        // vertex does not have a boundary
        break;
    }
    case PrimitiveType::Edge: {
        add(Simplex::vertex(t));
        add(Simplex::vertex(m.switch_vertex(t)));
        break;
    }
    case PrimitiveType::Face: {
        const Simplex e0 = Simplex::edge(t);
        const Simplex e1 = Simplex::edge(m.switch_edge(t));
        const Simplex e2 = Simplex::edge(m.switch_edge(m.switch_vertex(t)));
        add(e0);
        add(SimplexBoundary(m, e0));
        add(e1);
        add(SimplexBoundary(m, e1));
        add(e2);
        add(SimplexBoundary(m, e2));
        break;
    }
    case PrimitiveType::Tetrahedron: {
        const Simplex f0 = Simplex::face(t);
        const Simplex f1 = Simplex::face(m.switch_face(t));
        const Simplex f2 = Simplex::face(m.switch_face(m.switch_edge(t)));
        const Simplex f3 = Simplex::face(m.switch_face(m.switch_edge(m.switch_vertex(t))));
        add(f0);
        add(SimplexBoundary(m, f0));
        add(f1);
        add(SimplexBoundary(m, f1));
        add(f2);
        add(SimplexBoundary(m, f2));
        add(f3);
        add(SimplexBoundary(m, f3));
        break;
    }
    default: assert(false); break;
    }

    sort_and_clean();
}

} // namespace wmtk::simplex
