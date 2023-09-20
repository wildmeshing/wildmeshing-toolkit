#include "SimplexBoundary.hpp"

namespace wmtk::simplex::internal {

SimplexBoundary::SimplexBoundary(const Mesh& mesh, const Simplex& simplex, const bool sort)
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
        add(SimplexBoundary(m, e0, false));
        add(e1);
        add(SimplexBoundary(m, e1, false));
        add(e2);
        add(SimplexBoundary(m, e2, false));
        break;
    }
    case PrimitiveType::Tetrahedron: {
        const Simplex f0 = Simplex::face(t);
        const Simplex f1 = Simplex::face(m.switch_face(t));
        const Simplex f2 = Simplex::face(m.switch_face(m.switch_edge(t)));
        const Simplex f3 = Simplex::face(m.switch_face(m.switch_edge(m.switch_vertex(t))));
        add(f0);
        add(SimplexBoundary(m, f0, false));
        add(f1);
        add(SimplexBoundary(m, f1, false));
        add(f2);
        add(SimplexBoundary(m, f2, false));
        add(f3);
        add(SimplexBoundary(m, f3, false));
        break;
    }
    default: assert(false); break;
    }

    if (sort) {
        sort_and_clean();
    }
}

} // namespace wmtk::simplex::internal
