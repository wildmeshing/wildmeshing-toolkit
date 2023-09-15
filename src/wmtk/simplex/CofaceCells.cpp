#include "CofaceCells.hpp"

#include <queue>
#include <set>

#include "../TetMesh.hpp"
#include "../TriMesh.hpp"

namespace wmtk::simplex {

CofaceCells::CofaceCells(const TriMesh& mesh, const Simplex& simplex, const bool sort)
    : SimplexCollection(mesh)
{
    const TriMesh& m = static_cast<const TriMesh&>(m_mesh);

    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex: {
        std::set<Simplex, SimplexLessFunctor> touched_cells(m_simplex_is_less);
        std::queue<Tuple> q;
        q.push(simplex.tuple());
        while (!q.empty()) {
            const Tuple t = q.front();
            q.pop();

            {
                // check if cell already exists
                const auto [it, success] = touched_cells.insert(Simplex::face(t));
                if (!success) {
                    continue;
                }
            }

            add(Simplex::face(t));

            if (!m.is_boundary(t)) {
                q.push(m.switch_face(t));
            }
            const Tuple t_other = m.switch_edge(t);
            if (!m.is_boundary(t_other)) {
                q.push(m.switch_face(t_other));
            }
        }
        break;
    }
    case PrimitiveType::Edge: {
        const Tuple& t = simplex.tuple();
        add(Simplex::face(t));
        if (!m.is_boundary(t)) {
            add(Simplex::face(m.switch_face(t)));
        }
        break;
    }
    case PrimitiveType::Face: {
        add(simplex);
        break;
    }
    default: assert(false); break;
    }

    if (sort) {
        sort_and_clean();
    }
}

CofaceCells::CofaceCells(const TetMesh& mesh, const Simplex& simplex, const bool sort)
    : SimplexCollection(mesh)
{
    throw "this code was not tested yet";

    const TetMesh& m = static_cast<const TetMesh&>(m_mesh);

    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex: {
        std::set<Simplex, SimplexLessFunctor> touched_cells(m_simplex_is_less);
        std::queue<Tuple> q;
        q.push(simplex.tuple());
        while (!q.empty()) {
            const Tuple t = q.front();
            q.pop();

            {
                // check if cell already exists
                const auto [it, success] = touched_cells.insert(Simplex::tetrahedron(t));
                if (!success) {
                    continue;
                }
            }

            add(Simplex::tetrahedron(t));

            const Tuple& t1 = t;
            const Tuple t2 = m.switch_face(t);
            const Tuple t3 = m.switch_face(m.switch_edge(t));

            if (!m.is_boundary(t1)) {
                q.push(m.switch_tetrahedron(t1));
            }
            if (!m.is_boundary(t2)) {
                q.push(m.switch_tetrahedron(t2));
            }
            if (!m.is_boundary(t3)) {
                q.push(m.switch_tetrahedron(t3));
            }
        }
        break;
    }
    case PrimitiveType::Edge: {
        std::set<Simplex, SimplexLessFunctor> touched_cells(m_simplex_is_less);
        std::queue<Tuple> q;
        q.push(simplex.tuple());
        while (!q.empty()) {
            const Tuple t = q.front();
            q.pop();

            {
                // check if cell already exists
                const auto [it, success] = touched_cells.insert(Simplex::tetrahedron(t));
                if (!success) {
                    continue;
                }
            }

            add(Simplex::tetrahedron(t));

            const Tuple& t1 = t;
            const Tuple t2 = m.switch_face(t);

            if (!m.is_boundary(t1)) {
                q.push(m.switch_tetrahedron(t1));
            }
            if (!m.is_boundary(t2)) {
                q.push(m.switch_tetrahedron(t2));
            }
        }
        break;
    }
    case PrimitiveType::Face: {
        const Tuple& t = simplex.tuple();
        add(Simplex::tetrahedron(t));
        if (!m.is_boundary(t)) {
            add(Simplex::tetrahedron(m.switch_tetrahedron(t)));
        }
        break;
    }
    case PrimitiveType::Tetrahedron: {
        add(simplex);
        break;
    }
    default: assert(false); break;
    }

    if (sort) {
        sort_and_clean();
    }
}

} // namespace wmtk::simplex