#include "coface_cells.hpp"

#include <queue>
#include <set>

#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include "internal/SimplexLessFunctor.hpp"

namespace wmtk::simplex {

SimplexCollection
coface_cells(const TriMesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    SimplexCollection collection(mesh);

    internal::SimplexLessFunctor sef(mesh);

    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex: {
        std::set<Simplex, internal::SimplexLessFunctor> touched_cells(sef);
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

            collection.add(Simplex::face(t));

            if (!mesh.is_boundary(t)) {
                q.push(mesh.switch_face(t));
            }
            const Tuple t_other = mesh.switch_edge(t);
            if (!mesh.is_boundary(t_other)) {
                q.push(mesh.switch_face(t_other));
            }
        }
        break;
    }
    case PrimitiveType::Edge: {
        const Tuple& t = simplex.tuple();
        collection.add(Simplex::face(t));
        if (!mesh.is_boundary(t)) {
            collection.add(Simplex::face(mesh.switch_face(t)));
        }
        break;
    }
    case PrimitiveType::Face: {
        collection.add(simplex);
        break;
    }
    default: assert(false); break;
    }

    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

SimplexCollection
coface_cells(const TetMesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    SimplexCollection collection(mesh);

    internal::SimplexLessFunctor sef(mesh);

    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex: {
        std::set<Simplex, internal::SimplexLessFunctor> touched_cells(sef);
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

            collection.add(Simplex::tetrahedron(t));

            const Tuple& t1 = t;
            const Tuple t2 = mesh.switch_face(t);
            const Tuple t3 = mesh.switch_face(mesh.switch_edge(t));

            if (!mesh.is_boundary(t1)) {
                q.push(mesh.switch_tetrahedron(t1));
            }
            if (!mesh.is_boundary(t2)) {
                q.push(mesh.switch_tetrahedron(t2));
            }
            if (!mesh.is_boundary(t3)) {
                q.push(mesh.switch_tetrahedron(t3));
            }
        }
        break;
    }
    case PrimitiveType::Edge: {
        std::set<Simplex, internal::SimplexLessFunctor> touched_cells(sef);
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

            collection.add(Simplex::tetrahedron(t));

            const Tuple& t1 = t;
            const Tuple t2 = mesh.switch_face(t);

            if (!mesh.is_boundary(t1)) {
                q.push(mesh.switch_tetrahedron(t1));
            }
            if (!mesh.is_boundary(t2)) {
                q.push(mesh.switch_tetrahedron(t2));
            }
        }
        break;
    }
    case PrimitiveType::Face: {
        const Tuple& t = simplex.tuple();
        collection.add(Simplex::tetrahedron(t));
        if (!mesh.is_boundary(t)) {
            collection.add(Simplex::tetrahedron(mesh.switch_tetrahedron(t)));
        }
        break;
    }
    case PrimitiveType::Tetrahedron: {
        collection.add(simplex);
        break;
    }
    default: assert(false); break;
    }

    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

SimplexCollection coface_cells(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    switch (mesh.top_simplex_type()) {
    case PrimitiveType::Face:
        return coface_cells(static_cast<const TriMesh&>(mesh), simplex, sort_and_clean);
    case PrimitiveType::Tetrahedron:
        return coface_cells(static_cast<const TetMesh&>(mesh), simplex, sort_and_clean);
    default: assert(false); throw "unknown mesh type in coface_cells";
    }
}

} // namespace wmtk::simplex