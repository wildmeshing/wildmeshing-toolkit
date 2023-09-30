#include "top_level_cofaces.hpp"

#include <queue>
#include <set>

#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include "internal/SimplexLessFunctor.hpp"

namespace wmtk::simplex {
namespace {
std::vector<Simplex> tuple_to_simplices(const std::vector<Tuple>& tups, PrimitiveType primitive)
{
    std::vector<Simplex> r;
    r.reserve(tups.size());
    std::transform(tups.begin(), tups.end(), std::back_inserter(r), [primitive](const Tuple& t) {
        return Simplex(primitive, t);
    });
    return r;
}

std::vector<Tuple> top_level_cofaces_tuples_vertex(const TriMesh& mesh, const Tuple& t)
{
    std::vector<Tuple> collection;

    std::set<Tuple> touched_cells;
    std::queue<Tuple> q;
    q.push(t);
    while (!q.empty()) {
        const Tuple t = q.front();
        q.pop();

        {
            // check if cell already exists
            const auto [it, did_insert] = touched_cells.insert(t);
            if (!did_insert) {
                continue;
            }
        }
        collection.emplace_back(t);

        if (!mesh.is_boundary(t)) {
            q.push(mesh.switch_face(t));
        }
        const Tuple t_other = mesh.switch_edge(t);
        if (!mesh.is_boundary(t_other)) {
            q.push(mesh.switch_face(t_other));
        }
    }
    return collection;
}
std::vector<Tuple> top_level_cofaces_tuples_edge(const TriMesh& mesh, const Tuple& t)
{
    std::vector<Tuple> collection;
    collection.emplace_back(t);
    if (!mesh.is_boundary(t)) {
        collection.emplace_back(mesh.switch_face(t));
    }

    return collection;
}
std::vector<Tuple> top_level_cofaces_tuples_face(const TriMesh& mesh, const Tuple& t)
{
    return {t};
}

std::vector<Tuple> top_level_cofaces_tuples_vertex(const TetMesh& mesh, const Tuple& t)
{
    std::vector<Tuple> collection;
    std::set<Tuple> touched_cells;
    std::queue<Tuple> q;
    q.push(t);
    while (!q.empty()) {
        const Tuple t = q.front();
        q.pop();

        {
            // check if cell already exists
            const auto [it, was_inserted] = touched_cells.insert(t);
            if (!was_inserted) {
                continue;
            }
        }

        collection.emplace_back(t);

        const Tuple& t1 = t;
        const Tuple t2 = mesh.switch_face(t);
        const Tuple t3 = mesh.switch_tuples(t, {PrimitiveType::Edge, PrimitiveType::Face});

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
    return collection;
}
std::vector<Tuple> top_level_cofaces_tuples_edge(const TetMesh& mesh, const Tuple& t)
{
    std::vector<Tuple> collection;
    std::set<Tuple> touched_cells;
    std::queue<Tuple> q;
    q.push(t);
    while (!q.empty()) {
        const Tuple t = q.front();
        q.pop();

        {
            // check if cell already exists
            const auto [it, was_inserted] = touched_cells.insert(t);
            if (!was_inserted) {
                continue;
            }
        }

        collection.emplace_back(t);

        const Tuple& t1 = t;
        const Tuple t2 = mesh.switch_face(t);

        if (!mesh.is_boundary(t1)) {
            q.push(mesh.switch_tetrahedron(t1));
        }
        if (!mesh.is_boundary(t2)) {
            q.push(mesh.switch_tetrahedron(t2));
        }
    }
    return collection;
}

std::vector<Tuple> top_level_cofaces_tuples_face(const TetMesh& mesh, const Tuple& t)
{
    std::vector<Tuple> collection = {t};
    if (!mesh.is_boundary(t)) {
        collection.emplace_back(mesh.switch_tetrahedron(t));
    }
    return collection;
}
std::vector<Tuple> top_level_cofaces_tuples_tet(const TetMesh& mesh, const Tuple& t)
{
    return {t};
}


} // namespace

std::vector<Tuple> top_level_cofaces_tuples(const TriMesh& mesh, const Simplex& simplex)
{
    std::vector<Tuple> collection;


    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex: {
        collection = top_level_cofaces_tuples_vertex(mesh, simplex.tuple());
        break;
    }
    case PrimitiveType::Edge: {
        collection = top_level_cofaces_tuples_edge(mesh, simplex.tuple());
        break;
    }
    case PrimitiveType::Face: {
        collection = top_level_cofaces_tuples_face(mesh, simplex.tuple());
        break;
    }
    default: assert(false); break;
    }


    return collection;
}

std::vector<Tuple> top_level_cofaces_tuples(const TetMesh& mesh, const Simplex& simplex)
{
    std::vector<Tuple> collection;


    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex: {
        collection = top_level_cofaces_tuples_vertex(mesh, simplex.tuple());
        break;
    }
    case PrimitiveType::Edge: {
        collection = top_level_cofaces_tuples_edge(mesh, simplex.tuple());
        break;
    }
    case PrimitiveType::Face: {
        collection = top_level_cofaces_tuples_face(mesh, simplex.tuple());
        break;
    }
    case PrimitiveType::Tetrahedron: {
        collection = top_level_cofaces_tuples_tet(mesh, simplex.tuple());
        break;
    }
    default: assert(false); break;
    }

    return collection;
}

std::vector<Tuple> top_level_cofaces_tuples(const Mesh& mesh, const Simplex& simplex)
{
    switch (mesh.top_simplex_type()) {
    case PrimitiveType::Face:
        return top_level_cofaces_tuples(static_cast<const TriMesh&>(mesh), simplex);
    case PrimitiveType::Tetrahedron:
        return top_level_cofaces_tuples(static_cast<const TetMesh&>(mesh), simplex);
    default: assert(false); throw "unknown mesh type in top_level_cofaces_tuples";
    }
}

SimplexCollection
top_level_cofaces(const TriMesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    SimplexCollection collection(
        mesh,
        tuple_to_simplices(top_level_cofaces_tuples(mesh, simplex), PrimitiveType::Face));
    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

SimplexCollection
top_level_cofaces(const TetMesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    SimplexCollection collection(
        mesh,
        tuple_to_simplices(top_level_cofaces_tuples(mesh, simplex), PrimitiveType::Tetrahedron));
    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

SimplexCollection
top_level_cofaces(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    SimplexCollection collection(
        mesh,
        tuple_to_simplices(top_level_cofaces_tuples(mesh, simplex), mesh.top_simplex_type()));
    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

} // namespace wmtk::simplex
