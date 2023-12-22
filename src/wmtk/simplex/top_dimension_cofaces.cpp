#include "top_dimension_cofaces.hpp"
#include <wmtk/utils/TupleCellLessThanFunctor.hpp>
#include "utils/tuple_vector_to_homogeneous_simplex_vector.hpp"

#include <queue>
#include <set>

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include "internal/SimplexLessFunctor.hpp"

namespace wmtk::simplex {

namespace {


std::vector<Tuple> top_dimension_cofaces_tuples_vertex(const TriMesh& mesh, const Tuple& t_in)
{
    std::vector<Tuple> collection;

    assert(mesh.is_valid_slow(t_in));
    std::set<Tuple, wmtk::utils::TupleCellLessThan> touched_cells;
    std::queue<Tuple> q;
    q.push(t_in);
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

        if (!mesh.is_boundary_edge(t)) {
            q.push(mesh.switch_face(t));
        }
        const Tuple t_other = mesh.switch_edge(t);
        if (!mesh.is_boundary_edge(t_other)) {
            q.push(mesh.switch_face(t_other));
        }
    }
    return collection;
}
std::vector<Tuple> top_dimension_cofaces_tuples_edge(const TriMesh& mesh, const Tuple& t)
{
    std::vector<Tuple> collection;
    collection.emplace_back(t);
    if (!mesh.is_boundary_edge(t)) {
        collection.emplace_back(mesh.switch_face(t));
    }

    return collection;
}
std::vector<Tuple> top_dimension_cofaces_tuples_face(const TriMesh& mesh, const Tuple& t)
{
    return {t};
}

std::vector<Tuple> top_dimension_cofaces_tuples_vertex(const TetMesh& mesh, const Tuple& input)
{
    std::vector<Tuple> collection;
    std::set<Tuple, wmtk::utils::TupleCellLessThan> touched_cells;
    std::queue<Tuple> q;
    q.push(input);
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

        if (!mesh.is_boundary_face(t1)) {
            q.push(mesh.switch_tetrahedron(t1));
        }
        if (!mesh.is_boundary_face(t2)) {
            q.push(mesh.switch_tetrahedron(t2));
        }
        if (!mesh.is_boundary_face(t3)) {
            q.push(mesh.switch_tetrahedron(t3));
        }
    }
    return collection;
}
std::vector<Tuple> top_dimension_cofaces_tuples_edge(const TetMesh& mesh, const Tuple& input)
{
    std::vector<Tuple> collection;
    std::set<Tuple, wmtk::utils::TupleCellLessThan> touched_cells;
    std::queue<Tuple> q;
    q.push(input);
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

        if (!mesh.is_boundary_face(t1)) {
            q.push(mesh.switch_tetrahedron(t1));
        }
        if (!mesh.is_boundary_face(t2)) {
            q.push(mesh.switch_tetrahedron(t2));
        }
    }
    return collection;
}

std::vector<Tuple> top_dimension_cofaces_tuples_face(const TetMesh& mesh, const Tuple& input)
{
    std::vector<Tuple> collection = {input};
    if (!mesh.is_boundary_face(input)) {
        collection.emplace_back(mesh.switch_tetrahedron(input));
    }
    return collection;
}
std::vector<Tuple> top_dimension_cofaces_tuples_tet(const TetMesh& mesh, const Tuple& t)
{
    return {t};
}


} // namespace

std::vector<Tuple> top_dimension_cofaces_tuples(const EdgeMesh& mesh, const Simplex& simplex)
{
    std::vector<Tuple> collection;


    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex: {
        collection.emplace_back(simplex.tuple());
        if (!mesh.is_boundary_vertex(simplex.tuple())) {
            collection.emplace_back(mesh.switch_edge(simplex.tuple()));
        }
        break;
    }
    case PrimitiveType::Edge: {
        collection.emplace_back(simplex.tuple());
        break;
    }
    case PrimitiveType::Face: {
        throw std::runtime_error(
            "top_dimension_cofaces_tuples not implemented for Face in EdgeMesh");
        break;
    }
    case PrimitiveType::Tetrahedron:
    case PrimitiveType::HalfEdge:
    default: assert(false); break;
    }


    return collection;
}


std::vector<Tuple> top_dimension_cofaces_tuples(const TriMesh& mesh, const Simplex& simplex)
{
    std::vector<Tuple> collection;


    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex: {
        collection = top_dimension_cofaces_tuples_vertex(mesh, simplex.tuple());
        break;
    }
    case PrimitiveType::Edge: {
        collection = top_dimension_cofaces_tuples_edge(mesh, simplex.tuple());
        break;
    }
    case PrimitiveType::Face: {
        collection = top_dimension_cofaces_tuples_face(mesh, simplex.tuple());
        break;
    }
    case PrimitiveType::Tetrahedron:
    case PrimitiveType::HalfEdge:
    default: assert(false); break;
    }


    return collection;
}

std::vector<Tuple> top_dimension_cofaces_tuples(const TetMesh& mesh, const Simplex& simplex)
{
    std::vector<Tuple> collection;


    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex: {
        collection = top_dimension_cofaces_tuples_vertex(mesh, simplex.tuple());
        break;
    }
    case PrimitiveType::Edge: {
        collection = top_dimension_cofaces_tuples_edge(mesh, simplex.tuple());
        break;
    }
    case PrimitiveType::Face: {
        collection = top_dimension_cofaces_tuples_face(mesh, simplex.tuple());
        break;
    }
    case PrimitiveType::Tetrahedron: {
        collection = top_dimension_cofaces_tuples_tet(mesh, simplex.tuple());
        break;
    }
    case PrimitiveType::HalfEdge:
    default: assert(false); break;
    }

    return collection;
}

std::vector<Tuple> top_dimension_cofaces_tuples(const Mesh& mesh, const Simplex& simplex)
{
    switch (mesh.top_simplex_type()) {
    case PrimitiveType::Face:
        return top_dimension_cofaces_tuples(static_cast<const TriMesh&>(mesh), simplex);
    case PrimitiveType::Tetrahedron:
        return top_dimension_cofaces_tuples(static_cast<const TetMesh&>(mesh), simplex);
    case PrimitiveType::Vertex:
    case PrimitiveType::Edge:
        return top_dimension_cofaces_tuples(static_cast<const EdgeMesh&>(mesh), simplex);
    case PrimitiveType::HalfEdge:
    default:
        assert(false);
        throw std::runtime_error("unknown mesh type in top_dimension_cofaces_tuples");
    }
}

SimplexCollection
top_dimension_cofaces(const TriMesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    SimplexCollection collection(
        mesh,
        utils::tuple_vector_to_homogeneous_simplex_vector(
            top_dimension_cofaces_tuples(mesh, simplex),
            PrimitiveType::Face));
    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

SimplexCollection
top_dimension_cofaces(const TetMesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    SimplexCollection collection(
        mesh,
        utils::tuple_vector_to_homogeneous_simplex_vector(
            top_dimension_cofaces_tuples(mesh, simplex),
            PrimitiveType::Tetrahedron));
    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

SimplexCollection
top_dimension_cofaces(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    SimplexCollection collection(
        mesh,
        utils::tuple_vector_to_homogeneous_simplex_vector(
            top_dimension_cofaces_tuples(mesh, simplex),
            mesh.top_simplex_type()));
    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

} // namespace wmtk::simplex
