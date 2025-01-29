#include "top_dimension_cofaces.hpp"
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleCellLessThanFunctor.hpp>
#include "utils/tuple_vector_to_homogeneous_simplex_vector.hpp"

#include <queue>
#include <set>

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::simplex {

namespace {


void top_dimension_cofaces_tuples_vertex(
    const TriMesh& mesh,
    const Tuple& t_in,
    std::vector<Tuple>& collection)
{
    assert(mesh.is_valid(t_in));
    Tuple t = t_in;
    do {
        collection.emplace_back(t);

        if (mesh.is_boundary_edge(t)) {
            break;
        }
        t = mesh.switch_tuples(t, {PrimitiveType::Triangle, PrimitiveType::Edge});
    } while (t != t_in);

    if (t == t_in && !mesh.is_boundary_edge(t)) {
        return;
    }

    t = mesh.switch_edge(t_in);

    if (mesh.is_boundary_edge(t)) {
        return;
    }
    t = mesh.switch_tuples(t, {PrimitiveType::Triangle, PrimitiveType::Edge});

    do {
        collection.emplace_back(t);

        if (mesh.is_boundary_edge(t)) {
            break;
        }
        t = mesh.switch_tuples(t, {PrimitiveType::Triangle, PrimitiveType::Edge});
    } while (true);
}
void top_dimension_cofaces_tuples_edge(
    const TriMesh& mesh,
    const Tuple& t,
    std::vector<Tuple>& collection)
{
    collection.emplace_back(t);
    if (!mesh.is_boundary_edge(t)) {
        collection.emplace_back(mesh.switch_face(t));
    }
}
void top_dimension_cofaces_tuples_face(
    const TriMesh& mesh,
    const Tuple& t,
    std::vector<Tuple>& collection)
{
    collection.emplace_back(t);
}

void top_dimension_cofaces_tuples_vertex(
    const TetMesh& mesh,
    const Tuple& input,
    std::vector<Tuple>& collection)
{
    std::vector<int64_t> visited;
    visited.reserve(50);

    auto is_visited = [&visited](const Tuple& t) -> bool {
        const int64_t c = t.global_cid();
        for (const int64_t v : visited) {
            if (v == c) {
                return true;
            }
        }
        visited.emplace_back(c);
        return false;
    };

    std::vector<Tuple> q(200);
    size_t q_front = 0;
    size_t q_back = 1;
    q[0] = input;

    while (q_front != q_back) {
        const Tuple t = q[q_front++];


        if (is_visited(t)) {
            continue;
        }

        collection.emplace_back(t);

        if (q_back + 4 >= q.size()) {
            q.resize(q.size() * 1.5);
            // logger().info(
            //     "Increasing size of queue in top_dimension_cofaces_tuples_vertex to {}.",
            //     q.size());
            // logger().info("Current collection size: {}", collection.size());
        }

        const Tuple& t1 = t;
        const Tuple t2 = mesh.switch_face(t);
        const Tuple t3 = mesh.switch_tuples(t, {PrimitiveType::Edge, PrimitiveType::Triangle});

        if (!mesh.is_boundary_face(t1)) {
            const Tuple ts = mesh.switch_tetrahedron(t1);
            q[q_back++] = ts;
        }
        if (!mesh.is_boundary_face(t2)) {
            const Tuple ts = mesh.switch_tetrahedron(t2);
            q[q_back++] = ts;
        }
        if (!mesh.is_boundary_face(t3)) {
            const Tuple ts = mesh.switch_tetrahedron(t3);
            q[q_back++] = ts;
        }
    }
}
void top_dimension_cofaces_tuples_edge(
    const TetMesh& mesh,
    const Tuple& input,
    std::vector<Tuple>& collection)
{
    // std::set<Tuple, wmtk::utils::TupleCellLessThan> touched_cells;
    // std::queue<Tuple> q;
    // q.push(input);
    // while (!q.empty()) {
    //     const Tuple t = q.front();
    //     q.pop();
    //
    //     {
    //        // check if cell already exists
    //        const auto& [it, was_inserted] = touched_cells.insert(t);
    //        if (!was_inserted) {
    //            continue;
    //        }
    //    }
    //
    //    collection.emplace_back(t);
    //
    //    const Tuple& t1 = t;
    //    const Tuple t2 = mesh.switch_face(t);
    //
    //    if (!mesh.is_boundary_face(t1)) {
    //        q.push(mesh.switch_tuples(t1, {PrimitiveType::Tetrahedron, PrimitiveType::Triangle}));
    //    }
    //    if (!mesh.is_boundary_face(t2)) {
    //        q.push(mesh.switch_tetrahedron(t2));
    //    }
    //}


    assert(mesh.is_valid(input));
    Tuple t = input;
    do {
        collection.emplace_back(t);

        if (mesh.is_boundary_face(t)) {
            break;
        }
        t = mesh.switch_tuples(t, {PrimitiveType::Tetrahedron, PrimitiveType::Triangle});
    } while (t != input);

    if (t == input && !mesh.is_boundary_face(t)) {
        return;
    }

    t = input;

    if (mesh.is_boundary_face(mesh.switch_face(t))) {
        return;
    }

    t = mesh.switch_tuples(t, {PrimitiveType::Triangle, PrimitiveType::Tetrahedron});

    do {
        collection.emplace_back(t);

        if (mesh.is_boundary_face(mesh.switch_face(t))) {
            break;
        }
        t = mesh.switch_tuples(t, {PrimitiveType::Triangle, PrimitiveType::Tetrahedron});
    } while (true);

    // t = mesh.switch_face(input);

    // if (mesh.is_boundary_face(t)) {
    //     return;
    // }
    // t = mesh.switch_tuples(t, {PrimitiveType::Tetrahedron, PrimitiveType::Triangle});

    // do {
    //     collection.emplace_back(t);

    //     if (mesh.is_boundary_face(t)) {
    //         break;
    //     }
    //     t = mesh.switch_tuples(t, {PrimitiveType::Tetrahedron, PrimitiveType::Triangle});
    // } while (true);
}

void top_dimension_cofaces_tuples_face(
    const TetMesh& mesh,
    const Tuple& input,
    std::vector<Tuple>& collection)
{
    collection.emplace_back(input);
    if (!mesh.is_boundary_face(input)) {
        collection.emplace_back(mesh.switch_tetrahedron(input));
    }
}
void top_dimension_cofaces_tuples_tet(
    const TetMesh& mesh,
    const Tuple& t,
    std::vector<Tuple>& collection)
{
    collection.emplace_back(t);
}

std::vector<Tuple> top_dimension_cofaces_tuples_vertex(const TriMesh& mesh, const Tuple& t)
{
    std::vector<Tuple> collection;
    top_dimension_cofaces_tuples_vertex(mesh, t, collection);
    return collection;
}
std::vector<Tuple> top_dimension_cofaces_tuples_edge(const TriMesh& mesh, const Tuple& t)
{
    std::vector<Tuple> collection;
    top_dimension_cofaces_tuples_edge(mesh, t, collection);
    return collection;
}
std::vector<Tuple> top_dimension_cofaces_tuples_face(const TriMesh& mesh, const Tuple& t)
{
    std::vector<Tuple> collection;
    top_dimension_cofaces_tuples_face(mesh, t, collection);
    return collection;
}

std::vector<Tuple> top_dimension_cofaces_tuples_vertex(const TetMesh& mesh, const Tuple& input)
{
    std::vector<Tuple> collection;
    top_dimension_cofaces_tuples_vertex(mesh, input, collection);
    return collection;
}
std::vector<Tuple> top_dimension_cofaces_tuples_edge(const TetMesh& mesh, const Tuple& input)
{
    std::vector<Tuple> collection;
    top_dimension_cofaces_tuples_edge(mesh, input, collection);
    return collection;
}
std::vector<Tuple> top_dimension_cofaces_tuples_face(const TetMesh& mesh, const Tuple& input)
{
    std::vector<Tuple> collection;
    top_dimension_cofaces_tuples_face(mesh, input, collection);
    return collection;
}
std::vector<Tuple> top_dimension_cofaces_tuples_tet(const TetMesh& mesh, const Tuple& t)
{
    std::vector<Tuple> collection;
    top_dimension_cofaces_tuples_tet(mesh, t, collection);
    return collection;
}


} // namespace

namespace {


void top_dimension_cofaces_tuples_vertex(
    const TriMesh& mesh,
    const Tuple& t_in,
    SimplexCollection& collection)
{
    assert(mesh.is_valid(t_in));
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
        collection.add(PrimitiveType::Triangle, t);

        if (!mesh.is_boundary_edge(t)) {
            q.push(mesh.switch_tuples(t, {PrimitiveType::Triangle, PrimitiveType::Edge}));
        }
        const Tuple t_other = mesh.switch_edge(t);
        if (!mesh.is_boundary_edge(t_other)) {
            q.push(mesh.switch_face(t_other));
        }
    }
}
void top_dimension_cofaces_tuples_edge(
    const TriMesh& mesh,
    const Tuple& t,
    SimplexCollection& collection)
{
    collection.add(PrimitiveType::Triangle, t);
    if (!mesh.is_boundary_edge(t)) {
        collection.add(PrimitiveType::Triangle, mesh.switch_face(t));
    }
}
void top_dimension_cofaces_tuples_face(
    const TriMesh& mesh,
    const Tuple& t,
    SimplexCollection& collection)
{
    collection.add(PrimitiveType::Triangle, t);
}

void top_dimension_cofaces_tuples_vertex(
    const TetMesh& mesh,
    const Tuple& input,
    SimplexCollection& collection)
{
    std::set<Tuple, wmtk::utils::TupleCellLessThan> touched_cells;
    std::queue<Tuple> q;
    q.push(input);
    while (!q.empty()) {
        const Tuple t = q.front();
        q.pop();

        {
            // check if cell already exists
            const auto& [it, was_inserted] = touched_cells.insert(t);
            if (!was_inserted) {
                continue;
            }
        }

        collection.add(PrimitiveType::Tetrahedron, t);

        const Tuple& t1 = t;
        const Tuple t2 = mesh.switch_face(t);
        const Tuple t3 = mesh.switch_tuples(t, {PrimitiveType::Edge, PrimitiveType::Triangle});

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
}
void top_dimension_cofaces_tuples_edge(
    const TetMesh& mesh,
    const Tuple& input,
    SimplexCollection& collection)
{
    std::set<Tuple, wmtk::utils::TupleCellLessThan> touched_cells;
    std::queue<Tuple> q;
    q.push(input);
    while (!q.empty()) {
        const Tuple t = q.front();
        q.pop();

        {
            // check if cell already exists
            const auto& [it, was_inserted] = touched_cells.insert(t);
            if (!was_inserted) {
                continue;
            }
        }

        collection.add(PrimitiveType::Tetrahedron, t);

        const Tuple& t1 = t;
        const Tuple t2 = mesh.switch_face(t);

        if (!mesh.is_boundary_face(t1)) {
            q.push(mesh.switch_tuples(t1, {PrimitiveType::Tetrahedron, PrimitiveType::Triangle}));
        }
        if (!mesh.is_boundary_face(t2)) {
            q.push(mesh.switch_tetrahedron(t2));
        }
    }
}

void top_dimension_cofaces_tuples_face(
    const TetMesh& mesh,
    const Tuple& input,
    SimplexCollection& collection)
{
    collection.add(PrimitiveType::Tetrahedron, input);
    if (!mesh.is_boundary_face(input)) {
        collection.add(PrimitiveType::Tetrahedron, mesh.switch_tetrahedron(input));
    }
}
void top_dimension_cofaces_tuples_tet(
    const TetMesh& mesh,
    const Tuple& t,
    SimplexCollection& collection)
{
    collection.add(PrimitiveType::Tetrahedron, t);
}


} // namespace

void top_dimension_cofaces(
    const Simplex& simplex,
    SimplexCollection& simplex_collection,
    const bool sort_and_clean)
{
    const auto& m = simplex_collection.mesh();
    top_dimension_cofaces_tuples(m, simplex, simplex_collection);
    if (sort_and_clean) {
        simplex_collection.sort_and_clean();
    }
}

void top_dimension_cofaces_tuples(
    const PointMesh& mesh,
    const Simplex& simplex,
    SimplexCollection& collection)
{
    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex: {
        collection.add(simplex);
        break;
    }
    case PrimitiveType::Edge: {
        log_and_throw_error("top_dimension_cofaces_tuples not implemented for Edge in PointMesh");
        break;
    }
    case PrimitiveType::Triangle: {
        log_and_throw_error("top_dimension_cofaces_tuples not implemented for Face in PointMesh");
    }
    case PrimitiveType::Tetrahedron:
        log_and_throw_error(
            "top_dimension_cofaces_tuples not implemented for Tetrahedron in PointMesh");
    default: assert(false); break;
    }
}

void top_dimension_cofaces_tuples(
    const EdgeMesh& mesh,
    const Simplex& simplex,
    SimplexCollection& collection)
{
    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex: {
        collection.add(simplex);
        if (!mesh.is_boundary_vertex(simplex.tuple())) {
            collection.add(simplex.primitive_type(), mesh.switch_edge(simplex.tuple()));
        }
        break;
    }
    case PrimitiveType::Edge: {
        collection.add(simplex);
        break;
    }
    case PrimitiveType::Triangle: {
        log_and_throw_error("top_dimension_cofaces_tuples not implemented for Face in EdgeMesh");
    }
    case PrimitiveType::Tetrahedron:
    default: assert(false); break;
    }
}

void top_dimension_cofaces_tuples(
    const TriMesh& mesh,
    const Simplex& simplex,
    SimplexCollection& collection)
{
    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex: {
        top_dimension_cofaces_tuples_vertex(mesh, simplex.tuple(), collection);
        break;
    }
    case PrimitiveType::Edge: {
        top_dimension_cofaces_tuples_edge(mesh, simplex.tuple(), collection);
        break;
    }
    case PrimitiveType::Triangle: {
        top_dimension_cofaces_tuples_face(mesh, simplex.tuple(), collection);
        break;
    }
    case PrimitiveType::Tetrahedron:
    default: assert(false); break;
    }
}

void top_dimension_cofaces_tuples(
    const TetMesh& mesh,
    const Simplex& simplex,
    SimplexCollection& collection)
{
    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex: {
        top_dimension_cofaces_tuples_vertex(mesh, simplex.tuple(), collection);
        break;
    }
    case PrimitiveType::Edge: {
        top_dimension_cofaces_tuples_edge(mesh, simplex.tuple(), collection);
        break;
    }
    case PrimitiveType::Triangle: {
        top_dimension_cofaces_tuples_face(mesh, simplex.tuple(), collection);
        break;
    }
    case PrimitiveType::Tetrahedron: {
        top_dimension_cofaces_tuples_tet(mesh, simplex.tuple(), collection);
        break;
    }
    default: assert(false); break;
    }
}

void top_dimension_cofaces_tuples(
    const Mesh& mesh,
    const Simplex& simplex,
    SimplexCollection& collection)
{
    switch (mesh.top_simplex_type()) {
    case PrimitiveType::Triangle:
        top_dimension_cofaces_tuples(static_cast<const TriMesh&>(mesh), simplex, collection);
        break;
    case PrimitiveType::Tetrahedron:
        top_dimension_cofaces_tuples(static_cast<const TetMesh&>(mesh), simplex, collection);
        break;
    case PrimitiveType::Edge:
        top_dimension_cofaces_tuples(static_cast<const EdgeMesh&>(mesh), simplex, collection);
        break;
    case PrimitiveType::Vertex:
        top_dimension_cofaces_tuples(static_cast<const PointMesh&>(mesh), simplex, collection);
        break;
    default: log_and_throw_error("unknown mesh type in top_dimension_cofaces_tuples");
    }
}


void top_dimension_cofaces_tuples(
    const PointMesh& mesh,
    const Simplex& simplex,
    std::vector<Tuple>& collection)
{
    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex: {
        collection.emplace_back(simplex.tuple());
        break;
    }
    case PrimitiveType::Edge: {
        log_and_throw_error("top_dimension_cofaces_tuples not implemented for Edge in PointMesh");
        break;
    }
    case PrimitiveType::Triangle: {
        log_and_throw_error("top_dimension_cofaces_tuples not implemented for Face in PointMesh");
    }
    case PrimitiveType::Tetrahedron:
        log_and_throw_error(
            "top_dimension_cofaces_tuples not implemented for Tetrahedron in PointMesh");
    default: assert(false); break;
    }
}
void top_dimension_cofaces_tuples(
    const EdgeMesh& mesh,
    const Simplex& simplex,
    std::vector<Tuple>& collection)
{
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
    case PrimitiveType::Triangle: {
        throw std::runtime_error(
            "top_dimension_cofaces_tuples not implemented for Face in EdgeMesh");
        break;
    }
    case PrimitiveType::Tetrahedron:
    default: assert(false); break;
    }
}

void top_dimension_cofaces_tuples(
    const TriMesh& mesh,
    const Simplex& simplex,
    std::vector<Tuple>& collection)
{
    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex: {
        collection = top_dimension_cofaces_tuples_vertex(mesh, simplex.tuple());
        break;
    }
    case PrimitiveType::Edge: {
        collection = top_dimension_cofaces_tuples_edge(mesh, simplex.tuple());
        break;
    }
    case PrimitiveType::Triangle: {
        collection = top_dimension_cofaces_tuples_face(mesh, simplex.tuple());
        break;
    }
    case PrimitiveType::Tetrahedron:
    default: assert(false); break;
    }
}

void top_dimension_cofaces_tuples(
    const TetMesh& mesh,
    const Simplex& simplex,
    std::vector<Tuple>& collection)
{
    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex: {
        collection = top_dimension_cofaces_tuples_vertex(mesh, simplex.tuple());
        break;
    }
    case PrimitiveType::Edge: {
        collection = top_dimension_cofaces_tuples_edge(mesh, simplex.tuple());
        break;
    }
    case PrimitiveType::Triangle: {
        collection = top_dimension_cofaces_tuples_face(mesh, simplex.tuple());
        break;
    }
    case PrimitiveType::Tetrahedron: {
        collection = top_dimension_cofaces_tuples_tet(mesh, simplex.tuple());
        break;
    }
    default: assert(false); break;
    }
}

void top_dimension_cofaces_tuples(
    const Mesh& mesh,
    const Simplex& simplex,
    std::vector<Tuple>& tuples)
{
    switch (mesh.top_simplex_type()) {
    case PrimitiveType::Triangle:
        top_dimension_cofaces_tuples(static_cast<const TriMesh&>(mesh), simplex, tuples);
        break;
    case PrimitiveType::Tetrahedron:
        top_dimension_cofaces_tuples(static_cast<const TetMesh&>(mesh), simplex, tuples);
        break;
    case PrimitiveType::Edge:
        top_dimension_cofaces_tuples(static_cast<const EdgeMesh&>(mesh), simplex, tuples);
        break;
    case PrimitiveType::Vertex:
        top_dimension_cofaces_tuples(static_cast<const PointMesh&>(mesh), simplex, tuples);
        break;
    default: assert(false); //"unknown mesh type in top_dimension_cofaces_tuples"
    }
}


SimplexCollection
top_dimension_cofaces(const TriMesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    SimplexCollection collection(
        mesh,
        utils::tuple_vector_to_homogeneous_simplex_vector(
            mesh,
            top_dimension_cofaces_tuples(mesh, simplex),
            PrimitiveType::Triangle));
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
            mesh,
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
            mesh,
            top_dimension_cofaces_tuples(mesh, simplex),
            mesh.top_simplex_type()));
    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

std::vector<Tuple> top_dimension_cofaces_tuples(const PointMesh& mesh, const Simplex& simplex)
{
    std::vector<Tuple> collection;
    top_dimension_cofaces_tuples(mesh, simplex, collection);
    return collection;
}


std::vector<Tuple> top_dimension_cofaces_tuples(const EdgeMesh& mesh, const Simplex& simplex)
{
    std::vector<Tuple> collection;
    top_dimension_cofaces_tuples(mesh, simplex, collection);
    return collection;
}

std::vector<Tuple> top_dimension_cofaces_tuples(const TriMesh& mesh, const Simplex& simplex)
{
    std::vector<Tuple> collection;
    top_dimension_cofaces_tuples(mesh, simplex, collection);
    return collection;
}

std::vector<Tuple> top_dimension_cofaces_tuples(const TetMesh& mesh, const Simplex& simplex)
{
    std::vector<Tuple> collection;
    top_dimension_cofaces_tuples(mesh, simplex, collection);
    return collection;
}

std::vector<Tuple> top_dimension_cofaces_tuples(const Mesh& mesh, const Simplex& simplex)
{
    switch (mesh.top_simplex_type()) {
    case PrimitiveType::Triangle:
        return top_dimension_cofaces_tuples(static_cast<const TriMesh&>(mesh), simplex);
    case PrimitiveType::Tetrahedron:
        return top_dimension_cofaces_tuples(static_cast<const TetMesh&>(mesh), simplex);
    case PrimitiveType::Edge:
        return top_dimension_cofaces_tuples(static_cast<const EdgeMesh&>(mesh), simplex);
    case PrimitiveType::Vertex:
        return top_dimension_cofaces_tuples(static_cast<const PointMesh&>(mesh), simplex);
    default:
        assert(false);
        throw std::runtime_error("unknown mesh type in top_dimension_cofaces_tuples");
    }
}

} // namespace wmtk::simplex
