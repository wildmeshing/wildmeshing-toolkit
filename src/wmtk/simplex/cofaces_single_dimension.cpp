#include "cofaces_single_dimension.hpp"
#include <queue>
#include <set>
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/utils/SimplexComparisons.hpp>
#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleCellLessThanFunctor.hpp>
#include <wmtk/utils/primitive_range.hpp>
#include "internal/boundary_with_preserved_face.hpp"
#include "link.hpp"
#include "top_dimension_cofaces.hpp"
namespace wmtk::simplex {


namespace {

// wrapper for calling the internal function boundary_with_preserved_face_tuples
std::vector<Tuple> boundary_with_preserved_face_tuples(
    const Mesh& mesh,
    const std::vector<Tuple>& tuples,
    const PrimitiveType pt,
    const PrimitiveType coface_pt)
{
    std::vector<Tuple> r;
    for (const Tuple& t : tuples) {
        const auto tups =
            wmtk::simplex::internal::boundary_with_preserved_face_tuples(mesh, t, pt, coface_pt);
        r.insert(r.end(), tups.begin(), tups.end());
    }

    const PrimitiveType boundary_pt = get_primitive_type_from_id(get_primitive_type_id(pt) - 1);
    std::sort(r.begin(), r.end(), [&](const Tuple& a, const Tuple& b) -> bool {
        return utils::SimplexComparisons::less(mesh, boundary_pt, a, b);
    });
    auto last = std::unique(r.begin(), r.end(), [&](const Tuple& a, const Tuple& b) -> bool {
        return utils::SimplexComparisons::equal(mesh, boundary_pt, a, b);
    });
    r.erase(last, r.end());

    return r;
}


std::vector<Tuple> cofaces_single_dimension_vertex(
    const TriMesh& mesh,
    const Simplex& my_simplex,
    PrimitiveType cofaces_type)
{
    if (cofaces_type == PrimitiveType::Vertex) {
        return {my_simplex.tuple()};
    } else if (cofaces_type == PrimitiveType::Triangle) {
        return top_dimension_cofaces_tuples(mesh, my_simplex);
    }

    assert(cofaces_type == PrimitiveType::Edge);

    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;
    constexpr PrimitiveType PF = PrimitiveType::Triangle;

    std::vector<Tuple> tuples;
    tuples.reserve(6);

    assert(mesh.is_valid(my_simplex.tuple()));
    const Tuple t_in = my_simplex.tuple();
    Tuple t = t_in;

    do {
        const Tuple t_collect = mesh.switch_tuple(t, PV);
        tuples.emplace_back(t_collect);

        if (mesh.is_boundary_edge(t)) {
            break;
        }
        t = mesh.switch_tuples(t, {PF, PE});
    } while (t != t_in);


    if (t == t_in && !mesh.is_boundary_edge(t)) {
        return tuples;
    }

    t = mesh.switch_edge(t_in);

    tuples.emplace_back(t);
    if (mesh.is_boundary_edge(t)) {
        return tuples;
    }
    t = mesh.switch_tuples(t, {PF, PE});

    do {
        const Tuple t_collect = mesh.switch_tuple(t, PV);
        tuples.emplace_back(t_collect);

        if (mesh.is_boundary_edge(t)) {
            break;
        }
        t = mesh.switch_tuples(t, {PF, PE});
    } while (true);

    return tuples;
}

std::vector<Tuple> cofaces_single_dimension_edge(
    const TriMesh& mesh,
    const Simplex& my_simplex,
    PrimitiveType cofaces_type)
{
    if (cofaces_type == PrimitiveType::Edge) {
        return {my_simplex.tuple()};
    }

    assert(cofaces_type == PrimitiveType::Triangle);
    return top_dimension_cofaces_tuples(mesh, my_simplex);
}

std::vector<Tuple> cofaces_single_dimension_face(
    const TriMesh& mesh,
    const Simplex& my_simplex,
    PrimitiveType cofaces_type)
{
    assert(cofaces_type == PrimitiveType::Triangle);
    return {my_simplex.tuple()};
}

} // namespace


std::vector<Tuple> cofaces_single_dimension_tuples(
    const Mesh& mesh,
    const Simplex& my_simplex,
    PrimitiveType cofaces_type)
{
    std::vector<Tuple> tuples;
    if (my_simplex.primitive_type() == cofaces_type) {
        tuples = {my_simplex.tuple()};
        return tuples;
    }

    tuples = top_dimension_cofaces_tuples(mesh, my_simplex);


    assert(my_simplex.primitive_type() < cofaces_type);
    auto range = wmtk::utils::primitive_range(mesh.top_simplex_type(), cofaces_type);
    range.pop_back();
    for (const auto& pt : range) {
        tuples = boundary_with_preserved_face_tuples(mesh, tuples, pt, my_simplex.primitive_type());
    }
    return tuples;
}


std::vector<Simplex> cofaces_single_dimension_simplices(
    const Mesh& mesh,
    const Simplex& simplex,
    PrimitiveType cofaces_type)
{
    return utils::tuple_vector_to_homogeneous_simplex_vector(
        mesh,
        cofaces_single_dimension_tuples(mesh, simplex, cofaces_type),
        cofaces_type);
}

SimplexCollection cofaces_single_dimension(
    const Mesh& mesh,
    const Simplex& my_simplex,
    PrimitiveType cofaces_type,
    bool sort_and_clean)
{
    SimplexCollection collection(
        mesh,
        cofaces_single_dimension_simplices(mesh, my_simplex, cofaces_type));
    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

SimplexCollection cofaces_single_dimension(
    const TriMesh& mesh,
    const Simplex& my_simplex,
    PrimitiveType cofaces_type,
    bool sort_and_clean)
{
    SimplexCollection collection(
        mesh,
        cofaces_single_dimension_simplices(mesh, my_simplex, cofaces_type));
    if (sort_and_clean) {
        collection.sort();
    }

    return collection;
}

std::vector<Tuple> cofaces_single_dimension_tuples(
    const TriMesh& mesh,
    const Simplex& my_simplex,
    PrimitiveType cofaces_type)
{
    switch (my_simplex.primitive_type()) {
    case PrimitiveType::Vertex:
        return cofaces_single_dimension_vertex(mesh, my_simplex, cofaces_type);
    case PrimitiveType::Edge: return cofaces_single_dimension_edge(mesh, my_simplex, cofaces_type);
    case PrimitiveType::Triangle:
        return cofaces_single_dimension_face(mesh, my_simplex, cofaces_type);
    default:
        log_and_throw_error("Unknown primitive type in cofaces_single_dimension_tuples");
        break;
    }
}

std::vector<Simplex> cofaces_single_dimension_simplices(
    const TriMesh& mesh,
    const Simplex& simplex,
    PrimitiveType cofaces_type)
{
    return utils::tuple_vector_to_homogeneous_simplex_vector(
        mesh,
        cofaces_single_dimension_tuples(mesh, simplex, cofaces_type),
        cofaces_type);
}

} // namespace wmtk::simplex
