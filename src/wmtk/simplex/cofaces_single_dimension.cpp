#include "cofaces_single_dimension.hpp"
#include <queue>
#include <set>
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/tuples_preserving_primitive_types.hpp>
#include <wmtk/simplex/utils/SimplexComparisons.hpp>
#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleCellLessThanFunctor.hpp>
#include <wmtk/utils/primitive_range.hpp>
#include "internal/boundary_with_preserved_face.hpp"
#include "link.hpp"
#include "top_dimension_cofaces.hpp"
namespace wmtk::simplex {


//namespace {
//
//// wrapper for calling the internal function boundary_with_preserved_face_tuples
//std::vector<Tuple> boundary_with_preserved_face_tuples(
//    const Mesh& mesh,
//    const std::vector<Tuple>& tuples,
//    const PrimitiveType pt,
//    const PrimitiveType coface_pt)
//{
//    std::vector<Tuple> r;
//    for (const Tuple& t : tuples) {
//        const auto tups =
//            wmtk::simplex::internal::boundary_with_preserved_face_tuples(mesh, t, pt, coface_pt);
//        r.insert(r.end(), tups.begin(), tups.end());
//    }
//
//    const PrimitiveType boundary_pt = get_primitive_type_from_id(get_primitive_type_id(pt) - 1);
//    std::sort(r.begin(), r.end(), [&](const Tuple& a, const Tuple& b) -> bool {
//        return utils::SimplexComparisons::less(mesh, boundary_pt, a, b);
//    });
//    auto last = std::unique(r.begin(), r.end(), [&](const Tuple& a, const Tuple& b) -> bool {
//        return utils::SimplexComparisons::equal(mesh, boundary_pt, a, b);
//    });
//    r.erase(last, r.end());
//
//    return r;
//}
//} // namespace


std::vector<Tuple> cofaces_single_dimension_tuples(
    const Mesh& mesh,
    const Simplex& my_simplex,
    PrimitiveType cofaces_type)
{
    SimplexCollection cof = cofaces_single_dimension(mesh, my_simplex, cofaces_type);

    return cof.simplex_vector_tuples(cofaces_type);
}


std::vector<Simplex> cofaces_single_dimension_simplices(
    const Mesh& mesh,
    const Simplex& simplex,
    PrimitiveType cofaces_type)
{
    SimplexCollection cof = cofaces_single_dimension(mesh, simplex, cofaces_type);

    return cof.simplex_vector();
}

SimplexCollection cofaces_single_dimension(
    const Mesh& mesh,
    const Simplex& my_simplex,
    PrimitiveType cofaces_type,
    bool sort_and_clean)
{
    simplex::SimplexCollection cofaces(mesh);

    cofaces_single_dimension(cofaces, my_simplex, cofaces_type, sort_and_clean);

    return cofaces;
}

void cofaces_single_dimension_general(
    SimplexCollection& collection,
    const Simplex& my_simplex,
    PrimitiveType cofaces_type,
    bool sort_and_clean)
{
    if (my_simplex.primitive_type() == cofaces_type) {
        collection.add(my_simplex);
        return;
    }

    std::vector<Tuple> tuples = top_dimension_cofaces_tuples(collection.mesh(), my_simplex);

    assert(my_simplex.primitive_type() < cofaces_type);

    for (const Tuple& t : tuples) {
        simplices_preserving_primitive_types(
            collection,
            t,
            collection.mesh().top_simplex_type(),
            my_simplex.primitive_type(),
            cofaces_type);
    }

    if (sort_and_clean) {
        collection.sort_and_clean();
    }
}

namespace {
/**
 * @brief Special case of a TriMesh where we want to get the edges adjacent to a vertex.
 */
void cofaces_single_dimension_tri_vertex_edges(
    SimplexCollection& collection,
    const Simplex& my_simplex,
    bool sort_and_clean)
{
    assert(my_simplex.primitive_type() == PrimitiveType::Vertex);

    const TriMesh& mesh = static_cast<const TriMesh&>(collection.mesh());

    const Tuple& t_in = my_simplex.tuple();
    assert(mesh.is_valid_slow(t_in));

    Tuple t = t_in;
    do {
        collection.add(PrimitiveType::Edge, t);

        if (mesh.is_boundary_edge(t)) {
            break;
        }
        t = mesh.switch_tuples(t, {PrimitiveType::Triangle, PrimitiveType::Edge});
    } while (t != t_in);

    if (t == t_in && !mesh.is_boundary_edge(t)) {
        return;
    }

    t = mesh.switch_edge(t_in);
    collection.add(PrimitiveType::Edge, t);

    if (mesh.is_boundary_edge(t)) {
        return;
    }
    t = mesh.switch_tuples(t, {PrimitiveType::Triangle, PrimitiveType::Edge});

    do {
        collection.add(PrimitiveType::Edge, t);

        if (mesh.is_boundary_edge(t)) {
            break;
        }
        t = mesh.switch_tuples(t, {PrimitiveType::Triangle, PrimitiveType::Edge});
    } while (true);

    if (sort_and_clean) {
        collection.sort_and_clean();
    }
}

void cofaces_single_dimension_tet_edge_triangles(
    SimplexCollection& collection,
    const Simplex& my_simplex,
    bool sort_and_clean)
{
    assert(my_simplex.primitive_type() == PrimitiveType::Vertex);

    const TetMesh& mesh = static_cast<const TetMesh&>(collection.mesh());

    const Tuple& t_in = my_simplex.tuple();
    assert(mesh.is_valid_slow(t_in));
    Tuple t = t_in;
    do {
        collection.add(PrimitiveType::Triangle, t);

        if (mesh.is_boundary_face(t)) {
            break;
        }
        t = mesh.switch_tuples(t, {PrimitiveType::Tetrahedron, PrimitiveType::Triangle});
    } while (t != t_in);

    if (t == t_in && !mesh.is_boundary_face(t)) {
        return;
    }

    t = mesh.switch_face(t_in);
    collection.add(PrimitiveType::Triangle, t);

    if (mesh.is_boundary_face(t)) {
        return;
    }
    t = mesh.switch_tuples(t, {PrimitiveType::Tetrahedron, PrimitiveType::Triangle});

    do {
        collection.add(PrimitiveType::Triangle, t);

        if (mesh.is_boundary_face(t)) {
            break;
        }
        t = mesh.switch_tuples(t, {PrimitiveType::Tetrahedron, PrimitiveType::Triangle});
    } while (true);

    if (sort_and_clean) {
        collection.sort_and_clean();
    }
}
} // namespace

void cofaces_single_dimension(
    SimplexCollection& collection,
    const Simplex& my_simplex,
    PrimitiveType cofaces_type,
    bool sort_and_clean)
{
    assert(my_simplex.primitive_type() < cofaces_type);

    const Mesh& m = collection.mesh();
    if (m.top_simplex_type() == PrimitiveType::Triangle &&
        my_simplex.primitive_type() == PrimitiveType::Vertex &&
        cofaces_type == PrimitiveType::Edge) {
        cofaces_single_dimension_tri_vertex_edges(collection, my_simplex, sort_and_clean);
        return;
    }

    if (m.top_simplex_type() == PrimitiveType::Tetrahedron &&
        my_simplex.primitive_type() == PrimitiveType::Edge &&
        cofaces_type == PrimitiveType::Triangle) {
        cofaces_single_dimension_tet_edge_triangles(collection, my_simplex, sort_and_clean);
        return;
    }

    cofaces_single_dimension_general(collection, my_simplex, cofaces_type, sort_and_clean);
}

} // namespace wmtk::simplex
