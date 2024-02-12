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

std::vector<Tuple> boundary_with_preserved_face_tuples(
    const Mesh& mesh,
    const std::vector<Tuple>& tuples,
    PrimitiveType pt,
    PrimitiveType coface_pt)
{
    const PrimitiveType boundary_pt = get_primitive_type_from_id(get_primitive_type_id(pt) - 1);
    std::vector<Tuple> r;
    for (const Tuple& t : tuples) {
        auto tups =
            wmtk::simplex::internal::boundary_with_preserved_face_tuples(mesh, t, pt, coface_pt);
        r.insert(r.end(), tups.begin(), tups.end());
    }
    std::sort(r.begin(), r.end(), [&](const Tuple& a, const Tuple& b) -> bool {
        return utils::SimplexComparisons::less(mesh, boundary_pt, a, b);
    });
    auto last = std::unique(r.begin(), r.end(), [&](const Tuple& a, const Tuple& b) -> bool {
        return utils::SimplexComparisons::equal(mesh, boundary_pt, a, b);
    });
    r.erase(last, r.end());

    return r;
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

} // namespace wmtk::simplex
