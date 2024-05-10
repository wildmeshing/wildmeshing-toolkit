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
} // namespace


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

    if (my_simplex.primitive_type() == cofaces_type) {
        cofaces.add(my_simplex);
        return cofaces;
    }

    std::vector<Tuple> tuples = top_dimension_cofaces_tuples(mesh, my_simplex);

    assert(my_simplex.primitive_type() < cofaces_type);

    for (const Tuple& t : tuples) {
        simplices_preserving_primitive_types(
            cofaces,
            t,
            mesh.top_simplex_type(),
            my_simplex.primitive_type(),
            cofaces_type);
    }

    if (sort_and_clean) {
        cofaces.sort_and_clean();
    }

    return cofaces;
}

} // namespace wmtk::simplex
