#include "boundary.hpp"
#include "SimplexCollection.hpp"
#include "utils/tuple_vector_to_homogeneous_simplex_vector.hpp"

namespace wmtk::simplex {

std::vector<Tuple> boundary_tuples(const Mesh& m, const Tuple& t, PrimitiveType pt)
{
    std::vector<Tuple> ret;
    constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Triangle;
    constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;
    switch (pt) {
    case PrimitiveType::Vertex: {
        // vertex does not have a boundary
    } break;
    case PrimitiveType::Edge: {
        ret = {t, m.switch_tuple(t, PV)};
    } break;
    case PrimitiveType::Triangle: {
        ret = {
            t, //
            m.switch_tuples(t, {PE}),
            m.switch_tuples(t, {PV, PE})};
    } break;
    case PrimitiveType::Tetrahedron: {
        ret = {
            t, //
            m.switch_tuples(t, {PF}), //
            m.switch_tuples(t, {PE, PF}), //
            m.switch_tuples(t, {PV, PE, PF})};
    } break;
    default: assert(false); break;
    }
    return ret;
}

std::vector<Tuple> boundary_tuples(const Mesh& mesh, const Simplex& simplex)
{
    return boundary_tuples(mesh, simplex.tuple(), simplex.primitive_type());
}

SimplexCollection boundary(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    SimplexCollection collection(
        mesh,
        utils::tuple_vector_to_homogeneous_simplex_vector(
            mesh,
            boundary_tuples(mesh, simplex),

            get_primitive_type_from_id(get_primitive_type_id(simplex.primitive_type()) - 1)));

    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}
} // namespace wmtk::simplex
