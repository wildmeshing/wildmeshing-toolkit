#include "boundary_with_preserved_face.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>

namespace wmtk::simplex::internal {

std::vector<Tuple> boundary_with_preserved_face_tuples(
    const Mesh& mesh,
    const Tuple& t,
    PrimitiveType pt,
    PrimitiveType face_pt)
{
    std::vector<Tuple> ret;
    constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Triangle;
    constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;
    if (face_pt < pt) {
        ret.emplace_back(t);
    }
    switch (pt) {
    case PrimitiveType::Vertex: {
        // vertex does not have a boundary
    } break;
    case PrimitiveType::Edge: {
    } break;
    case PrimitiveType::Triangle: {
        if (face_pt < PE) {
            ret.emplace_back(mesh.switch_tuples(t, {PE}));
        }
    } break;
    case PrimitiveType::Tetrahedron: {
        if (face_pt < PF) {
            ret.emplace_back(mesh.switch_tuples(t, {PF}));
            if (face_pt < PE) {
                ret.emplace_back(mesh.switch_tuples(t, {PE, PF}));
            }
        }
    } break;
    default:
        throw std::runtime_error("called boundary_with_preserveD_coface_tuples with halfedge");
        break;
    }
    return ret;
}

std::vector<Tuple>
boundary_with_preserved_face_tuples(const Mesh& mesh, const Simplex& simplex, PrimitiveType face_pt)
{
    return boundary_with_preserved_face_tuples(
        mesh,
        simplex.tuple(),
        simplex.primitive_type(),
        face_pt);
}

std::vector<Simplex> boundary_with_preserved_face_simplices(
    const Mesh& mesh,
    const Tuple& t,
    PrimitiveType pt,
    PrimitiveType face_pt)
{
    if (pt == PrimitiveType::Vertex) {
        return {};
    }
    return utils::tuple_vector_to_homogeneous_simplex_vector(
        mesh,
        boundary_with_preserved_face_tuples(mesh, t, pt, face_pt),
        get_primitive_type_from_id(get_primitive_type_id(pt) - 1));
}

std::vector<Simplex> boundary_with_preserved_face_simplices(
    const Mesh& mesh,
    const Simplex& simplex,
    PrimitiveType face_pt)
{
    return boundary_with_preserved_face_simplices(
        mesh,
        simplex.tuple(),
        simplex.primitive_type(),
        face_pt);
}
} // namespace wmtk::simplex::internal
