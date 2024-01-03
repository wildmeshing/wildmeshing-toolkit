#include "AttributesUpdateBase.hpp"

#include <wmtk/simplex/closed_star.hpp>

namespace wmtk::operations {


AttributesUpdateBase::AttributesUpdateBase(Mesh& m)
    : Operation(m)
{}

std::vector<simplex::Simplex> AttributesUpdateBase::unmodified_primitives(
    const simplex::Simplex& simplex) const
{
    return {simplex};
}


std::vector<simplex::Simplex> AttributesUpdateBase::execute(const simplex::Simplex& simplex)
{
    Accessor<int64_t> accessor = hash_accessor();
    assert(simplex.primitive_type() == primitive_type());
    assert(mesh().is_valid(simplex.tuple(), accessor));

    // const simplex::SimplexCollection star = simplex::closed_star(mesh(), simplex);
    // const auto star_faces = star.simplex_vector();
    // std::vector<Tuple> incident_face_tuple;
    // incident_face_tuple.reserve(star_faces.size());
    // for (const simplex::Simplex& s : star_faces) {
    //     incident_face_tuple.emplace_back(s.tuple());
    // }

    // mesh().update_vertex_operation_hashes(simplex.tuple(), accessor);
    //
    // assert(!mesh().is_valid(simplex.tuple(), accessor));

    auto new_tuple = resurrect_tuple(simplex.tuple());
    assert(mesh().is_valid(new_tuple, accessor));

    return {simplex::Simplex(primitive_type(), new_tuple)};
}

} // namespace wmtk::operations
