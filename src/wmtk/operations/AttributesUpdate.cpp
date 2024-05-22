#include "AttributesUpdate.hpp"

#include <wmtk/simplex/closed_star.hpp>

namespace wmtk::operations {


AttributesUpdate::AttributesUpdate(Mesh& m)
    : Operation(m)
{
    operation_name = "AttributesUpdate";
}

std::vector<simplex::Simplex> AttributesUpdate::unmodified_primitives(
    const simplex::Simplex& simplex) const
{
    return {simplex};
}


std::vector<simplex::Simplex> AttributesUpdate::execute(const simplex::Simplex& simplex)
{
    assert(simplex.primitive_type() == primitive_type());
    assert(mesh().is_valid_slow(simplex.tuple()));

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
    assert(mesh().is_valid_slow(new_tuple));

    return {simplex::Simplex(mesh(), primitive_type(), new_tuple)};
}

AttributesUpdateWithFunction::AttributesUpdateWithFunction(Mesh& m)
    : AttributesUpdate(m)
{}

std::vector<simplex::Simplex> AttributesUpdateWithFunction::execute(const simplex::Simplex& simplex)
{
    if (bool(m_function)) {
        if (!m_function(mesh(), simplex)) return {};
    }
    return AttributesUpdate::execute(simplex);
}

} // namespace wmtk::operations
