#include "AttributesUpdateBase.hpp"

#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/utils/UpdateVertexMultiMeshMapHash.hpp>
namespace wmtk::operations {

void OperationSettings<AttributesUpdateBase>::create_invariants()
{
    invariants = std::make_shared<InvariantCollection>(m_mesh);
}

AttributesUpdateBase::AttributesUpdateBase(
    Mesh& m,
    const Simplex& t,
    const OperationSettings<AttributesUpdateBase>& settings)
    : TupleOperation(settings.invariants, t)
    , m_settings{settings}
    , m_mesh(m)
    , m_hash_accessor(get_hash_accessor(m))
{
    assert(t.primitive_type() == primitive_type());
    assert(m.is_valid_slow(t.tuple()));
    assert(m.is_valid_slow(input_tuple()));
}

Accessor<long>& AttributesUpdateBase::hash_accessor()
{
    return m_hash_accessor;
}

std::string AttributesUpdateBase::name() const
{
    return "attributes_update";
}

const Tuple& AttributesUpdateBase::return_tuple() const
{
    return m_output_tuple;
}

std::vector<Simplex> AttributesUpdateBase::modified_primitives() const
{
    return {Simplex(primitive_type(), m_output_tuple)};
}

std::vector<Simplex> AttributesUpdateBase::unmodified_primitives() const
{
    return {input_simplex()};
}


bool AttributesUpdateBase::execute()
{
    const SimplicialComplex star = SimplicialComplex::closed_star(mesh(), input_simplex());
    const auto star_faces = star.get_faces();
    std::vector<Tuple> incident_face_tuple;
    incident_face_tuple.reserve(star_faces.size());
    for (const Simplex& s : star_faces) {
        incident_face_tuple.emplace_back(s.tuple());
    }

    mesh().update_vertex_operation_hashes(input_tuple(), hash_accessor());

    assert(!mesh().is_valid(input_tuple(), hash_accessor()));

    m_output_tuple = resurrect_tuple(input_tuple());
    assert(mesh().is_valid_slow(m_output_tuple));


    return true;
}

} // namespace wmtk::operations
