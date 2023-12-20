#include "VertexAttributesUpdateBase.hpp"

#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/utils/UpdateVertexMultiMeshMapHash.hpp>
namespace wmtk::operations {
void OperationSettings<tri_mesh::VertexAttributesUpdateBase>::create_invariants()
{
    invariants = std::make_shared<InvariantCollection>(m_mesh);
}

namespace tri_mesh {
VertexAttributesUpdateBase::VertexAttributesUpdateBase(
    Mesh& m,
    const Simplex& t,
    const OperationSettings<VertexAttributesUpdateBase>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.invariants, t)
    , m_settings{settings}
{
    assert(t.primitive_type() == PrimitiveType::Vertex);
    assert(m.is_valid_slow(t.tuple()));
    assert(m.is_valid_slow(input_tuple()));
}

std::string VertexAttributesUpdateBase::name() const
{
    return "tri_mesh_vertex_attributes_update";
}

const Tuple& VertexAttributesUpdateBase::return_tuple() const
{
    return m_output_tuple;
}

std::vector<Simplex> VertexAttributesUpdateBase::modified_primitives() const
{
    return {Simplex(PrimitiveType::Vertex, m_output_tuple)};
}

std::vector<Simplex> VertexAttributesUpdateBase::unmodified_primitives() const
{
    return {input_simplex()};
}


bool VertexAttributesUpdateBase::execute()
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
} // namespace tri_mesh

} // namespace wmtk::operations
