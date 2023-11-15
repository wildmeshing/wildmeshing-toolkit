#include "VertexAttributesUpdateBase.hpp"

#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::operations {
void OperationSettings<tri_mesh::VertexAttributesUpdateBase>::initialize_invariants(
    const TriMesh& m)
{
    // outdated + is valid tuple
    invariants = basic_invariant_collection(m);
}

namespace tri_mesh {
VertexAttributesUpdateBase::VertexAttributesUpdateBase(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<VertexAttributesUpdateBase>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.invariants, t)
//, m_settings{settings}
{
    assert(m.is_valid_slow(t));
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

std::vector<Tuple> VertexAttributesUpdateBase::modified_primitives(PrimitiveType type) const
{
    if (type == PrimitiveType::Face) {
        assert(mesh().is_valid_slow(m_output_tuple));
        Simplex v(PrimitiveType::Vertex, m_output_tuple);
        auto sc = SimplicialComplex::open_star(mesh(), v);
        auto faces = sc.get_simplices(PrimitiveType::Face);
        std::vector<Tuple> ret;
        for (const auto& face : faces) {
            ret.emplace_back(face.tuple());
        }
        return ret;
    } else {
        return {};
    }
}


bool VertexAttributesUpdateBase::execute()
{
    const SimplicialComplex star =
        SimplicialComplex::closed_star(mesh(), Simplex::vertex(input_tuple()));
    const auto star_faces = star.get_faces();
    std::vector<Tuple> incident_face_tuple;
    incident_face_tuple.reserve(star_faces.size());
    for (const Simplex& s : star_faces) {
        incident_face_tuple.emplace_back(s.tuple());
    }

    update_cell_hashes(incident_face_tuple);
    assert(!mesh().is_valid(input_tuple(), hash_accessor()));

    m_output_tuple = resurrect_tuple(input_tuple());
    assert(mesh().is_valid_slow(m_output_tuple));


    return true;
}
} // namespace tri_mesh

} // namespace wmtk::operations
