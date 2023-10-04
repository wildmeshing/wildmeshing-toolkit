#include "VertexRelocateWithTag.hpp"

#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::operations::tri_mesh {
VertexRelocateWithTag::VertexRelocateWithTag(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<VertexRelocateWithTag>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.invariants, t)
    , m_pos_accessor(m.create_accessor<double>(settings.position))
    , m_vertex_tag_accessor(m.create_accessor<long>(settings.vertex_tag))
    , m_edge_tag_accessor(m.create_accessor<long>(settings.edge_tag))
    , m_settings{settings}
{}

std::string VertexRelocateWithTag::name() const
{
    return "vertex_relocate_with_tag";
}

const Tuple& VertexRelocateWithTag::return_tuple() const
{
    return m_output_tuple;
}

bool VertexRelocateWithTag::before() const
{
    long vt0 = m_vertex_tag_accessor.const_vector_attribute(input_tuple())(0);
    long vt1 = m_vertex_tag_accessor.const_vector_attribute(mesh().switch_vertex(input_tuple()))(0);
    if (vt0 == m_settings.input_tag_value && vt1 == m_settings.input_tag_value) {
        return false;
    }
    if (!mesh().is_valid_slow(input_tuple())) {
        return false;
    }
    if (!m_settings.smooth_boundary && mesh().is_boundary_vertex(input_tuple())) {
        return false;
    }
    return true;
}

bool VertexRelocateWithTag::execute()
{
    long vt0 = m_vertex_tag_accessor.const_vector_attribute(input_tuple())(0);
    long vt1 = m_vertex_tag_accessor.const_vector_attribute(mesh().switch_vertex(input_tuple()))(0);
    int case_num;
    enum { EMBEDDING_CASE, OFFSET_CASE };
    if ((vt0 == m_settings.input_tag_value && vt1 == m_settings.offset_tag_value) &&
        (vt1 == m_settings.input_tag_value && vt0 == m_settings.offset_tag_value)) {
        case_num = OFFSET_CASE;
    } else {
        case_num = EMBEDDING_CASE;
    }

    switch (case_num) {
    case EMBEDDING_CASE: {
        const std::vector<Simplex> one_ring =
            SimplicialComplex::vertex_one_ring(mesh(), input_tuple());
        auto p_mid = m_pos_accessor.vector_attribute(input_tuple());
        p_mid = Eigen::Vector3d::Zero();
        for (const Simplex& s : one_ring) {
            p_mid += m_pos_accessor.vector_attribute(s.tuple());
        }
        p_mid /= one_ring.size();

        const SimplicialComplex star =
            SimplicialComplex::closed_star(mesh(), Simplex::vertex(input_tuple()));
        const auto star_faces = star.get_faces();
        std::vector<Tuple> incident_face_tuple;
        incident_face_tuple.reserve(star_faces.size());
        for (const Simplex& s : star_faces) {
            incident_face_tuple.emplace_back(s.tuple());
        }

        update_cell_hashes(incident_face_tuple);

        assert(!mesh().is_valid_slow(input_tuple()));

        m_output_tuple = resurrect_tuple(input_tuple());
        assert(mesh().is_valid_slow(m_output_tuple));
    } break;
    case OFFSET_CASE: {
    }
    default: break;
    }


    return true;
}


} // namespace wmtk::operations::tri_mesh
