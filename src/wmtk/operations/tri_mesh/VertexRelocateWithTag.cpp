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

void VertexRelocateWithTag::update_topology()
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

    assert(!mesh().is_valid_slow(input_tuple()));

    m_output_tuple = resurrect_tuple(input_tuple());
    assert(mesh().is_valid_slow(m_output_tuple));
}

double VertexRelocateWithTag::get_area(const Tuple& t)
{
    auto p0 = m_pos_accessor.vector_attribute(input_tuple());
    auto p1 = m_pos_accessor.vector_attribute(mesh().switch_edge(input_tuple()));
    auto p2 = m_pos_accessor.vector_attribute(mesh().switch_vertex(input_tuple()));
    double a, b, c, d;
    a = (p1 - p0)(0);
    b = (p1 - p0)(1);
    c = (p2 - p0)(0);
    d = (p2 - p0)(1);
    return a * d - b * c;
}

bool VertexRelocateWithTag::is_invert()
{
    const SimplicialComplex star =
        SimplicialComplex::closed_star(mesh(), Simplex::vertex(input_tuple()));
    const auto star_faces = star.get_faces();
    for (const Simplex& face : star_faces) {
        if (get_area(face.tuple()) < 0) {
            return true;
        }
    }
    return false;
}

void VertexRelocateWithTag::modify_pos(const Eigen::Vector3d& origin_pos)
{
    Eigen::Vector3d best_pos = origin_pos;
    auto cur_pos = m_pos_accessor.vector_attribute(input_tuple());
    Eigen::Vector3d last_invert_pos = cur_pos;
    for (long i = 0; i < m_settings.iteration_time_for_optimal_position; ++i) {
        if (is_invert()) {
            last_invert_pos = cur_pos;
            cur_pos += best_pos;
            cur_pos /= 2.0;
        } else {
            best_pos = cur_pos;
            cur_pos += last_invert_pos;
            cur_pos /= 2.0;
        }
        update_topology();
    }
    if (is_invert()) {
        cur_pos = best_pos;
        update_topology();
    }
}

void VertexRelocateWithTag::push_offset() {}

void VertexRelocateWithTag::relocate(const int case_num)
{
    if (case_num == INPUT_CASE) {
        return;
    }

    const std::vector<Simplex> one_ring = SimplicialComplex::vertex_one_ring(mesh(), input_tuple());
    auto p_mid = m_pos_accessor.vector_attribute(input_tuple());
    Eigen::Vector3d origin_pos = p_mid;
    p_mid = Eigen::Vector3d::Zero();
    int offset_num = 0;
    for (const Simplex& s : one_ring) {
        if (case_num == OFFSET_CASE) {
            long tag = m_vertex_tag_accessor.vector_attribute(s.tuple())(0);
            if (tag == m_settings.offset_tag_value) {
                p_mid += m_pos_accessor.vector_attribute(s.tuple());
                offset_num++;
            }
        } else {
            p_mid += m_pos_accessor.vector_attribute(s.tuple());
        }
    }

    if (case_num == OFFSET_CASE) {
        assert(offset_num == 2);
        p_mid /= 2.0;
        update_topology();
        push_offset();
    } else {
        p_mid /= one_ring.size();
    }

    // this block is replicated here since I am not sure if new pos has updated in stars.
    update_topology();

    // modify the position to the optimal position
    if (is_invert()) {
        modify_pos(origin_pos);
    }
}

bool VertexRelocateWithTag::execute()
{
    long vt0 = m_vertex_tag_accessor.const_vector_attribute(input_tuple())(0);
    int case_num;

    if (vt0 == m_settings.offset_tag_value) {
        case_num = OFFSET_CASE;
    } else if (vt0 == m_settings.embedding_tag_value) {
        case_num = EMBEDDING_CASE;
    } else {
        case_num = INPUT_CASE;
    }

    switch (case_num) {
    case EMBEDDING_CASE: {
        relocate(EMBEDDING_CASE);
    } break;
    case OFFSET_CASE: {
        relocate(OFFSET_CASE);
    }; break;
    case INPUT_CASE:
    default: break;
    }

    return true;
}


} // namespace wmtk::operations::tri_mesh
