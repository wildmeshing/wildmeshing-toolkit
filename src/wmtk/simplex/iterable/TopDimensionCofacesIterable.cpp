#include "TopDimensionCofacesIterable.hpp"

#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::simplex {


TopDimensionCofacesIterable::TopDimensionCofacesIterable(const Mesh& mesh, const Simplex& simplex)
    //: m_collection(top_dimension_cofaces(mesh, simplex))
    : m_mesh(&mesh)
    , m_simplex(simplex)
{}

TopDimensionCofacesIterable::Iterator::Iterator(
    const Mesh& mesh,
    const Simplex& simplex,
    bool is_done)
    : m_mesh(&mesh)
    , m_simplex(simplex)
    , t(simplex.tuple())
    , m_phase(IteratorPhase::Forward)
    , m_is_end(is_done)
{
    if (m_is_end) {
        return;
    }

    if (m_mesh->top_simplex_type() == PrimitiveType::Triangle) {
        if (m_simplex.primitive_type() == PrimitiveType::Vertex) {
            init_trimesh_vertex();
        }
        return;
    }

    log_and_throw_error(
        "TopDimensionCofacesIterable not implemented for that simplex and/or mesh type.");
}

TopDimensionCofacesIterable::Iterator TopDimensionCofacesIterable::Iterator::operator++()
{
    if (m_mesh->top_simplex_type() == PrimitiveType::Triangle) {
        switch (m_simplex.primitive_type()) {
        case PrimitiveType::Vertex: return step_trimesh_vertex();
        case PrimitiveType::Edge: return step_trimesh_edge();
        case PrimitiveType::Triangle: return step_trimesh_face();
        default: assert(false); // unknown simplex type
        }
    }

    log_and_throw_error(
        "TopDimensionCofacesIterable not implemented for that simplex and/or mesh type.");
}

bool TopDimensionCofacesIterable::Iterator::operator!=(const Iterator& other) const
{
    const bool diff_end = (m_is_end != other.m_is_end);
    return diff_end;
}

Tuple TopDimensionCofacesIterable::Iterator::operator*()
{
    return t;
}

const Tuple& TopDimensionCofacesIterable::Iterator::operator*() const
{
    return t;
}

void TopDimensionCofacesIterable::Iterator::init_trimesh_vertex()
{
    constexpr PrimitiveType PE = PrimitiveType::Edge;

    // check if forward or backward phase can be executed
    if (m_mesh->is_boundary(PE, t)) {
        m_phase = IteratorPhase::Intermediate;

        // check if a backward phase exists
        const Tuple opp_of_input = m_mesh->switch_tuple(m_simplex.tuple(), PE);
        if (m_mesh->is_boundary(PE, opp_of_input)) {
            m_phase = IteratorPhase::End;
        }
    }
}

TopDimensionCofacesIterable::Iterator TopDimensionCofacesIterable::Iterator::step_trimesh_vertex()
{
    constexpr PrimitiveType PE = PrimitiveType::Edge;
    constexpr PrimitiveType PF = PrimitiveType::Triangle;

    if (m_phase == IteratorPhase::End) {
        m_is_end = true;
        return *this;
    }

    if (m_phase == IteratorPhase::Intermediate) {
        // switch to backward phase
        t = m_mesh->switch_tuple(m_simplex.tuple(), PE);
        m_phase = IteratorPhase::Backward;
    }

    t = m_mesh->switch_tuples(t, {PF, PE});

    if (t == m_simplex.tuple()) {
        m_is_end = true;
        return *this;
    }

    if (m_mesh->is_boundary(PE, t)) {
        if (m_phase == IteratorPhase::Forward) {
            // check if a backward phase exists
            const Tuple opp_of_input = m_mesh->switch_tuple(m_simplex.tuple(), PE);
            if (m_mesh->is_boundary(PE, opp_of_input)) {
                m_phase = IteratorPhase::End;
            } else {
                m_phase = IteratorPhase::Intermediate;
            }
        } else {
            m_phase = IteratorPhase::End;
        }
    }

    return *this;
}

TopDimensionCofacesIterable::Iterator TopDimensionCofacesIterable::Iterator::step_trimesh_edge()
{
    constexpr PrimitiveType PE = PrimitiveType::Edge;
    constexpr PrimitiveType PF = PrimitiveType::Triangle;

    if (m_phase == IteratorPhase::End || m_mesh->is_boundary(PE, t)) {
        m_is_end = true;
    } else {
        t = m_mesh->switch_tuple(t, PF);
        m_phase = IteratorPhase::End;
    }

    return *this;
}

TopDimensionCofacesIterable::Iterator TopDimensionCofacesIterable::Iterator::step_trimesh_face()
{
    m_is_end = true;
    return *this;
}

} // namespace wmtk::simplex
