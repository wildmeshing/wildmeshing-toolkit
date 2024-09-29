#include "TopDimensionCofacesIterable.hpp"

#include <wmtk/simplex/top_dimension_cofaces.hpp>

namespace wmtk::simplex {


TopDimensionCofacesIterable::TopDimensionCofacesIterable(Mesh& mesh, const Simplex& simplex)
    //: m_collection(top_dimension_cofaces(mesh, simplex))
    : m_mesh(&mesh)
    , m_simplex(simplex)
{}

TopDimensionCofacesIterable::Iterator::Iterator(Mesh& mesh, const Simplex& simplex, bool is_done)
    : m_mesh(&mesh)
    , m_simplex(simplex)
    , t(simplex.tuple())
    , m_phase(IteratorPhase::Forward)
    , m_is_end(is_done)
{
    if (m_is_end) {
        return;
    }

    constexpr PrimitiveType PE = PrimitiveType::Edge;

    // check if forward or backward phase can be executed
    if (m_mesh->is_boundary(PE, t)) {
        m_phase = IteratorPhase::Backward;

        // check if a backward phase exists
        const Tuple opp_of_input = m_mesh->switch_tuple(m_simplex.tuple(), PE);
        if (m_mesh->is_boundary(PE, opp_of_input)) {
            m_phase = IteratorPhase::End;
        }
    }
}

TopDimensionCofacesIterable::Iterator TopDimensionCofacesIterable::Iterator::operator++()
{
    constexpr PrimitiveType PE = PrimitiveType::Edge;
    constexpr PrimitiveType PF = PrimitiveType::Triangle;

    if (m_phase == IteratorPhase::End) {
        m_is_end = true;
        return *this;
    }

    if (m_phase == IteratorPhase::Forward) {
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
                m_phase = IteratorPhase::Backward;
            }
        } else {
            m_phase = IteratorPhase::End;
        }
    }

    return *this;
}

bool TopDimensionCofacesIterable::Iterator::operator!=(const Iterator& other) const
{
    // const bool diff_tuple = (t != other.t);
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

} // namespace wmtk::simplex
