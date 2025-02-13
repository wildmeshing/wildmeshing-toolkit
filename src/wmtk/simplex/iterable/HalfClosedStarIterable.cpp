#include "HalfClosedStarIterable.hpp"

#include <wmtk/autogen/SimplexDart.hpp>
#include <wmtk/autogen/local_switch_tuple.hpp>
#include <wmtk/simplex/cofaces_in_simplex_iterable.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::simplex {


HalfClosedStarIterable::HalfClosedStarIterable(const Mesh& mesh, const Tuple& tuple)
    : m_mesh(mesh)
    , m_tuple(tuple)
    , m_tdc_itrbl(mesh, Simplex(mesh, PrimitiveType::Edge, tuple), true)
    , m_it_end(m_tdc_itrbl.end())
{}

HalfClosedStarIterable::Iterator::Iterator(HalfClosedStarIterable& container, const Tuple& t)
    : m_container(container)
    , m_mesh(container.m_mesh)
    , m_it(container.m_tdc_itrbl, t)
    , m_t(t)
{
    if (m_t.is_null()) {
        return;
    }

    m_pt = 1;
}

HalfClosedStarIterable::Iterator& HalfClosedStarIterable::Iterator::operator++()
{
    if (m_phase == IteratorPhase::Faces) {
        if (step_faces()) {
            return *this;
        } else {
            m_face_counter = 0;
            m_pt = 1;
            m_phase = IteratorPhase::OpenStar;
        }
    }

    if (m_mesh.top_simplex_type() == PrimitiveType::Edge) {
        m_t = Tuple();
        m_pt = -1;
        return *this;
    }

    switch (m_mesh.top_simplex_type()) {
    case PrimitiveType::Triangle: return step_tri_mesh();
    case PrimitiveType::Tetrahedron: return step_tet_mesh();
    case PrimitiveType::Edge:
    case PrimitiveType::Vertex:
    default: assert(false); break;
    }

    // unreachable code
    m_t = Tuple();
    m_pt = -1;
    return *this;
}

bool HalfClosedStarIterable::Iterator::operator!=(const Iterator& other) const
{
    return (m_t != other.m_t) || (m_pt != other.m_pt);
}

IdSimplex HalfClosedStarIterable::Iterator::operator*()
{
    return m_container.m_mesh.get_id_simplex(m_t, get_primitive_type_from_id(m_pt));
}

const IdSimplex HalfClosedStarIterable::Iterator::operator*() const
{
    return m_container.m_mesh.get_id_simplex(m_t, get_primitive_type_from_id(m_pt));
}

HalfClosedStarIterable::Iterator& HalfClosedStarIterable::Iterator::step_tri_mesh()
{
    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;

    ++m_face_counter;

    switch (m_face_counter) {
    case 1: m_pt = 2; return *this;
    case 2:
        m_pt = 1;
        m_t = m_mesh.switch_tuple(m_t, PE);
        return *this;
    default: break;
    }
    m_pt = 2;
    m_face_counter = 1;

    ++m_it;
    m_t = *m_it;

    if (m_t.is_null()) {
        m_pt = -1;
    }

    return *this;
}

HalfClosedStarIterable::Iterator& HalfClosedStarIterable::Iterator::step_tet_mesh()
{
    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;
    constexpr PrimitiveType PF = PrimitiveType::Triangle;

    ++m_face_counter;

    if (m_it.is_intermediate() && m_face_counter == 3) {
        m_pt = 2;
        m_face_counter = 1;
        ++m_it;
        m_t = *m_it;
        if (m_t.is_null()) {
            m_pt = -1;
        }
        return *this;
    }
    switch (m_face_counter) {
    case 1: m_pt = 2; return *this; // coface triangle
    case 2:
        m_t = m_mesh.switch_tuples(m_t, {PE, PF});
        m_pt = 1;
        return *this;
    case 3: m_pt = 2; return *this;
    case 4: m_pt = 3; return *this;
    default: break;
    }
    m_pt = 2;
    m_face_counter = 1;

    ++m_it;
    m_t = *m_it;

    if (m_t.is_null()) {
        m_pt = -1;
    }

    return *this;
}

bool HalfClosedStarIterable::Iterator::step_faces()
{
    if (m_face_counter == 1) {
        return false;
    }
    m_pt = 0;
    ++m_face_counter;
    return true;
}

} // namespace wmtk::simplex
