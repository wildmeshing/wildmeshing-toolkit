#include "ClosedStarIterable.hpp"

#include <wmtk/autogen/SimplexDart.hpp>
#include <wmtk/autogen/local_switch_tuple.hpp>
#include <wmtk/simplex/cofaces_in_simplex_iterable.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::simplex {


ClosedStarIterable::ClosedStarIterable(const Mesh& mesh, const Simplex& simplex)
    : m_mesh(mesh)
    , m_simplex(simplex)
    , m_tdc_itrbl(mesh, simplex, true)
    , m_it_end(m_tdc_itrbl.end())
{}

ClosedStarIterable::Iterator::Iterator(ClosedStarIterable& container, const Tuple& t)
    : m_container(container)
    , m_mesh(container.m_mesh)
    , m_it(container.m_tdc_itrbl, t)
    , m_t(t)
{
    if (m_t.is_null()) {
        return;
    }

    m_pt = get_primitive_type_id(container.m_simplex.primitive_type());

    if (depth() == 3) {
        m_phase = IteratorPhase::OpenStar;
    }

    init();
}

ClosedStarIterable::Iterator& ClosedStarIterable::Iterator::operator++()
{
    if (depth() == 3) {
        return step_depth_3();
    }

    const int8_t s = get_primitive_type_id(m_container.m_simplex.primitive_type());

    if (m_phase == IteratorPhase::Faces) {
        if (step_faces()) {
            return *this;
        } else {
            m_face_counter = 0;
            m_pt = s;
            m_phase = IteratorPhase::OpenStar;
        }
    }

    if (s == m_mesh.top_cell_dimension()) {
        m_t = Tuple();
        m_pt = -1;
        return *this;
    }

    switch (m_mesh.top_simplex_type()) {
    case PrimitiveType::Edge: return step_edge_mesh();
    case PrimitiveType::Triangle: return step_tri_mesh();
    case PrimitiveType::Tetrahedron: return step_tet_mesh();
    case PrimitiveType::Vertex:
    default: assert(false); break;
    }

    // unreachable code
    m_t = Tuple();
    m_pt = -1;
    return *this;
}

bool ClosedStarIterable::Iterator::operator!=(const Iterator& other) const
{
    return (m_t != other.m_t) || (m_pt != other.m_pt);
}

IdSimplex ClosedStarIterable::Iterator::operator*()
{
    return m_container.m_mesh.get_id_simplex(m_t, get_primitive_type_from_id(m_pt));
}

const IdSimplex ClosedStarIterable::Iterator::operator*() const
{
    return m_container.m_mesh.get_id_simplex(m_t, get_primitive_type_from_id(m_pt));
}

int64_t ClosedStarIterable::Iterator::depth()
{
    const simplex::Simplex& simplex = m_container.m_simplex;
    assert(m_mesh.top_cell_dimension() >= get_primitive_type_id(simplex.primitive_type()));
    assert(m_mesh.top_cell_dimension() - get_primitive_type_id(simplex.primitive_type()) < 4);

    return m_mesh.top_cell_dimension() - get_primitive_type_id(simplex.primitive_type());
}

void ClosedStarIterable::Iterator::init() {}

ClosedStarIterable::Iterator& ClosedStarIterable::Iterator::step_depth_3()
{
    const simplex::Simplex& simplex = m_container.m_simplex;
    auto& visited_c = m_container.m_visited_cofaces;
    auto& visited_l = m_container.m_visited_link;

    assert(m_mesh.top_simplex_type() == PrimitiveType::Tetrahedron);
    assert(simplex.primitive_type() == PrimitiveType::Vertex);

    ++m_pt;
    while (true) {
        if (m_phase == IteratorPhase::OpenStar) {
            if (m_pt == 4) {
                // go to link
                m_t = navigate_to_link(*m_it);
                m_pt = 0;
                m_face_counter = 0;
                m_phase = IteratorPhase::Link;

            } else {
                for (; m_face_counter < 3; ++m_face_counter) {
                    for (; m_pt < 3; ++m_pt) {
                        if (!visited_c[m_pt - 1].is_visited(
                                m_mesh.get_id_simplex(m_t, get_primitive_type_from_id(m_pt)))) {
                            return *this;
                        }
                    }
                    m_t = m_mesh.switch_tuples(m_t, {PrimitiveType::Edge, PrimitiveType::Triangle});
                    m_pt = 1;
                }

                // return tet
                m_pt = 3;
                return *this;
            }
        }
        if (m_phase == IteratorPhase::Link) {
            if (m_pt == 3) {
                // go to next cell
                m_phase = IteratorPhase::OpenStar;
                m_pt = 1;
                m_face_counter = 0;
                ++m_it;
                m_t = *m_it;
                if (m_t.is_null()) {
                    m_pt = -1;
                    // m_pt = m_pt;
                    return *this;
                }
            } else {
                for (; m_face_counter < 3; ++m_face_counter) {
                    for (; m_pt < 2; ++m_pt) {
                        if (!visited_l[m_pt].is_visited(
                                m_mesh.get_id_simplex(m_t, get_primitive_type_from_id(m_pt)))) {
                            return *this;
                        }
                    }
                    m_t = m_mesh.switch_tuples(m_t, {PrimitiveType::Vertex, PrimitiveType::Edge});
                    m_pt = 0;
                }

                // return face
                m_pt = 2;
                return *this;
            }
        }
    }
}

ClosedStarIterable::Iterator& ClosedStarIterable::Iterator::step_edge_mesh()
{
    ++m_face_counter;

    switch (m_face_counter) {
    case 1: m_pt = 1; return *this;
    case 2:
        m_pt = 0;
        m_t = m_mesh.switch_tuple(m_t, PrimitiveType::Vertex);
        return *this;
    default: break;
    }
    m_pt = 1;
    m_face_counter = 1;

    ++m_it;
    m_t = *m_it;

    if (m_t.is_null()) {
        m_pt = -1;
    }

    return *this;
}

ClosedStarIterable::Iterator& ClosedStarIterable::Iterator::step_tri_mesh()
{
    const simplex::Simplex& simplex = m_container.m_simplex;

    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;

    ++m_face_counter;

    switch (simplex.primitive_type()) {
    case PV: {
        if (m_it.is_intermediate() && m_face_counter == 2) {
            m_pt = 0;
            m_t = m_mesh.switch_tuple(m_t, PV);
            m_face_counter = 4;
            return *this;
        }
        switch (m_face_counter) {
        case 1: m_pt = 1; return *this;
        case 2: m_pt = 2; return *this;
        case 3:
            m_pt = 0;
            m_t = m_mesh.switch_tuples(m_t, {PV, PE});
            return *this;
        case 4: m_pt = 1; return *this;
        default: break;
        }
        m_pt = 1;
        break;
    }
    case PE: {
        switch (m_face_counter) {
        case 1: m_pt = 2; return *this;
        case 2:
            m_pt = 1;
            m_t = m_mesh.switch_tuple(m_t, PE);
            return *this;
        case 3:
            m_pt = 0;
            m_t = m_mesh.switch_tuples(m_t, {PV, PE});
            return *this;
        case 4: m_pt = 1; return *this;
        default: break;
        }
        m_pt = 2;
        break;
    }
    case PrimitiveType::Triangle:
    case PrimitiveType::Tetrahedron:
    default: assert(false); break;
    }

    m_face_counter = 1;

    ++m_it;
    m_t = *m_it;

    if (m_t.is_null()) {
        m_pt = -1;
    }

    return *this;
}

ClosedStarIterable::Iterator& ClosedStarIterable::Iterator::step_tet_mesh()
{
    const simplex::Simplex& simplex = m_container.m_simplex;

    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;
    constexpr PrimitiveType PF = PrimitiveType::Triangle;

    ++m_face_counter;

    switch (simplex.primitive_type()) {
    case PE: {
        if (m_it.is_intermediate() && m_face_counter == 5) {
            m_pt = 2;
            break;
        }
        switch (m_face_counter) {
        case 1: m_pt = 2; return *this; // coface triangle
        case 2: // link vertex
            m_pt = 0;
            m_t = m_mesh.switch_tuples(m_t, {PE, PV});
            return *this;
        case 3: m_pt = 1; return *this;
        case 4: m_t = m_mesh.switch_tuple(m_t, PE); return *this;
        case 5: m_t = m_mesh.switch_tuples(m_t, {PF, PE}); return *this;
        case 6: m_pt = 2; return *this;
        case 7: m_t = m_mesh.switch_tuple(m_t, PF); return *this;
        case 8: m_pt = 3; return *this;
        default: break;
        }
        m_pt = 2;
        break;
    }
    case PF: {
        switch (m_face_counter) {
        case 1: m_pt = 3; return *this;
        case 2: // link vertex
            m_pt = 0;
            m_t = m_mesh.switch_tuples(m_t, {PF, PE, PV});
            return *this;
        case 3: m_pt = 1; return *this;
        case 4: m_pt = 2; return *this;
        case 5:
            m_pt = 1;
            m_t = m_mesh.switch_tuples(m_t, {PF, PE});
            return *this;
        case 6: m_pt = 2; return *this;
        case 7:
            m_pt = 1;
            m_t = m_mesh.switch_tuples(m_t, {PF, PE});
            return *this;
        case 8: m_pt = 2; return *this;
        default: break;
        }
        m_pt = 3;
        break;
    }
    case PrimitiveType::Tetrahedron:
    case PrimitiveType::Vertex:
    default: break;
    }

    m_face_counter = 1;

    ++m_it;
    m_t = *m_it;

    if (m_t.is_null()) {
        m_pt = -1;
    }

    return *this;
}

Tuple ClosedStarIterable::Iterator::navigate_to_link(Tuple t)
{
    if (t.is_null()) {
        return t;
    }
    const simplex::Simplex& simplex = m_container.m_simplex;

    /*
     * Assume a tuple that contains the vertices (a,b,c,d) and the simplex is an edge, i.e.,
     * (a,b). The link contains all the vertices that are not in the simplex. To get a tuple
     * that represents all simplices of the link, we need to move (a,b) to the end of that
     * tuple.
     * (a,b,c,d) becomes (c,d,a,b) with the following permutations
     *              (a,b,c,d)
     * switch edge: (a,c,b,d)
     * switch face: (a,c,d,b)
     * switch vert: (c,a,d,b)
     * switch edge: (c,d,a,b)
     *
     * The following code implements these permutations.
     */
    const int8_t m = m_mesh.top_cell_dimension();
    const int8_t s = get_primitive_type_id(simplex.primitive_type());

    for (int8_t j = s; j > -1; --j) {
        for (int8_t i = 0; i < m - s; ++i) {
            t = m_mesh.switch_tuple(t, get_primitive_type_from_id(j + i));
        }
    }


    return t;
}

bool ClosedStarIterable::Iterator::step_faces()
{
    switch (m_container.m_simplex.primitive_type()) {
    case PrimitiveType::Vertex: return false;
    case PrimitiveType::Edge: return step_faces_edge();
    case PrimitiveType::Triangle: return step_faces_triangle();
    case PrimitiveType::Tetrahedron: return step_faces_tetrahedron();
    default: assert(false);
    }
    return false;
}

bool ClosedStarIterable::Iterator::step_faces_edge()
{
    if (m_face_counter == 2) {
        return false;
    }
    m_pt = 0;
    m_t = m_container.m_mesh.switch_tuple(m_t, PrimitiveType::Vertex);
    ++m_face_counter;
    return true;
}

bool ClosedStarIterable::Iterator::step_faces_triangle()
{
    ++m_pt;
    if (m_pt > 1) {
        if (m_face_counter == 3) {
            return false;
        }
        ++m_face_counter;
        m_pt = 0;
        m_t = m_container.m_mesh.switch_tuples(m_t, {PrimitiveType::Vertex, PrimitiveType::Edge});
    }
    return true;
}

bool ClosedStarIterable::Iterator::step_faces_tetrahedron()
{
    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;
    constexpr PrimitiveType PF = PrimitiveType::Triangle;

    switch (m_face_counter) {
    case 0: m_pt = 0; break; // the tet itself
    case 1: m_pt = 1; break;
    case 2:
        m_t = m_mesh.switch_tuples(m_t, {PV, PE});
        m_pt = 0;
        break;
    case 3: m_pt = 1; break;
    case 4:
        m_t = m_mesh.switch_tuples(m_t, {PV, PE});
        m_pt = 0;
        break;
    case 5: m_pt = 1; break;
    case 6: m_pt = 2; break; // base triangle
    case 7: // opposite vertex
        m_t = m_mesh.switch_tuples(m_t, {PF, PE, PV});
        m_pt = 0;
        break;
    case 8: m_pt = 1; break;
    case 9: m_pt = 2; break;
    case 10:
        m_t = m_mesh.switch_tuples(m_t, {PF, PE});
        m_pt = 1;
        break;
    case 11: m_pt = 2; break;
    case 12:
        m_t = m_mesh.switch_tuples(m_t, {PF, PE});
        m_pt = 1;
        break;
    case 13: m_pt = 2; break;
    default: return false;
    }

    ++m_face_counter;
    return true;
}

} // namespace wmtk::simplex
