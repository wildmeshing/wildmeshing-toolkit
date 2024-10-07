#include "ClosedStarIterable.hpp"

#include <wmtk/autogen/SimplexDart.hpp>
#include <wmtk/autogen/local_switch_tuple.hpp>
#include <wmtk/simplex/cofaces_in_simplex_iterable.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleInspector.hpp>

namespace wmtk::simplex {


ClosedStarIterable::ClosedStarIterable(const Mesh& mesh, const Simplex& simplex)
    : m_mesh(&mesh)
    , m_simplex(simplex)
    , m_tdc_itrbl(mesh, simplex, true)
    , m_it_end(m_tdc_itrbl.end())
{}

ClosedStarIterable::Iterator::Iterator(ClosedStarIterable& container, const Tuple& t)
    : m_container(&container)
    , m_it(container.m_tdc_itrbl, t)
    , m_t(t)
{
    if (m_t.is_null()) {
        return;
    }

    m_pt = get_primitive_type_id(container.m_simplex.primitive_type());
    m_sub_pt = 0;

    if (depth() == 3) {
        m_sub_pt = m_pt;
        m_phase = IteratorPhase::OpenStar;
    }

    init();
}

ClosedStarIterable::Iterator& ClosedStarIterable::Iterator::operator++()
{
    if (depth() == 3) {
        return step_depth_3();
    }


    const Mesh& mesh = *(m_container->m_mesh);
    const simplex::Simplex& simplex = m_container->m_simplex;
    const int8_t m = mesh.top_cell_dimension();
    const int8_t s = get_primitive_type_id(simplex.primitive_type());

    if (m_phase == IteratorPhase::Faces) {
        if (s > 0) {
            step_faces();
            return *this;
        } else {
            m_phase = IteratorPhase::OpenStar;
        }
    }

    if (s == m) {
        m_t = Tuple();
        m_pt = -1;
        m_sub_pt = m_pt;
        return *this;
    }

    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;
    constexpr PrimitiveType PF = PrimitiveType::Triangle;
    constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

    ++m_face_counter;
    switch (mesh.top_simplex_type()) {
    case PE: {
        switch (m_face_counter) {
        case 1: m_sub_pt = 1; return *this;
        case 2:
            m_t = mesh.switch_tuple(m_t, PV);
            m_sub_pt = 0;
            return *this;
        default: break;
        }
        m_sub_pt = 1;
        m_face_counter = 1;
        break;
    }
    case PF: {
        switch (simplex.primitive_type()) {
        case PV: {
            if (m_it.is_intermediate() && m_face_counter == 2) {
                m_sub_pt = 0;
                m_t = mesh.switch_tuple(m_t, PV);
                m_face_counter = 4;
                return *this;
            }
            switch (m_face_counter) {
            case 1: m_sub_pt = 1; return *this;
            case 2: m_sub_pt = 2; return *this;
            case 3:
                m_sub_pt = 0;
                m_t = mesh.switch_tuples(m_t, {PV, PE});
                return *this;
            case 4: m_sub_pt = 1; return *this;
            default: break;
            }
            m_sub_pt = 1;
            break;
        }
        case PE: {
            switch (m_face_counter) {
            case 1: m_sub_pt = 2; return *this;
            case 2:
                m_sub_pt = 1;
                m_t = mesh.switch_tuple(m_t, PE);
                return *this;
            case 3:
                m_sub_pt = 0;
                m_t = mesh.switch_tuples(m_t, {PV, PE});
                return *this;
            case 4: m_sub_pt = 1; return *this;
            default: break;
            }
            m_sub_pt = 2;
            break;
        }
        default: break;
        }
        m_face_counter = 1;
        break;
    }
    case PT: {
        switch (simplex.primitive_type()) {
        case PE: {
            if (m_it.is_intermediate() && m_face_counter == 5) {
                m_sub_pt = 2;
                break;
            }
            switch (m_face_counter) {
            case 1: m_sub_pt = 2; return *this; // coface triangle
            case 2: // link vertex
                m_sub_pt = 0;
                m_t = mesh.switch_tuples(m_t, {PE, PV});
                return *this;
            case 3: m_sub_pt = 1; return *this;
            case 4: m_t = mesh.switch_tuple(m_t, PE); return *this;
            case 5: m_t = mesh.switch_tuples(m_t, {PF, PE}); return *this;
            case 6: m_sub_pt = 2; return *this;
            case 7: m_t = mesh.switch_tuple(m_t, PF); return *this;
            case 8: m_sub_pt = 3; return *this;
            default: break;
            }
            m_sub_pt = 2;
            break;
        }
        case PF: {
            switch (m_face_counter) {
            case 1: m_sub_pt = 3; return *this;
            case 2: // link vertex
                m_sub_pt = 0;
                m_t = mesh.switch_tuples(m_t, {PF, PE, PV});
                return *this;
            case 3: m_sub_pt = 1; return *this;
            case 4: m_sub_pt = 2; return *this;
            case 5:
                m_sub_pt = 1;
                m_t = mesh.switch_tuples(m_t, {PF, PE});
                return *this;
            case 6: m_sub_pt = 2; return *this;
            case 7:
                m_sub_pt = 1;
                m_t = mesh.switch_tuples(m_t, {PF, PE});
                return *this;
            case 8: m_sub_pt = 2; return *this;
            default: break;
            }
            m_sub_pt = 3;
            break;
        }
        default: break;
        }
        m_face_counter = 1;
        break;
    }
    default: break;
    }

    // if (m_phase == IteratorPhase::OpenStar) {
    //     if (m_sub_pt < m && !m_it.is_intermediate()) {
    //        // go to next primitive type
    //        ++m_sub_pt;
    //        return *this;
    //    } else {
    //        m_phase = IteratorPhase::Link;
    //        m_t = navigate_to_link(*m_it);
    //        m_pt = 0;
    //        m_sub_pt = 0;
    //        return *this;
    //    }
    //}
    //
    // if (m_phase == IteratorPhase::Link) {
    //    if (s != 0 && m_sub_face_counter < 2) {
    //        // go through cofaces of link simplex
    //        m_sub_pt = m_pt + 1;
    //        ++m_sub_face_counter;
    //        if (m_sub_face_counter != 1) {
    //            m_t = mesh.switch_tuple(m_t, get_primitive_type_from_id(m_sub_pt));
    //        }
    //        return *this;
    //    }
    //    m_sub_face_counter = 0;
    //
    //    if (m_pt < m - s - 1 && !m_it.is_intermediate()) {
    //        // go to next primitive type
    //        ++m_pt;
    //        m_sub_pt = m_pt;
    //    } else {
    //        m_phase = IteratorPhase::OpenStar;
    //        m_pt = s + 1;
    //        m_sub_pt = m_pt;
    //        // change tuple
    //        ++m_it;
    //        m_t = *m_it;
    //    }
    //}

    ++m_it;
    m_t = *m_it;

    if (m_t.is_null()) {
        m_pt = -1;
        m_sub_pt = m_pt;
    }

    return *this;
}

bool ClosedStarIterable::Iterator::operator!=(const Iterator& other) const
{
    return (m_t != other.m_t) || (m_pt != other.m_pt);
}

IdSimplex ClosedStarIterable::Iterator::operator*()
{
    return m_container->m_mesh->get_id_simplex(m_t, get_primitive_type_from_id(m_sub_pt));
}

const IdSimplex ClosedStarIterable::Iterator::operator*() const
{
    return m_container->m_mesh->get_id_simplex(m_t, get_primitive_type_from_id(m_sub_pt));
}

int64_t ClosedStarIterable::Iterator::depth()
{
    const Mesh& mesh = *(m_container->m_mesh);
    const simplex::Simplex& simplex = m_container->m_simplex;
    assert(mesh.top_cell_dimension() >= get_primitive_type_id(simplex.primitive_type()));
    assert(mesh.top_cell_dimension() - get_primitive_type_id(simplex.primitive_type()) < 4);

    return mesh.top_cell_dimension() - get_primitive_type_id(simplex.primitive_type());
}

void ClosedStarIterable::Iterator::init() {}

ClosedStarIterable::Iterator& ClosedStarIterable::Iterator::step_depth_3()
{
    const Mesh& mesh = *(m_container->m_mesh);
    const simplex::Simplex& simplex = m_container->m_simplex;
    auto& visited_c = m_container->m_visited_cofaces;
    auto& visited_l = m_container->m_visited_link;

    assert(mesh.top_simplex_type() == PrimitiveType::Tetrahedron);
    assert(simplex.primitive_type() == PrimitiveType::Vertex);

    ++m_sub_pt;
    while (true) {
        if (m_phase == IteratorPhase::OpenStar) {
            if (m_sub_pt == 4) {
                // go to link
                m_t = navigate_to_link(*m_it);
                m_sub_pt = 0;
                m_face_counter = 0;
                m_phase = IteratorPhase::Link;

            } else {
                for (; m_face_counter < 3; ++m_face_counter) {
                    for (; m_sub_pt < 3; ++m_sub_pt) {
                        if (!visited_c[m_sub_pt - 1].is_visited(
                                mesh.get_id_simplex(m_t, get_primitive_type_from_id(m_sub_pt)))) {
                            return *this;
                        }
                    }
                    m_t = mesh.switch_tuples(m_t, {PrimitiveType::Edge, PrimitiveType::Triangle});
                    m_sub_pt = 1;
                }

                // return tet
                m_sub_pt = 3;
                return *this;
            }
        }
        if (m_phase == IteratorPhase::Link) {
            if (m_sub_pt == 3) {
                // go to next cell
                m_phase = IteratorPhase::OpenStar;
                m_sub_pt = 1;
                m_face_counter = 0;
                ++m_it;
                m_t = *m_it;
                if (m_t.is_null()) {
                    m_pt = -1;
                    m_sub_pt = m_pt;
                    return *this;
                }
            } else {
                for (; m_face_counter < 3; ++m_face_counter) {
                    for (; m_sub_pt < 2; ++m_sub_pt) {
                        if (!visited_l[m_sub_pt].is_visited(
                                mesh.get_id_simplex(m_t, get_primitive_type_from_id(m_sub_pt)))) {
                            return *this;
                        }
                    }
                    m_t = mesh.switch_tuples(m_t, {PrimitiveType::Vertex, PrimitiveType::Edge});
                    m_sub_pt = 0;
                }

                // return face
                m_sub_pt = 2;
                return *this;
            }
        }
    }
}

Tuple ClosedStarIterable::Iterator::navigate_to_link(Tuple t)
{
    if (t.is_null()) {
        return t;
    }
    // invert the simplex using SimplexDart
    const Mesh& mesh = *(m_container->m_mesh);
    // const PrimitiveType& mesh_pt = mesh.top_simplex_type();
    // autogen::SimplexDart sd(mesh_pt);

    // switch (mesh.top_simplex_type()) {
    // case PrimitiveType::Triangle: {
    //     const int8_t index_switch = sd.product(
    //         sd.primitive_as_index(PrimitiveType::Edge),
    //         sd.primitive_as_index(PrimitiveType::Vertex));
    //     m_t = autogen::local_switch_tuple(mesh_pt, m_t, index_switch);
    //     break;
    // }
    // default: log_and_throw_error("missing mesh navigation in link"); break;
    // }

    {
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
        const simplex::Simplex& simplex = m_container->m_simplex;
        const int8_t m = mesh.top_cell_dimension();
        const int8_t s = get_primitive_type_id(simplex.primitive_type());

        for (int8_t j = s; j > -1; --j) {
            for (int8_t i = 0; i < m - s; ++i) {
                t = mesh.switch_tuple(t, get_primitive_type_from_id(j + i));
            }
        }
    }

    return t;
}

void ClosedStarIterable::Iterator::step_faces()
{
    const Mesh& mesh = *(m_container->m_mesh);
    const simplex::Simplex& simplex = m_container->m_simplex;
    const int8_t m = mesh.top_cell_dimension();
    const int8_t s = get_primitive_type_id(simplex.primitive_type());

    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;
    constexpr PrimitiveType PF = PrimitiveType::Triangle;
    constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

    switch (simplex.primitive_type()) {
    case PV: {
        break;
    }
    case PE: {
        ++m_sub_face_counter;
        if (m_sub_face_counter == 2) {
            break;
        }
        m_t = mesh.switch_tuple(m_t, PV);
        return;
    }
    case PF: {
        ++m_sub_pt;
        if (m_sub_pt == 2) {
            m_sub_pt = 0;
            ++m_sub_face_counter;
            if (m_sub_face_counter == 3) {
                break;
            }
            m_t = mesh.switch_tuples(m_t, {PV, PE});
        }
        return;
    }
    case PT: {
        ++m_sub_face_counter;
        switch (m_sub_face_counter) {
        case 1: m_sub_pt = 1; return;
        case 2:
            m_sub_pt = 0;
            m_t = mesh.switch_tuples(m_t, {PV, PE});
            return;
        case 3: m_sub_pt = 1; return;
        case 4:
            m_sub_pt = 0;
            m_t = mesh.switch_tuples(m_t, {PV, PE});
            return;
        case 5: m_sub_pt = 1; return;
        case 6: m_sub_pt = 2; return;
        case 7: // opposite vertex
            m_sub_pt = 0;
            m_t = mesh.switch_tuples(m_t, {PF, PE, PV});
            return;
        case 8: m_sub_pt = 1; return;
        case 9: m_sub_pt = 2; return;
        case 10:
            m_sub_pt = 1;
            m_t = mesh.switch_tuples(m_t, {PF, PE});
            return;
        case 11: m_sub_pt = 2; return;
        case 12:
            m_sub_pt = 1;
            m_t = mesh.switch_tuples(m_t, {PF, PE});
            return;
        case 13: m_sub_pt = 2; return;
        default: break;
        }
        break;
    }
    default: break;
    }

    m_sub_face_counter = 0;
    m_sub_pt = s;
    m_phase = IteratorPhase::OpenStar;
}

} // namespace wmtk::simplex
