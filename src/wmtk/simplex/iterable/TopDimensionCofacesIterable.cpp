#include "TopDimensionCofacesIterable.hpp"

#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleInspector.hpp>

namespace wmtk::simplex {


TopDimensionCofacesIterable::TopDimensionCofacesIterable(const Mesh& mesh, const Simplex& simplex)
    : m_mesh(&mesh)
    , m_simplex(simplex)
{}

TopDimensionCofacesIterable::Iterator::Iterator(
    const Mesh& mesh,
    const Simplex& simplex,
    bool is_end)
    : m_mesh(&mesh)
    , m_simplex(simplex)
    , m_t(simplex.tuple())
    , m_phase(IteratorPhase::Forward)
    , m_is_end(is_end)
{
    if (m_is_end) {
        return;
    }

    init(depth());
}

TopDimensionCofacesIterable::Iterator TopDimensionCofacesIterable::Iterator::operator++()
{
    switch (depth()) {
    case 0: return step_depth_0();
    case 1: return step_depth_1();
    case 2: return step_depth_2();
    case 3: return step_depth_3();
    default: break;
    }

    assert(false); // unknown simplex or mesh type
    return Iterator(*m_mesh, m_simplex, true);
}

bool TopDimensionCofacesIterable::Iterator::operator!=(const Iterator& other) const
{
    const bool diff_end = (m_is_end != other.m_is_end);
    return diff_end;
}

Tuple TopDimensionCofacesIterable::Iterator::operator*()
{
    return m_t;
}

const Tuple& TopDimensionCofacesIterable::Iterator::operator*() const
{
    return m_t;
}

PrimitiveType TopDimensionCofacesIterable::Iterator::pt(int64_t depth) const
{
    return get_primitive_type_from_id(m_mesh->top_cell_dimension() - depth);
}

int64_t TopDimensionCofacesIterable::Iterator::depth()
{
    assert(m_mesh->top_cell_dimension() >= get_primitive_type_id(m_simplex.primitive_type()));
    assert(m_mesh->top_cell_dimension() - get_primitive_type_id(m_simplex.primitive_type()) < 4);

    return m_mesh->top_cell_dimension() - get_primitive_type_id(m_simplex.primitive_type());
}

void TopDimensionCofacesIterable::Iterator::init(int64_t depth)
{
    // No initialization necessary if simplex is d or d-1.

    if (depth == 2) {
        // d - 2 --> iteration

        // check if forward or backward phase can be executed
        if (m_mesh->is_boundary(pt(1), m_t)) {
            m_phase = IteratorPhase::Intermediate;

            // check if a backward phase exists
            const Tuple opp_of_input = m_mesh->switch_tuple(m_simplex.tuple(), pt(1));
            if (m_mesh->is_boundary(pt(1), opp_of_input)) {
                m_phase = IteratorPhase::End;
            }
        }
    } else if (depth == 3) {
        // d - 3 --> BFS

        m_visited = std::vector<bool>(m_mesh->get_all(PrimitiveType::Tetrahedron).size(), false);

        m_visited[wmtk::utils::TupleInspector::global_cid(m_t)] = true;

        const std::array<Tuple, 3> t_tris = {
            {m_t, m_mesh->switch_tuple(m_t, pt(1)), m_mesh->switch_tuples(m_t, {pt(2), pt(1)})}};

        for (const Tuple& tt : t_tris) {
            if (m_mesh->is_boundary(pt(1), tt)) {
                continue;
            }
            const Tuple neigh = m_mesh->switch_tuple(tt, pt(0));
            const int64_t neigh_id = wmtk::utils::TupleInspector::global_cid(neigh);

            m_visited[neigh_id] = true;
            m_queue.push(neigh);
        }
    }
}

TopDimensionCofacesIterable::Iterator TopDimensionCofacesIterable::Iterator::step_depth_0()
{
    m_is_end = true;
    return *this;
}

TopDimensionCofacesIterable::Iterator TopDimensionCofacesIterable::Iterator::step_depth_1()
{
    if (m_phase == IteratorPhase::End || m_mesh->is_boundary(pt(1), m_t)) {
        m_is_end = true;
    } else {
        m_t = m_mesh->switch_tuple(m_t, pt(0));
        m_phase = IteratorPhase::End;
    }

    return *this;
}

TopDimensionCofacesIterable::Iterator TopDimensionCofacesIterable::Iterator::step_depth_2()
{
    if (m_phase == IteratorPhase::End) {
        m_is_end = true;
        return *this;
    }

    if (m_phase == IteratorPhase::Intermediate) {
        // switch to backward phase
        m_t = m_mesh->switch_tuple(m_simplex.tuple(), pt(1));
        m_phase = IteratorPhase::Backward;
    }

    m_t = m_mesh->switch_tuples(m_t, {pt(0), pt(1)});

    if (m_t == m_simplex.tuple()) {
        m_is_end = true;
        return *this;
    }

    if (m_mesh->is_boundary(pt(1), m_t)) {
        if (m_phase == IteratorPhase::Forward) {
            // check if a backward phase exists
            const Tuple opp_of_input = m_mesh->switch_tuple(m_simplex.tuple(), pt(1));
            if (m_mesh->is_boundary(pt(1), opp_of_input)) {
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

TopDimensionCofacesIterable::Iterator TopDimensionCofacesIterable::Iterator::step_depth_3()
{
    if (m_queue.empty()) {
        m_is_end = true;
        return *this;
    }

    m_t = m_queue.front();
    m_queue.pop();

    const std::array<Tuple, 3> t_tris = {
        {m_t, m_mesh->switch_tuple(m_t, pt(1)), m_mesh->switch_tuples(m_t, {pt(2), pt(1)})}};

    for (const Tuple& tt : t_tris) {
        if (m_mesh->is_boundary(pt(1), tt)) {
            continue;
        }
        const Tuple neigh = m_mesh->switch_tuple(tt, pt(0));
        const int64_t neigh_id = wmtk::utils::TupleInspector::global_cid(neigh);

        if (!m_visited[neigh_id]) {
            m_visited[neigh_id] = true;
            m_queue.push(neigh);
        }
    }


    return *this;
}

} // namespace wmtk::simplex
