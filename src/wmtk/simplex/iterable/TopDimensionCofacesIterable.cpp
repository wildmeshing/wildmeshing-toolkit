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
    const TopDimensionCofacesIterable& container,
    const Tuple& t)
    : m_container(&container)
    , m_t(t)
    , m_phase(IteratorPhase::Forward)
{
    if (m_t.is_null()) {
        return;
    }

    init(depth());
}

TopDimensionCofacesIterable::Iterator& TopDimensionCofacesIterable::Iterator::operator++()
{
    switch (depth()) {
    case 0: return step_depth_0();
    case 1: return step_depth_1();
    case 2: return step_depth_2();
    case 3: return step_depth_3();
    default: break;
    }

    assert(false); // unknown simplex or mesh type
    m_t = Tuple();
    return *this;
}

bool TopDimensionCofacesIterable::Iterator::operator!=(const Iterator& other) const
{
    return m_t != other.m_t;
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
    return get_primitive_type_from_id(m_container->m_mesh->top_cell_dimension() - depth);
}

int64_t TopDimensionCofacesIterable::Iterator::depth()
{
    const Mesh& mesh = *(m_container->m_mesh);
    const simplex::Simplex& simplex = m_container->m_simplex;
    assert(mesh.top_cell_dimension() >= get_primitive_type_id(simplex.primitive_type()));
    assert(mesh.top_cell_dimension() - get_primitive_type_id(simplex.primitive_type()) < 4);

    return mesh.top_cell_dimension() - get_primitive_type_id(simplex.primitive_type());
}

void TopDimensionCofacesIterable::Iterator::init(int64_t depth)
{
    const Mesh& mesh = *(m_container->m_mesh);
    const simplex::Simplex& simplex = m_container->m_simplex;
    // No initialization necessary if simplex is d or d-1.

    if (depth == 2) {
        // d - 2 --> iteration

        // check if forward or backward phase can be executed
        if (mesh.is_boundary(pt(1), m_t)) {
            m_phase = IteratorPhase::Intermediate;
        }
    } else if (depth == 3) {
        // d - 3 --> BFS

        m_visited.is_visited(wmtk::utils::TupleInspector::global_cid(m_t));
        m_queue.push(m_t);

        step_depth_3();
    }
}

TopDimensionCofacesIterable::Iterator& TopDimensionCofacesIterable::Iterator::step_depth_0()
{
    m_t = Tuple();
    return *this;
}

TopDimensionCofacesIterable::Iterator& TopDimensionCofacesIterable::Iterator::step_depth_1()
{
    const Mesh& mesh = *(m_container->m_mesh);
    const simplex::Simplex& simplex = m_container->m_simplex;

    if (m_phase == IteratorPhase::End || mesh.is_boundary(pt(1), m_t)) {
        m_t = Tuple();
        return *this;
    } else {
        m_t = mesh.switch_tuple(m_t, pt(0));
        m_phase = IteratorPhase::End;
        return *this;
    }
}

TopDimensionCofacesIterable::Iterator& TopDimensionCofacesIterable::Iterator::step_depth_2()
{
    const Mesh& mesh = *(m_container->m_mesh);
    const simplex::Simplex& simplex = m_container->m_simplex;

    if (m_phase == IteratorPhase::Intermediate) {
        // go to opposite of input
        m_t = mesh.switch_tuple(simplex.tuple(), pt(1));
        if (mesh.is_boundary(pt(1), m_t)) {
            m_phase = IteratorPhase::End;
        } else {
            // switch to backward phase
            m_phase = IteratorPhase::Backward;
        }
    }

    if (m_phase == IteratorPhase::End) {
        m_t = Tuple();
        return *this;
    }

    m_t = mesh.switch_tuples(m_t, {pt(0), pt(1)});

    if (m_t == simplex.tuple()) {
        m_t = Tuple();
        return *this;
    }

    if (mesh.is_boundary(pt(1), m_t)) {
        if (m_phase == IteratorPhase::Forward) {
            m_phase = IteratorPhase::Intermediate;
        } else {
            m_phase = IteratorPhase::End;
        }
    }

    return *this;
}

TopDimensionCofacesIterable::Iterator& TopDimensionCofacesIterable::Iterator::step_depth_3()
{
    const Mesh& mesh = *(m_container->m_mesh);

    if (m_queue.empty()) {
        m_t = Tuple();
        return *this;
    }

    m_t = m_queue.front();
    m_queue.pop();

    const std::array<Tuple, 3> t_tris = {
        {m_t, mesh.switch_tuple(m_t, pt(1)), mesh.switch_tuples(m_t, {pt(2), pt(1)})}};

    for (const Tuple& tt : t_tris) {
        if (mesh.is_boundary(pt(1), tt)) {
            continue;
        }
        const Tuple neigh = mesh.switch_tuple(tt, pt(0));
        const int64_t neigh_id = wmtk::utils::TupleInspector::global_cid(neigh);

        if (!m_visited.is_visited(neigh_id)) {
            m_queue.push(neigh);
        }
    }


    return *this;
}

} // namespace wmtk::simplex
