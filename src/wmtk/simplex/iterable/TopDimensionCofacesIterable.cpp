#include "TopDimensionCofacesIterable.hpp"

namespace wmtk::simplex {


TopDimensionCofacesIterable::TopDimensionCofacesIterable(
    const Mesh& mesh,
    const Simplex& simplex,
    const bool retrieve_intermediate_tuple)
    : m_mesh(mesh)
    , m_simplex(simplex)
    , m_retrieve_intermediate_tuple(retrieve_intermediate_tuple)
{}

TopDimensionCofacesIterable::Iterator::Iterator(
    TopDimensionCofacesIterable& container,
    const Tuple& t)
    : m_container(container)
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

Tuple& TopDimensionCofacesIterable::Iterator::operator*()
{
    return m_t;
}

const Tuple& TopDimensionCofacesIterable::Iterator::operator*() const
{
    return m_t;
}

PrimitiveType TopDimensionCofacesIterable::Iterator::pt(int64_t depth) const
{
    return get_primitive_type_from_id(m_container.m_mesh.top_cell_dimension() - depth);
}

int64_t TopDimensionCofacesIterable::Iterator::depth()
{
    const Mesh& mesh = m_container.m_mesh;
    const simplex::Simplex& simplex = m_container.m_simplex;
    assert(mesh.top_cell_dimension() >= get_primitive_type_id(simplex.primitive_type()));
    assert(mesh.top_cell_dimension() - get_primitive_type_id(simplex.primitive_type()) < 4);

    return mesh.top_cell_dimension() - get_primitive_type_id(simplex.primitive_type());
}

void TopDimensionCofacesIterable::Iterator::init(int64_t depth)
{
    const Mesh& mesh = m_container.m_mesh;
    const simplex::Simplex& simplex = m_container.m_simplex;
    // No initialization necessary if simplex is d or d-1.

    if (depth == 2) {
        // d - 2 --> iteration

        // check if forward or backward phase can be executed
        if (mesh.is_boundary(pt(1), m_t)) {
            m_phase = IteratorPhase::Intermediate;
        }
    } else if (depth == 3) {
        // d - 3 --> BFS
        m_container.m_visited.is_visited(m_t.global_cid());

        add_neighbors_to_queue();
    }
}

TopDimensionCofacesIterable::Iterator& TopDimensionCofacesIterable::Iterator::step_depth_0()
{
    m_t = Tuple();
    return *this;
}

TopDimensionCofacesIterable::Iterator& TopDimensionCofacesIterable::Iterator::step_depth_1()
{
    const Mesh& mesh = m_container.m_mesh;
    const simplex::Simplex& simplex = m_container.m_simplex;

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
    const Mesh& mesh = m_container.m_mesh;
    const simplex::Simplex& simplex = m_container.m_simplex;

    m_is_intermediate = false;

    if (m_phase == IteratorPhase::Intermediate) {
        // go to opposite of input
        m_t = mesh.switch_tuple(simplex.tuple(), pt(1));
        if (mesh.is_boundary(pt(1), m_t)) {
            m_phase = IteratorPhase::End;
        } else {
            // switch to backward phase
            m_phase = IteratorPhase::Backward;
        }
        if (m_container.m_retrieve_intermediate_tuple) {
            m_is_intermediate = true;
            return *this;
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
    const Mesh& mesh = m_container.m_mesh;
    auto& q = m_container.m_q;
    auto& q_front = m_container.m_q_front;

    if (q_front == q.size()) {
        m_t = Tuple();
        return *this;
    }

    m_t = q[q_front++];

    add_neighbors_to_queue();


    return *this;
}

void TopDimensionCofacesIterable::Iterator::add_neighbors_to_queue()
{
    const Mesh& mesh = m_container.m_mesh;
    const simplex::Simplex& simplex = m_container.m_simplex;

    assert(mesh.top_simplex_type() == PrimitiveType::Tetrahedron);
    assert(simplex.primitive_type() == PrimitiveType::Vertex);

    Tuple t = m_t;
    for (size_t i = 0; i < 3; ++i) {
        if (!mesh.is_boundary(PrimitiveType::Triangle, t)) {
            const Tuple neigh = mesh.switch_tuple(t, PrimitiveType::Tetrahedron);
            const int64_t neigh_id = neigh.global_cid();

            if (!m_container.m_visited.is_visited(neigh_id)) {
                m_container.m_q.emplace_back(neigh);
            }
        }
        t = mesh.switch_tuples(t, {PrimitiveType::Edge, PrimitiveType::Triangle});
    }
}

bool TopDimensionCofacesIterable::Iterator::is_intermediate() const
{
    return m_is_intermediate;
}

} // namespace wmtk::simplex
