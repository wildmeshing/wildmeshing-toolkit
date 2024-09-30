#include "TopDimensionCofacesIterable.hpp"

#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleInspector.hpp>

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
    , m_t(simplex.tuple())
    , m_phase(IteratorPhase::Forward)
    , m_is_end(is_done)
{
    if (m_is_end) {
        return;
    }

    switch (m_mesh->top_simplex_type()) {
    case PrimitiveType::Triangle: {
        if (m_simplex.primitive_type() == PrimitiveType::Vertex) {
            init_trimesh_vertex();
        }
        return;
    }
    case PrimitiveType::Tetrahedron: {
        init_tetmesh();
        return;
    }
    default:
        log_and_throw_error(
            "TopDimensionCofacesIterable not implemented for that simplex and/or mesh type.");
        break;
    }
}

TopDimensionCofacesIterable::Iterator TopDimensionCofacesIterable::Iterator::operator++()
{
    switch (m_mesh->top_simplex_type()) {
    case PrimitiveType::Triangle: {
        switch (m_simplex.primitive_type()) {
        case PrimitiveType::Vertex: return step_trimesh_vertex();
        case PrimitiveType::Edge: return step_trimesh_edge();
        case PrimitiveType::Triangle: return step_trimesh_face();
        default: assert(false); // unknown simplex type
        }
        break;
    }
    case PrimitiveType::Tetrahedron: {
        switch (m_simplex.primitive_type()) {
        case PrimitiveType::Vertex: return step_tetmesh_vertex();
        case PrimitiveType::Edge: break;
        case PrimitiveType::Triangle: break;
        case PrimitiveType::Tetrahedron: break;
        default: assert(false); // unknown simplex type
        }
        break;
    }
    default: break;
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
    return m_t;
}

const Tuple& TopDimensionCofacesIterable::Iterator::operator*() const
{
    return m_t;
}

void TopDimensionCofacesIterable::Iterator::init_trimesh_vertex()
{
    constexpr PrimitiveType PE = PrimitiveType::Edge;

    // check if forward or backward phase can be executed
    if (m_mesh->is_boundary(PE, m_t)) {
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
        m_t = m_mesh->switch_tuple(m_simplex.tuple(), PE);
        m_phase = IteratorPhase::Backward;
    }

    m_t = m_mesh->switch_tuples(m_t, {PF, PE});

    if (m_t == m_simplex.tuple()) {
        m_is_end = true;
        return *this;
    }

    if (m_mesh->is_boundary(PE, m_t)) {
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

    if (m_phase == IteratorPhase::End || m_mesh->is_boundary(PE, m_t)) {
        m_is_end = true;
    } else {
        m_t = m_mesh->switch_tuple(m_t, PF);
        m_phase = IteratorPhase::End;
    }

    return *this;
}

TopDimensionCofacesIterable::Iterator TopDimensionCofacesIterable::Iterator::step_trimesh_face()
{
    m_is_end = true;
    return *this;
}

void TopDimensionCofacesIterable::Iterator::init_tetmesh()
{
    constexpr PrimitiveType PE = PrimitiveType::Edge;
    constexpr PrimitiveType PF = PrimitiveType::Triangle;
    constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

    switch (m_simplex.primitive_type()) {
    case PrimitiveType::Vertex: {
        m_visited = std::vector<bool>(m_mesh->get_all(PrimitiveType::Tetrahedron).size(), false);

        m_visited[wmtk::utils::TupleInspector::global_cid(m_t)] = true;

        const std::array<Tuple, 3> t_tris = {
            m_t,
            m_mesh->switch_tuple(m_t, PF),
            m_mesh->switch_tuples(m_t, {PE, PF})};

        for (const Tuple& tt : t_tris) {
            if (m_mesh->is_boundary(PF, tt)) {
                continue;
            }
            const Tuple neigh = m_mesh->switch_tuple(tt, PT);
            const int64_t neigh_id = wmtk::utils::TupleInspector::global_cid(neigh);

            m_visited[neigh_id] = true;
            m_queue.push(neigh);
        }

        return;
    }
    case PrimitiveType::Edge: break;
    case PrimitiveType::Triangle: break;
    case PrimitiveType::Tetrahedron: break;
    default: break;
    }
    log_and_throw_error("not implemented");
}

TopDimensionCofacesIterable::Iterator TopDimensionCofacesIterable::Iterator::step_tetmesh_vertex()
{
    constexpr PrimitiveType PE = PrimitiveType::Edge;
    constexpr PrimitiveType PF = PrimitiveType::Triangle;
    constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;


    if (m_queue.empty()) {
        m_is_end = true;
        return *this;
    }

    m_t = m_queue.front();
    m_queue.pop();

    const std::array<Tuple, 3> t_tris = {
        m_t,
        m_mesh->switch_tuple(m_t, PF),
        m_mesh->switch_tuples(m_t, {PE, PF})};

    for (const Tuple& tt : t_tris) {
        if (m_mesh->is_boundary(PF, tt)) {
            continue;
        }
        const Tuple neigh = m_mesh->switch_tuple(tt, PT);
        const int64_t neigh_id = wmtk::utils::TupleInspector::global_cid(neigh);

        if (!m_visited[neigh_id]) {
            m_visited[neigh_id] = true;
            m_queue.push(neigh);
        }
    }


    return *this;
}

} // namespace wmtk::simplex
