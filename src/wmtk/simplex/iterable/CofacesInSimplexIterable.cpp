#include "CofacesInSimplexIterable.hpp"

namespace wmtk::simplex {

CofacesInSimplexIterable::CofacesInSimplexIterable(
    const Mesh& mesh,
    const Simplex& simplex,
    const PrimitiveType in_simplex_type)
    : m_mesh(&mesh)
    , m_simplex(simplex)
    , m_in_simplex_type(in_simplex_type)
{
    assert(in_simplex_type <= mesh.top_simplex_type());
}

CofacesInSimplexIterable::Iterator::Iterator(
    const CofacesInSimplexIterable& container,
    const Tuple& t)
    : m_container(&container)
    , m_t(t)
{
    if (m_container->m_simplex.primitive_type() > m_container->m_in_simplex_type) {
        m_t = Tuple();
    }
}

CofacesInSimplexIterable::Iterator CofacesInSimplexIterable::Iterator::operator++()
{
    constexpr std::array<PrimitiveType, 3> pts = {
        {PrimitiveType::Vertex, PrimitiveType::Edge, PrimitiveType::Triangle}};

    const Mesh& mesh = *(m_container->m_mesh);
    const simplex::Simplex& simplex = m_container->m_simplex;

    const int8_t pt_start = get_primitive_type_id(simplex.primitive_type()) + 1;
    const int8_t pt_end = get_primitive_type_id(m_container->m_in_simplex_type);


    for (int8_t i = pt_start; i < pt_end; ++i) {
        m_t = mesh.switch_tuple(m_t, pts[i]);
    }


    if (m_t == simplex.tuple()) {
        m_t = Tuple();
    }

    return *this;
}

bool CofacesInSimplexIterable::Iterator::operator!=(const Iterator& other) const
{
    return m_t != other.m_t;
}

Tuple& CofacesInSimplexIterable::Iterator::operator*()
{
    return m_t;
}

} // namespace wmtk::simplex
