#include "Primitive.hpp"
#include <tuple>


namespace wmtk {

Primitive::Primitive(const PrimitiveType& primitive_type, const Tuple& t)
    : m_primitive_type{primitive_type}
    , m_tuple{t}
{}
Primitive::Primitive(const simplex::Simplex& simplex)
    : m_primitive_type{simplex.primitive_type()}
    , m_tuple{simplex.tuple()}
{}
Primitive::Primitive(const Cell& cell)
    : m_primitive_type{get_primitive_type_from_id(cell.dimension())}
    , m_tuple{cell.tuple()}
{}

PrimitiveType Primitive::primitive_type() const
{
    return m_primitive_type;
}
const Tuple& Primitive::tuple() const
{
    return m_tuple;
}

Primitive Primitive::vertex(const Tuple& t)
{
    return Primitive(PrimitiveType::Vertex, t);
}
Primitive Primitive::edge(const Tuple& t)
{
    return Primitive(PrimitiveType::Edge, t);
}
Primitive Primitive::face(const Tuple& t)
{
    return Primitive(PrimitiveType::Face, t);
}
Primitive Primitive::tetrahedron(const Tuple& t)
{
    return Primitive(PrimitiveType::Tetrahedron, t);
}

bool Primitive::operator==(const Primitive& o) const
{
    return m_primitive_type == o.m_primitive_type && m_tuple == o.m_tuple;
}

bool Primitive::operator<(const Primitive& o) const
{
    return std::tie(m_primitive_type, m_tuple) < std::tie(o.m_primitive_type, o.m_tuple);
}
} // namespace wmtk
