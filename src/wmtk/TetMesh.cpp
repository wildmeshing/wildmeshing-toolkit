#include "TetMesh.hpp"

namespace wmtk {
TetMesh::TetMesh()
    : m_vt_handle(register_attribute<long>("m_vt", PrimitiveType::Vertex, 1))
    , m_et_handle(register_attribute<long>("m_et", PrimitiveType::Edge, 1))
    , m_ft_handle(register_attribute<long>("m_ft", PrimitiveType::Face, 1))
    , m_tv_handle(register_attribute<long>("m_tv", PrimitiveType::Tetrahedron, 4))
    , m_te_handle(register_attribute<long>("m_te", PrimitiveType::Tetrahedron, 6))
    , m_tf_handle(register_attribute<long>("m_tf", PrimitiveType::Tetrahedron, 4))
    , m_tt_handle(register_attribute<long>("m_tt", PrimitiveType::Tetrahedron, 4))
{}
std::vector<Tuple> TetMesh::get_vertices() const
{
    throw "not implemented";
}
std::vector<Tuple> TetMesh::get_edges() const
{
    throw "not implemented";
}
std::vector<Tuple> TetMesh::get_faces() const
{
    throw "not implemented";
}
std::vector<Tuple> TetMesh::get_tetrahedrons() const
{
    throw "not implemented";
}

std::vector<Tuple> TetMesh::get_all(const PrimitiveType& type) const
{
    switch (type) {
    case PrimitiveType::Vertex: return get_vertices();
    case PrimitiveType::Edge: return get_edges(); break;
    case PrimitiveType::Face: return get_faces(); break;
    case PrimitiveType::Tetrahedron: return get_tetrahedrons(); break;
    default: throw std::runtime_error("Invalid primitive type");
    }
}
long TetMesh::id(const Tuple& tuple, const PrimitiveType& type) const
{
    return 0;
}

Tuple TetMesh::switch_tuple(const Tuple& tuple, const PrimitiveType& type) const
{
    return Tuple(0, 0, 0, 0, 0);
}

bool TetMesh::is_ccw(const Tuple& tuple) const
{
    return false;
}
} // namespace wmtk
