#include "Mesh.hpp"

namespace wmtk {

Mesh::Mesh() = default;

Mesh::~Mesh() = default;

TriMesh::TriMesh()
    : m_vf_accessor(register_attribute_with_accessor<long>("m_vf", PrimitiveType::Vertex, 1))
    , m_ef_accessor(register_attribute_with_accessor<long>("m_ef", PrimitiveType::Edge, 1))
    , m_fv_accessor(register_attribute_with_accessor<long>("m_fv", PrimitiveType::Face, 3))
    , m_fe_accessor(register_attribute_with_accessor<long>("m_fe", PrimitiveType::Face, 3))
    , m_ff_accessor(register_attribute_with_accessor<long>("m_ff", PrimitiveType::Face, 3))
{}

void TriMesh::split_edge(const Tuple& t) {}
void TriMesh::collapse_edge(const Tuple& t) {}
void TriMesh::build_vertex_connectivity(long n_vertices) {}
long TriMesh::id(const Tuple& tuple, const PrimitiveType& type) const
{
    return 0;
}

Tuple TriMesh::switch_tuple(const Tuple& tuple, const PrimitiveType& type) const
{ 
    return Tuple(0,0,0,0,0); 
}

bool TriMesh::is_ccw(const Tuple& tuple) const
{
    return false;
}

TetMesh::TetMesh()
    : m_vt_accessor(register_attribute_with_accessor<long>("m_vt", PrimitiveType::Vertex, 1))
    , m_et_accessor(register_attribute_with_accessor<long>("m_et", PrimitiveType::Edge, 1))
    , m_ft_accessor(register_attribute_with_accessor<long>("m_ft", PrimitiveType::Face, 1))
    , m_tv_accessor(register_attribute_with_accessor<long>("m_tv", PrimitiveType::Tetrahedron, 4))
    , m_te_accessor(register_attribute_with_accessor<long>("m_te", PrimitiveType::Tetrahedron, 6))
    , m_tf_accessor(register_attribute_with_accessor<long>("m_tf", PrimitiveType::Tetrahedron, 4))
    , m_tt_accessor(register_attribute_with_accessor<long>("m_tt", PrimitiveType::Tetrahedron, 4))
{}

long TetMesh::id(const Tuple& tuple, const PrimitiveType& type) const
{
    return 0;
}

Tuple TetMesh::switch_tuple(const Tuple& tuple, const PrimitiveType& type) const
{ 
    return Tuple(0,0,0,0,0); 
}

bool TetMesh::is_ccw(const Tuple& tuple) const
{
    return false;
}

} // namespace wmtk
