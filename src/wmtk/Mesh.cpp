#include "Mesh.hpp"

namespace wmtk {

TriMesh::TriMesh()
{
    m_vf_accessor = register_attribute_with_accessor<long>("m_vf", PrimitiveType::Vertex, 1);
    m_ef_accessor = register_attribute_with_accessor<long>("m_ef", PrimitiveType::Edge, 1);

    m_fv_accessor = register_attribute_with_accessor<long>("m_fv", PrimitiveType::Face, 3);
    m_fe_accessor = register_attribute_with_accessor<long>("m_fe", PrimitiveType::Face, 3);
    m_ff_accessor = register_attribute_with_accessor<long>("m_ff", PrimitiveType::Face, 3);
}


TetMesh::TetMesh()
{
    m_vt_accessor = register_attribute_with_accessor<long>("m_vt", PrimitiveType::Vertex, 1);
    m_et_accessor = register_attribute_with_accessor<long>("m_et", PrimitiveType::Edge, 1);
    m_ft_accessor = register_attribute_with_accessor<long>("m_ft", PrimitiveType::Face, 1);

    m_tv_accessor = register_attribute_with_accessor<long>("m_tv", PrimitiveType::Tetrahedron, 4);
    m_te_accessor = register_attribute_with_accessor<long>("m_te", PrimitiveType::Tetrahedron, 6);
    m_tf_accessor = register_attribute_with_accessor<long>("m_tf", PrimitiveType::Tetrahedron, 4);
    m_tt_accessor = register_attribute_with_accessor<long>("m_tt", PrimitiveType::Tetrahedron, 4);
}

} // namespace wmtk
