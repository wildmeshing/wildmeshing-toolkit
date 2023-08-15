#include "TetMeshOperationExecutor.hpp"

namespace wmtk {

// constructor
TetMesh::TetMeshOperationExecutor::TetMeshOperationExecutor(
    TetMesh& m,
    const Tuple& operating_tuple)
    : flag_accessors{{m.get_flag_accessor(PrimitiveType::Vertex), m.get_flag_accessor(PrimitiveType::Edge), m.get_flag_accessor(PrimitiveType::Face), m.get_flag_accessor(PrimitiveType::Tetrahedron)}}
    , tt_accessor(m.create_accessor<long>(m.m_tt_handle))
    , tf_accessor(m.create_accessor<long>(m.m_tf_handle))
    , te_accessor(m.create_accessor<long>(m.m_te_handle))
    , tv_accessor(m.create_accessor<long>(m.m_tv_handle))
    , vt_accessor(m.create_accessor<long>(m.m_vt_handle))
    , et_accessor(m.create_accessor<long>(m.m_et_handle))
    , ft_accessor(m.create_accessor<long>(m.m_ft_handle))
    , hash_accessor(m.get_cell_hash_accessor())
    , m_mesh(m)
    , m_operating_tuple(operating_tuple)
{
    // store ids of edge and incident vertices
    m_operating_edge_id = m_mesh.id_edge(m_operating_tuple);
    m_spine_vids[0] = m_mesh.id_vertex(m_operating_tuple);
    m_spine_vids[1] = m_mesh.id_vertex(m_mesh.switch_vertex(m_operating_tuple));

    // to be continued, need to implement switch tet first
}


} // namespace wmtk