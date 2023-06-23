#include "Mesh.hpp"

namespace wmtk {

Tuple Mesh::tuple_from_cell(long cid) const
{
    return Tuple(0, 0, 0, cid, 0);
    // TODO: figure out how to compute hash
    // return Tuple(0,0,0,cid, hash(cid));
}

TriMesh::TriMesh()
    : m_vf_accessor(register_attribute_with_accessor<long>("m_vf", PrimitiveType::Vertex, 1))
    , m_ef_accessor(register_attribute_with_accessor<long>("m_ef", PrimitiveType::Edge, 1))
    , m_fv_accessor(register_attribute_with_accessor<long>("m_fv", PrimitiveType::Face, 3))
    , m_fe_accessor(register_attribute_with_accessor<long>("m_fe", PrimitiveType::Face, 3))
    , m_ff_accessor(register_attribute_with_accessor<long>("m_ff", PrimitiveType::Face, 3))
{}


TetMesh::TetMesh()
    : m_vt_accessor(register_attribute_with_accessor<long>("m_vt", PrimitiveType::Vertex, 1))
    , m_et_accessor(register_attribute_with_accessor<long>("m_et", PrimitiveType::Edge, 1))
    , m_ft_accessor(register_attribute_with_accessor<long>("m_ft", PrimitiveType::Face, 1))
    , m_tv_accessor(register_attribute_with_accessor<long>("m_tv", PrimitiveType::Tetrahedron, 4))
    , m_te_accessor(register_attribute_with_accessor<long>("m_te", PrimitiveType::Tetrahedron, 6))
    , m_tf_accessor(register_attribute_with_accessor<long>("m_tf", PrimitiveType::Tetrahedron, 4))
    , m_tt_accessor(register_attribute_with_accessor<long>("m_tt", PrimitiveType::Tetrahedron, 4))
{}

// TODO
bool Mesh::is_valid(const Tuple& tuple) const
{
    // condition 1: global cid stays in bound, and is not removed

    // condition 2: hash


    // Condition 3: local ids are consistent
    const int v = tuple.m_local_vid;
    switch (tuple.m_local_eid) {
    case 0:
        if (tuple.m_local_vid == 1 || tuple.m_local_vid == 2)
            return true;
        else
            return false;
    case 1:
        if (tuple.m_local_vid == 0 || tuple.m_local_vid == 2)
            return true;
        else
            return false;
    case 2:
        if (tuple.m_local_vid == 1 || tuple.m_local_vid == 0)
            return true;
        else
            return false;
    default: throw std::runtime_error("tuple invlid failed local ids check");
    }
}
} // namespace wmtk

std::vector<Tuple> Mesh::get_all(const PrimitiveType& type) const
{
    switch (type) {
    case PrimitiveType::Vertex: return {
        }
    case PrimitiveType::Edge: return get_edges(); break;
    case PrimitiveType::Face: return get_faces(); break;
    case PrimitiveType::Tetrahedron: return get_tetrahedrons(); break;
    default: throw std::runtime_error("Invalid primitive type");
    }
}
} // namespace wmtk
