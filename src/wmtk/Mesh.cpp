#include "Mesh.hpp"
#include "Primitive.hpp"

namespace wmtk {

Mesh::Mesh() = default;

Mesh::~Mesh() = default;

template <typename T>
MeshAttributeHandle<T>
Mesh::register_attribute(const std::string& name, PrimitiveType ptype, long size)
{
    // return MeshAttributeHandle<T>{
    //    .m_base_handle = get_mesh_attributes<T>(ptype).register_attribute(name, size),
    //    .m_primitive_type = ptype};

    MeshAttributeHandle<T> r;
    r.m_base_handle = get_mesh_attributes<T>(ptype).register_attribute(name, size),
    r.m_primitive_type = ptype;
    return r;
}

long Mesh::capacity(PrimitiveType type) const
{
    switch (type) {
    case PrimitiveType::Vertex: return m_capacities[0]; break;
    case PrimitiveType::Edge: return m_capacities[1]; break;
    case PrimitiveType::Face: return m_capacities[2]; break;
    case PrimitiveType::Tetrahedron: {
        if (m_capacities.size() < 4)
            throw std::runtime_error("TetMesh not initialized");
        else
            return m_capacities[3];
        break;
    }
    default: throw std::runtime_error("Invalid primitive type");
    }
}

void Mesh::mesh_attributes_reserve(const PrimitiveType& top_d, long capacity)
{
    throw "not implemeted";
    // for (auto& d : PrimitiveType) {
    //     if (top_d == PrimitiveType::Face && d == PrimitiveType::Tetrahedron) continue;
    //     m_char_attributes[get_simplex_dimension(d)].reserve(capacity);
    //     m_long_attributes[get_simplex_dimension(d)].reserve(capacity);
    //     m_double_attributes[get_simplex_dimension(d)].reserve(capacity);
    //     // m_rational_attributes[get_simplex_dimension(d)].reserve(capacity);
    // }
}

TetMesh::TetMesh()
    : m_vt_handle(register_attribute<long>("m_vt", PrimitiveType::Vertex, 1))
    , m_et_handle(register_attribute<long>("m_et", PrimitiveType::Edge, 1))
    , m_ft_handle(register_attribute<long>("m_ft", PrimitiveType::Face, 1))
    , m_tv_handle(register_attribute<long>("m_tv", PrimitiveType::Tetrahedron, 4))
    , m_te_handle(register_attribute<long>("m_te", PrimitiveType::Tetrahedron, 6))
    , m_tf_handle(register_attribute<long>("m_tf", PrimitiveType::Tetrahedron, 4))
    , m_tt_handle(register_attribute<long>("m_tt", PrimitiveType::Tetrahedron, 4))
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

std::vector<Tuple> Mesh::get_all(const PrimitiveType& type) const
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

void tetmesh_topology_initialization(
    Eigen::Ref<const Mesh::RowVectors3d> V,
    Eigen::Ref<const Mesh::RowVectors4l> F,
    TetMesh& mesh)
{}
} // namespace wmtk
