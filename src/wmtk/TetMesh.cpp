#include "TetMesh.hpp"

#include <wmtk/utils/tetmesh_topology_initialization.h>
#include <wmtk/utils/Logger.hpp>

namespace wmtk {
TetMesh::TetMesh()

    : Mesh(4)
    , m_vt_handle(register_attribute<long>("m_vt", PrimitiveType::Vertex, 1))
    , m_et_handle(register_attribute<long>("m_et", PrimitiveType::Edge, 1))
    , m_ft_handle(register_attribute<long>("m_ft", PrimitiveType::Face, 1))
    , m_tv_handle(register_attribute<long>("m_tv", PrimitiveType::Tetrahedron, 4))
    , m_te_handle(register_attribute<long>("m_te", PrimitiveType::Tetrahedron, 6))
    , m_tf_handle(register_attribute<long>("m_tf", PrimitiveType::Tetrahedron, 4))
    , m_tt_handle(register_attribute<long>("m_tt", PrimitiveType::Tetrahedron, 4))
{}

Tuple TetMesh::vertex_tuple_from_id() const
{
    throw "not implemented";
}

Tuple TetMesh::tuple_from_id(PrimitiveType ptype, long id) const
{
    throw "not implemented";
}


void TetMesh::initialize(
    Eigen::Ref<const RowVectors4l> TV,
    Eigen::Ref<const RowVectors6l> TE,
    Eigen::Ref<const RowVectors4l> TF,
    Eigen::Ref<const RowVectors4l> TT,
    Eigen::Ref<const VectorXl> VT,
    Eigen::Ref<const VectorXl> ET,
    Eigen::Ref<const VectorXl> FT)

{
    // reserve memory for attributes

    std::vector<long> cap{
        static_cast<long>(VT.rows()),
        static_cast<long>(ET.rows()),
        static_cast<long>(FT.rows()),
        static_cast<long>(TT.rows())};
    set_capacities(cap);
    reserve_attributes_to_fit();

    // get Accessors for topology
    Accessor<long> vt_accessor = create_accessor<long>(m_vt_handle);
    Accessor<long> et_accessor = create_accessor<long>(m_et_handle);
    Accessor<long> ft_accessor = create_accessor<long>(m_ft_handle);

    Accessor<long> tv_accessor = create_accessor<long>(m_tv_handle);
    Accessor<long> te_accessor = create_accessor<long>(m_te_handle);
    Accessor<long> tf_accessor = create_accessor<long>(m_tf_handle);
    Accessor<long> tt_accessor = create_accessor<long>(m_tt_handle);

    // iterate over the matrices and fill attributes
    for (long i = 0; i < capacity(PrimitiveType::Tetrahedron); ++i) {
        tv_accessor.vector_attribute(i) = TV.row(i).transpose();
        te_accessor.vector_attribute(i) = TE.row(i).transpose();
        tf_accessor.vector_attribute(i) = TF.row(i).transpose();
        tt_accessor.vector_attribute(i) = TT.row(i).transpose();
    }
    // m_vt
    for (long i = 0; i < capacity(PrimitiveType::Vertex); ++i) {
        vt_accessor.scalar_attribute(i) = VT(i);
    }
    // m_et
    for (long i = 0; i < capacity(PrimitiveType::Edge); ++i) {
        et_accessor.scalar_attribute(i) = ET(i);
    }
    // m_ft
    for (long i = 0; i < capacity(PrimitiveType::Face); ++i) {
        ft_accessor.scalar_attribute(i) = FT(i);
    }
}

Tuple TetMesh::edge_tuple_from_id(long id) const
{
    throw "not implemented";
}


void TetMesh::initialize(Eigen::Ref<const RowVectors4l> T)
{
    auto [TE, TF, TT, VT, ET, FT] = tetmesh_topology_initialization(T);
    initialize(T, TE, TF, TT, VT, ET, FT);
}

long TetMesh::_debug_id(const Tuple& tuple, const PrimitiveType& type) const
{
    // do not remove this warning!
    wmtk::logger().warn("This function must only be used for debugging!!");
    return id(tuple, type);
}

Tuple TetMesh::face_tuple_from_id(long id) const
{
    throw "not implemented";
}


void TetMesh::split_edge(const Tuple& t)
{
    throw "not implemented";
}

void TetMesh::collapse_edge(const Tuple& t)
{
    throw "not implemented";
}


std::vector<Tuple> TetMesh::get_all(const PrimitiveType& type) const
{
    throw "not implemented";

    // switch (type) {
    // case PrimitiveType::Vertex: return get_vertices();
    // case PrimitiveType::Edge: return get_edges(); break;
    // case PrimitiveType::Face: return get_faces(); break;
    // case PrimitiveType::Tetrahedron: return get_tetrahedrons(); break;
    // default: throw std::runtime_error("Invalid primitive type");
    // }
}

long TetMesh::id(const Tuple& tuple, const PrimitiveType& type) const
{
    throw "not implemented";
}

Tuple TetMesh::switch_tuple(const Tuple& tuple, const PrimitiveType& type) const
{
    throw "not implemented";
}

bool TetMesh::is_ccw(const Tuple& tuple) const
{
    throw "not implemented";
}

bool TetMesh::is_valid(const Tuple& tuple) const
{
    throw "not implemented";
}
bool TetMesh::is_boundary(const Tuple& tuple) const
{
    throw "not implemented";
    // ConstAccessor<long> tt_accessor = create_accessor<long>(m_tt_handle);
    // return tt_accessor.scalar_attribute(tuple) < 0;
}
} // namespace wmtk
