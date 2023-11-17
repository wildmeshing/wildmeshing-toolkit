#include "TupleTag.hpp"
#include <wmtk/Accessor.hpp>
#include <wmtk/Primitive.hpp>
namespace wmtk::multimesh::utils::internal {
TupleTag::TupleTag(Mesh& mesh)
    : m_mesh(mesh)
    , m_vertex_tag_handle(
          mesh.register_attribute<long>("vertex_tag_handle", PrimitiveType::Vertex, 1))
    , m_edge_tag_handle(mesh.register_attribute<long>("edge_tag_handle", PrimitiveType::Edge, 1))
{}


bool TupleTag::critical_point(const std::set<long>& critical_points, const Tuple& v) const
{
    return critical_points.find(vid(v)) != critical_points.end();
}

long TupleTag::get_vertex_tag(const Tuple& tuple) const
{
    ConstAccessor<long> tag_acc = mesh().create_const_accessor<long>(vertex_tag_handle());
    return tag_acc.const_scalar_attribute(tuple);
}
long TupleTag::get_edge_tag(const Tuple& tuple) const
{
    ConstAccessor<long> tag_acc = mesh().create_const_accessor<long>(edge_tag_handle());
    return tag_acc.const_scalar_attribute(tuple);
}

void TupleTag::set_vertex_tag(const Tuple& tuple, long tag)
{
    Accessor<long> tag_acc = mesh().create_accessor<long>(vertex_tag_handle());
    tag_acc.scalar_attribute(tuple) = tag;
}

void TupleTag::set_edge_tag(const Tuple& tuple, long tag)
{
    Accessor<long> tag_acc = mesh().create_accessor<long>(edge_tag_handle());
    tag_acc.scalar_attribute(tuple) = tag;
}

long TupleTag::vid(const Tuple& tuple) const
{
    return mesh().id(tuple, PrimitiveType::Vertex);
}

Tuple TupleTag::v_tuple(long vid) const
{
    Tuple v_tuple = mesh().tuple_from_id(PrimitiveType::Vertex, vid);
    return v_tuple;
}

bool TupleTag::is_root(const Tuple& v) const
{
    return get_vertex_tag(v) == vid(v);
}


long TupleTag::get_root(const Tuple& v) const
{
    Tuple mutable_v = v;
    while (!is_root(mutable_v)) {
        long parent_vid = get_vertex_tag(mutable_v);
        mutable_v = v_tuple(parent_vid);
    }
    return get_vertex_tag(mutable_v);
}

void TupleTag::set_root(const Tuple& v, long root)
{
    Tuple mutable_v = v;
    while (!is_root(mutable_v)) {
        int tmp = get_vertex_tag(mutable_v);
        set_vertex_tag(mutable_v, root);
        mutable_v = v_tuple(tmp);
    }
    set_vertex_tag(mutable_v, root);
}

void TupleTag::sets_union(const std::set<long>& critical_points, const Tuple& v1, const Tuple& v2)
{
    long v1_root = get_root(v1);
    long v2_root = get_root(v2);
    if (v1_root == v2_root) {
        return;
    }
    if (critical_point(critical_points, v1) || critical_point(critical_points, v2)) {
        return;
    } else {
        long root = std::min(v1_root, v2_root);
        set_root(v1, root);
        set_root(v2, root);
    }
}

void TupleTag::union_find(const std::set<long>& critical_points, const Tuple& e)
{
    Tuple v1 = e;
    Tuple v2 = mesh().switch_vertex(e);
    long vid1 = vid(v1);
    long vid2 = vid(v2);
    // both vertices are critical points
    if (critical_point(critical_points, v1) && critical_point(critical_points, v2)) {
        return;
    } else {
        sets_union(critical_points, v1, v2);
    }
}

} // namespace wmtk::multimesh::utils::internal