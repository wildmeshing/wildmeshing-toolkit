#include "TupleTag.hpp"
#include <wmtk/Accessor.hpp>
#include <wmtk/Primitive.hpp>
namespace wmtk::multimesh::utils::internal {
TupleTag::TupleTag(Mesh& mesh, const std::set<int64_t>& critical_points)
    : m_mesh(mesh)
    , m_critical_points(critical_points)
    , m_vertex_tag_acc(mesh.create_accessor(
          mesh.register_attribute<int64_t>("vertex_tag", PrimitiveType::Vertex, 1)))
    , m_edge_tag_acc(mesh.create_accessor(
          mesh.register_attribute<int64_t>("edge_tag", PrimitiveType::Edge, 1)))
{
    initialize();
}
TupleTag::TupleTag(Mesh& mesh, const std::set<Tuple>& critical_points)
    : m_mesh(mesh)
    , m_vertex_tag_acc(mesh.create_accessor(
          mesh.register_attribute<long>("vertex_tag", PrimitiveType::Vertex, 1)))
    , m_edge_tag_acc(
          mesh.create_accessor(mesh.register_attribute<long>("edge_tag", PrimitiveType::Edge, 1)))
{
    critical_vertex_tuples_to_vids(critical_points);
    initialize();
}


void TupleTag::initialize()
{
    std::vector<Tuple> v_tuples = mesh().get_all(PrimitiveType::Vertex);
    std::vector<Tuple> e_tuples = mesh().get_all(PrimitiveType::Edge);
    // initializing all the edge tags to be -1
    for (const Tuple& e : e_tuples) {
        if (mesh().is_boundary(e, PrimitiveType::Edge)) {
            set_edge_tag(e, -1);
        }
    }
    // initializing all the vertex tags to be the vertex id
    for (const Tuple& v : v_tuples) {
        if (mesh().is_boundary(v, PrimitiveType::Vertex)) {
            set_vertex_tag(v, vid(v));
        }
    }
}

void TupleTag::critical_vertex_tuples_to_vids(const std::set<Tuple>& critical_vertex_tuples)
{
    for (const Tuple& v : critical_vertex_tuples) {
        assert(mesh().is_valid_slow(v));
        m_critical_points.insert(mesh().id(v, PrimitiveType::Vertex));
    }
    // assuming the critical Tuples are distinct
    assert(m_critical_points.size() == critical_vertex_tuples.size());
}

std::set<int64_t> TupleTag::run()
{
    std::set<int64_t> tags;
    std::vector<Tuple> v_tuples = mesh().get_all(PrimitiveType::Vertex);
    std::vector<Tuple> e_tuples = mesh().get_all(PrimitiveType::Edge);
    int64_t vid_max = mesh().capacity(PrimitiveType::Vertex);
    // the pass to tag all vertices
    for (const Tuple& e : e_tuples) {
        if (mesh().is_boundary(e, PrimitiveType::Edge)) {
            run(e);
        }
    }
    // the pass to tag all edges
    int64_t edge_tag = 0;
    for (const Tuple& e : e_tuples) {
        if (mesh().is_boundary(e, PrimitiveType::Edge)) {
            Tuple v1 = e;
            Tuple v2 = mesh().switch_vertex(e);
            // both vertices are critical points
            if (is_critical_vertex(v1) && is_critical_vertex(v2)) {
                set_edge_tag(e, edge_tag + vid_max);
                tags.insert(edge_tag + vid_max);
                edge_tag++;
            } else if (is_critical_vertex(v1)) {
                int64_t v2_root = vertex_get_root(v2);
                set_edge_tag(e, v2_root);
                tags.insert(v2_root);
            } else if (is_critical_vertex(v2)) {
                int64_t v1_root = vertex_get_root(v1);
                set_edge_tag(e, v1_root);
                tags.insert(v1_root);
            } else {
                int64_t v1_root = vertex_get_root(v1);
                int64_t v2_root = vertex_get_root(v2);
                assert(v1_root == v2_root);
                set_edge_tag(e, v1_root);
                tags.insert(v1_root);
            }
        }
    }
    return tags;
}


bool TupleTag::is_critical_vertex(const Tuple& v) const
{
    return m_critical_points.find(vid(v)) != m_critical_points.end();
}

int64_t TupleTag::get_vertex_tag(const Tuple& tuple) const
{
    return m_vertex_tag_acc.const_scalar_attribute(tuple);
}
int64_t TupleTag::get_edge_tag(const Tuple& tuple) const
{
    return m_edge_tag_acc.const_scalar_attribute(tuple);
}

void TupleTag::set_vertex_tag(const Tuple& tuple, int64_t tag)
{
    m_vertex_tag_acc.scalar_attribute(tuple) = tag;
}

void TupleTag::set_edge_tag(const Tuple& tuple, int64_t tag)
{
    m_edge_tag_acc.scalar_attribute(tuple) = tag;
}

int64_t TupleTag::vid(const Tuple& tuple) const
{
    return mesh().id(tuple, PrimitiveType::Vertex);
}

Tuple TupleTag::v_tuple(int64_t vid) const
{
    Tuple v_tuple = mesh().tuple_from_id(PrimitiveType::Vertex, vid);
    return v_tuple;
}

bool TupleTag::vertex_is_root(const Tuple& v) const
{
    return get_vertex_tag(v) == vid(v);
}


int64_t TupleTag::vertex_get_root(const Tuple& v) const
{
    Tuple mutable_v = v;
    while (!vertex_is_root(mutable_v)) {
        int64_t parent_vid = get_vertex_tag(mutable_v);
        mutable_v = v_tuple(parent_vid);
    }
    return get_vertex_tag(mutable_v);
}

void TupleTag::vertex_set_root(const Tuple& v, int64_t root)
{
    Tuple mutable_v = v;
    while (!vertex_is_root(mutable_v)) {
        int tmp = get_vertex_tag(mutable_v);
        set_vertex_tag(mutable_v, root);
        mutable_v = v_tuple(tmp);
    }
    set_vertex_tag(mutable_v, root);
}

void TupleTag::vertex_sets_unify(const Tuple& v1, const Tuple& v2)
{
    int64_t v1_root = vertex_get_root(v1);
    int64_t v2_root = vertex_get_root(v2);
    if (v1_root == v2_root) {
        return;
    }
    if (is_critical_vertex(v1) || is_critical_vertex(v2)) {
        return;
    } else {
        int64_t root = std::min(v1_root, v2_root);
        vertex_set_root(v1, root);
        vertex_set_root(v2, root);
    }
}

void TupleTag::run(const Tuple& e)
{
    Tuple v1 = e;
    Tuple v2 = mesh().switch_vertex(e);
    int64_t vid1 = vid(v1);
    int64_t vid2 = vid(v2);
    // both vertices are critical points
    if (is_critical_vertex(v1) && is_critical_vertex(v2)) {
        return;
    } else {
        vertex_sets_unify(v1, v2);
    }
}

} // namespace wmtk::multimesh::utils::internal
