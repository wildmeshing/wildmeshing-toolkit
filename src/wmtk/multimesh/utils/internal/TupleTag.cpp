#include "TupleTag.hpp"
#include <wmtk/Primitive.hpp>
#include <wmtk/attribute/Accessor.hpp>
namespace wmtk::multimesh::utils::internal {
TupleTag::TupleTag(
    Mesh& mesh,
    PrimitiveType facet_type,
    const std::set<int64_t>& critical_boundaries)
    : m_mesh(mesh)
    , m_critical_boundaries(critical_boundaries)
    , m_boundary_tag_acc(mesh.create_accessor<int64_t, 1>(
          mesh.register_attribute_typed<int64_t>("boundary_tag", facet_type - 1, 1)))
    , m_facet_tag_acc(mesh.create_accessor<int64_t, 1>(
          mesh.register_attribute_typed<int64_t>("facet_tag", facet_type, 1)))
{
    initialize();
}
PrimitiveType TupleTag::facet_type() const
{
    return m_facet_tag_acc.primitive_type();
}
PrimitiveType TupleTag::boundary_type() const
{
    return m_boundary_tag_acc.primitive_type();
}

void TupleTag::initialize()
{
    std::vector<Tuple> v_tuples = mesh().get_all(PrimitiveType::Vertex);
    std::vector<Tuple> e_tuples = mesh().get_all(PrimitiveType::Edge);
    // initializing all the facet tags to be -1
    for (const Tuple& e : e_tuples) {
        if (mesh().is_boundary(PrimitiveType::Edge, e)) {
            set_facet_tag(e, -1);
        }
    }
    // initializing all the boundary tags to be the boundary id
    for (const Tuple& v : v_tuples) {
        if (mesh().is_boundary(PrimitiveType::Vertex, v)) {
            set_boundary_tag(v, vid(v));
        }
    }
}

std::set<int64_t> TupleTag::run()
{
    std::set<int64_t> tags;
    std::vector<Tuple> v_tuples = mesh().get_all(PrimitiveType::Vertex);
    std::vector<Tuple> e_tuples = mesh().get_all(PrimitiveType::Edge);
    int64_t vid_max = mesh().capacity(PrimitiveType::Vertex);
    // the pass to tag all vertices
    for (const Tuple& e : e_tuples) {
        if (mesh().is_boundary(PrimitiveType::Edge, e)) {
            run(e);
        }
    }
    // the pass to tag all facets
    int64_t facet_tag = 0;
    for (const Tuple& e : e_tuples) {
        if (mesh().is_boundary(PrimitiveType::Edge, e)) {
            Tuple v1 = e;
            Tuple v2 = mesh().switch_tuple(e, PrimitiveType::Vertex);
            // both vertices are critical points
            if (is_critical_boundary(v1) && is_critical_boundary(v2)) {
                set_facet_tag(e, facet_tag + vid_max);
                tags.insert(facet_tag + vid_max);
                facet_tag++;
            } else if (is_critical_boundary(v1)) {
                int64_t v2_root = boundary_get_root(v2);
                set_facet_tag(e, v2_root);
                tags.insert(v2_root);
            } else if (is_critical_boundary(v2)) {
                int64_t v1_root = boundary_get_root(v1);
                set_facet_tag(e, v1_root);
                tags.insert(v1_root);
            } else {
                int64_t v1_root = boundary_get_root(v1);
                int64_t v2_root = boundary_get_root(v2);
                assert(v1_root == v2_root);
                set_facet_tag(e, v1_root);
                tags.insert(v1_root);
            }
        }
    }
    return tags;
}


bool TupleTag::is_critical_boundary(const Tuple& v) const
{
    return m_critical_boundaries.find(vid(v)) != m_critical_boundaries.end();
}

int64_t TupleTag::get_boundary_tag(const Tuple& tuple) const
{
    return m_boundary_tag_acc.const_scalar_attribute(tuple);
}
int64_t TupleTag::get_facet_tag(const Tuple& tuple) const
{
    return m_facet_tag_acc.const_scalar_attribute(tuple);
}

void TupleTag::set_boundary_tag(const Tuple& tuple, int64_t tag)
{
    m_boundary_tag_acc.scalar_attribute(tuple) = tag;
}

void TupleTag::set_facet_tag(const Tuple& tuple, int64_t tag)
{
    m_facet_tag_acc.scalar_attribute(tuple) = tag;
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

bool TupleTag::boundary_is_root(const Tuple& v) const
{
    return get_boundary_tag(v) == vid(v);
}


int64_t TupleTag::boundary_get_root(const Tuple& v) const
{
    Tuple mutable_v = v;
    while (!boundary_is_root(mutable_v)) {
        int64_t parent_vid = get_boundary_tag(mutable_v);
        mutable_v = v_tuple(parent_vid);
    }
    return get_boundary_tag(mutable_v);
}

void TupleTag::boundary_set_root(const Tuple& v, int64_t root)
{
    Tuple mutable_v = v;
    while (!boundary_is_root(mutable_v)) {
        int tmp = get_boundary_tag(mutable_v);
        set_boundary_tag(mutable_v, root);
        mutable_v = v_tuple(tmp);
    }
    set_boundary_tag(mutable_v, root);
}

void TupleTag::boundary_sets_unify(const Tuple& v1, const Tuple& v2)
{
    int64_t v1_root = boundary_get_root(v1);
    int64_t v2_root = boundary_get_root(v2);
    if (v1_root == v2_root) {
        return;
    }
    if (is_critical_boundary(v1) || is_critical_boundary(v2)) {
        return;
    } else {
        int64_t root = std::min(v1_root, v2_root);
        boundary_set_root(v1, root);
        boundary_set_root(v2, root);
    }
}

void TupleTag::run(const Tuple& e)
{
    Tuple v1 = e;
    Tuple v2 = mesh().switch_tuple(e, PrimitiveType::Vertex);
    int64_t vid1 = vid(v1);
    int64_t vid2 = vid(v2);
    // both vertices are critical points
    if (is_critical_boundary(v1) && is_critical_boundary(v2)) {
        return;
    } else {
        boundary_sets_unify(v1, v2);
    }
}

} // namespace wmtk::multimesh::utils::internal
