#include "create_tag.hpp"
namespace wmtk::multimesh::utils {
internal::TupleTag initialize_tags(Mesh& m, const std::set<long>& critical_vids)
{
    internal::TupleTag tuple_tag(m);
    std::vector<Tuple> v_tuples = m.get_all(PrimitiveType::Vertex);
    std::vector<Tuple> e_tuples = m.get_all(PrimitiveType::Edge);
    for (const Tuple& e : e_tuples) {
        if (tuple_tag.mesh().is_boundary(e, PrimitiveType::Edge)) {
            tuple_tag.set_vertex_tag(e, -1);
            tuple_tag.set_vertex_tag(m.switch_vertex(e), -1);
            tuple_tag.set_edge_tag(e, -1);
        }
    }
    return tuple_tag;
}
void create_tags(internal::TupleTag& tuple_tag, const std::set<long>& critical_vids)
{
    std::vector<Tuple> v_tuples = tuple_tag.mesh().get_all(PrimitiveType::Vertex);
    std::vector<Tuple> e_tuples = tuple_tag.mesh().get_all(PrimitiveType::Edge);
    long vid_max = tuple_tag.mesh().capacity(PrimitiveType::Vertex);
    // the pass to tag all vertices
    for (const Tuple& e : e_tuples) {
        if (tuple_tag.mesh().is_boundary(e, PrimitiveType::Edge)) {
            tuple_tag.union_find(critical_vids, e);
        }
    }
    long edge_tag = 0;
    for (const Tuple& e : e_tuples) {
        if (tuple_tag.mesh().is_boundary(e, PrimitiveType::Edge)) {
            Tuple v1 = e;
            Tuple v2 = tuple_tag.mesh().switch_vertex(e);
            // both vertices are critical points
            if (tuple_tag.critical_point(critical_vids, v1) &&
                tuple_tag.critical_point(critical_vids, v2)) {
                tuple_tag.set_edge_tag(e, edge_tag + vid_max);
                edge_tag++;
            } else if (tuple_tag.get_root(v1) == -1) {
                long v2_root = tuple_tag.get_root(v2);
                tuple_tag.set_edge_tag(e, v2_root);
            } else {
                long v1_root = tuple_tag.get_root(v1);
                tuple_tag.set_edge_tag(e, v1_root);
            }
        }
    }
}
} // namespace wmtk::multimesh::utils
