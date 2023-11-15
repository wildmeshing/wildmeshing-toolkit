#pragma once
#include <set>
#include <wmtk/Mesh.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/Attribute.hpp>
namespace wmtk::multimesh::utils::internal {

class TupleTag
{
    Mesh& m_mesh;
    MeshAttributeHandle<long> m_vertex_tag_handle;
    MeshAttributeHandle<long> m_edge_tag_handle;

public:
    // constrcutor
    TupleTag(Mesh& mesh);
    const Mesh& mesh() const { return m_mesh; }
    Mesh& mesh() { return m_mesh; }
    const MeshAttributeHandle<long>& vertex_tag_handle() const { return m_vertex_tag_handle; }
    const MeshAttributeHandle<long>& edge_tag_handle() const { return m_edge_tag_handle; }

    bool is_root(const Tuple& v) const;
    long get_root(const Tuple& v) const;
    void set_root(const Tuple& v, long root);

    void set_union(const Tuple& v1, const Tuple& v2);
    void union_find(const std::set<long>& critical_vids, const Tuple& v1, const Tuple& v2);

private:
    long get_vertex_tag(const Tuple& tuple) const;
    long get_edge_tag(const Tuple& tuple) const;
    void set_vertex_tag(const Tuple& tuple, long tag);
    void set_edge_tag(const Tuple& tuple, long tag);
    long vid(const Tuple& tuple) const;
    Tuple v_tuple(long vid) const;
};
} // namespace wmtk::multimesh::utils::internal