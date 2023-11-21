#pragma once
#include <set>
#include <wmtk/Mesh.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/Attribute.hpp>
namespace wmtk::multimesh::utils::internal {
/**
 * @brief TupleTag is a util helper class for tagging edges in a triangle mesh. These tags can then
 * be used to extract edge meshes as child mesh for multimesh.
 *
 * The tagging algorithm is based on the union-find algorithm. It first tags all vertices by
 * grouping them into disjoint sets. Then it creates edge tags based on the vertex tags. Vertex tag
 * and edge tags are stored as mesh attributes of the parent triangle mesh, and can be accessed
 * through "vertex_tag_handle" and "edge_tag_handle" respectively.
 */
class TupleTag
{
public:
    TupleTag(Mesh& mesh, const std::set<long>& critical_points);
    const Mesh& mesh() const { return m_mesh; }
    Mesh& mesh() { return m_mesh; }
    /**
     * @brief Go through edges of the parent mesh (triangle mesh) and initialize all the vertex tags
     * to be -1.
     */
    void initialize();
    std::set<long> run();
    /**
     * @brief Check if a vertex is a critical point.
     *
     * @param critical_points
     * @param v The vertex tuple
     * @return true if v is a critical point
     * @return false if v is not a critival point
     */
    bool is_critical_vertex(const Tuple& v) const;

    bool vertex_is_root(const Tuple& v) const;
    long vertex_get_root(const Tuple& v) const;
    void vertex_set_root(const Tuple& v, long root);

    /**
     * @brief Given two vertex tuple, join the sets that contain them.
     *
     * @param v1
     * @param v2
     */
    void vertex_sets_unify(const Tuple& v1, const Tuple& v2);
    /**
     * @brief Given an edge, join the sets that contains the two end vertices.
     *
     * @param critical_vids The sets of vertices that are the intersections of edges belong to
     * different sets (for example T junction vertices for an edge mesh)
     * @param e
     */
    void run(const Tuple& e);

    long get_vertex_tag(const Tuple& tuple) const;
    long get_edge_tag(const Tuple& tuple) const;
    void set_vertex_tag(const Tuple& tuple, long tag);
    void set_edge_tag(const Tuple& tuple, long tag);
    long vid(const Tuple& tuple) const;
    Tuple v_tuple(long vid) const;

    Mesh& m_mesh;
    std::set<long> m_critical_points;
    Accessor<long> m_vertex_tag_acc;
    Accessor<long> m_edge_tag_acc;
};
} // namespace wmtk::multimesh::utils::internal
