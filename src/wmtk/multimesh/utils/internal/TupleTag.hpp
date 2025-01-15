#pragma once
#include <set>
#include <wmtk/Mesh.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/Attribute.hpp>
namespace wmtk::multimesh::utils::internal {
/**
 * @brief TupleTag is a util helper class for tagging facets in a triangle mesh. These tags can then
 * be used to extract facet meshes as child mesh for multimesh.
 *
 * The tagging algorithm is based on the union-find algorithm. It first tags all vertices by
 * grouping them into disjoint sets. Then it creates facet tags based on the boundary tags. Vertex
 * tag and facet tags are stored as mesh attributes of the parent triangle mesh, and can be accessed
 * through "boundary_tag_handle" and "facet_tag_handle" respectively.
 */
class TupleTag
{
public:
    TupleTag(Mesh& mesh, const std::set<int64_t>& critical_boundaries)
        : TupleTag(mesh, PrimitiveType::Edge, critical_boundaries)
    {}

    TupleTag(Mesh& mesh, PrimitiveType facet_type, const std::set<int64_t>& critical_boundaries);
    const Mesh& mesh() const { return m_mesh; }
    Mesh& mesh() { return m_mesh; }
    /**
     * @brief Go through facets of the parent mesh (triangle mesh) and initialize all the boundary
     * tags to be -1.
     */
    void initialize();
    std::set<int64_t> run();
    /**
     * @brief Check if a boundary is a critical point.
     *
     * @param critical_boundaries
     * @param v The boundary tuple
     * @return true if v is a critical point
     * @return false if v is not a critival point
     */
    bool is_critical_boundary(const Tuple& v) const;

    bool boundary_is_root(const Tuple& v) const;
    int64_t boundary_get_root(const Tuple& v) const;
    void boundary_set_root(const Tuple& v, int64_t root);

    /**
     * @brief Given two boundary tuple, join the sets that contain them.
     *
     * @param v1
     * @param v2
     */
    void boundary_sets_unify(const Tuple& v1, const Tuple& v2);
    /**
     * @brief Given an facet, join the sets that contains the two end vertices.
     *
     * @param critical_vids The sets of vertices that are the intersections of facets belong to
     * different sets (for example T junction vertices for an facet mesh)
     * @param e
     */
    void run(const Tuple& e);

    int64_t get_boundary_tag(const Tuple& tuple) const;
    int64_t get_facet_tag(const Tuple& tuple) const;
    void set_boundary_tag(const Tuple& tuple, int64_t tag);
    void set_facet_tag(const Tuple& tuple, int64_t tag);
    int64_t vid(const Tuple& tuple) const;
    Tuple v_tuple(int64_t vid) const;

    Mesh& m_mesh;
    std::set<int64_t> m_critical_boundaries;
    wmtk::attribute::Accessor<int64_t, Mesh, 1> m_boundary_tag_acc;
    wmtk::attribute::Accessor<int64_t, Mesh, 1> m_facet_tag_acc;

    PrimitiveType facet_type() const;
    PrimitiveType boundary_type() const;


    // this is shoved in this class to reduce the number of times we have to friend mesh - this is
    // called by wmtk::multimesh::utils::extract_child_mesh_from_tag
    template <typename T>
    static std::shared_ptr<Mesh> extract_and_register_child_mesh_from_tag_handle(
        Mesh& m,
        const wmtk::attribute::TypedAttributeHandle<T>& tag_handle,
        const T& tag_value,
        bool child_is_free);
};
} // namespace wmtk::multimesh::utils::internal
