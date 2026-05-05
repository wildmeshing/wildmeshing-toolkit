#pragma once
#include <wmtk/TriMesh.h>
#include <algorithm>
#include <set>
#include "Parameters.h"
#include "SimplicialComplexBVH.hpp"


namespace wmtk::components::topological_offset {


class VertexAttributes2d
{
public:
    Vector2d m_posf;
    int label = 0;

    VertexAttributes2d() {};
    VertexAttributes2d(const Vector2d& p);
};


class EdgeAttributes2d
{
public:
    int label = 0;
};


class FaceAttributes2d
{
public:
    int label = 0;
    std::set<int64_t> tag;
};


class TopoOffsetTriMesh : public wmtk::TriMesh
{
public: // mode for splitting in marching tets
    enum class EdgeSplitMode {
        Midpoint = 0,
        BinarySearch =
            1 // requires that every edge being split has one vertex labelled 0 and the other 1 or 2
    };

public:
    int m_vtu_counter = 0;
    std::array<size_t, 3> m_init_counts = {{0, 0, 0}};
    size_t m_tags_count;
    SimplicialComplexBVH m_input_complex_bvh;
    EdgeSplitMode m_edge_split_mode = EdgeSplitMode::Midpoint;

    // just for retaining in output. dont actually use
    bool m_has_envelope = false;
    MatrixXd m_V_envelope;
    MatrixXi m_F_envelope;

    Parameters& m_params;

    using VertAttCol = wmtk::AttributeCollection<VertexAttributes2d>;
    using EdgeAttCol = wmtk::AttributeCollection<EdgeAttributes2d>;
    using FaceAttCol = wmtk::AttributeCollection<FaceAttributes2d>;
    VertAttCol m_vertex_attribute;
    EdgeAttCol m_edge_attribute;
    FaceAttCol m_face_attribute;

    TopoOffsetTriMesh(Parameters& _m_params, int _num_threads = 0)
        : m_params(_m_params)
    {
        NUM_THREADS = _num_threads;
        p_vertex_attrs = &m_vertex_attribute;
        p_edge_attrs = &m_edge_attribute;
        p_face_attrs = &m_face_attribute;
    }

    ~TopoOffsetTriMesh() {}

    /**
     * @brief initialize TriMesh from vertex, face, tag data
     * @param V: #V by 2 vertex matrix
     * @param F: #F by 3 face matrix
     * @param F_tags: #F by #physical groups tag matrix
     * @param V_env: #V_env by 2 EnvelopeSurface vertex matrix
     * @param F_env: #F_env by 2 EnvelopeSurface edge matrix
     */
    void init_from_image(
        const MatrixXd& V,
        const MatrixXi& F,
        const MatrixSi& F_tags,
        MatrixXd& V_env,
        MatrixXi& F_env);

    /**
     * @brief check if the input complex is empty. Only valid after calling init_from_image(...).
     * Checks if any vertices (therefore any simplices) are labelled 1, if not returns true
     */
    bool empty_input_complex();

    /**
     * @brief initialize BVH for input complex. Must be called after init_from_image(...)
     */
    void init_input_complex_bvh();

    /**
     * @brief split edge at point by minimizing m_params.target_distance - d() (where d() is
     * distance to input complex via BVH) along the edge. Uses binary search, so implicitly assumes
     * distance field is monotonic along edge. May give weird results if not monotonic
     */
    void edge_split_binary_search(const size_t v1, const size_t v2, Vector2d& p_new) const;

    //// overriden splits/invariants
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& t) override;
    bool split_face_before(const Tuple& t) override;
    bool split_face_after(const Tuple& t) override;
    bool invariants(const std::vector<Tuple>& tris) override;
    //// overriden splits/invariants


    /**
     * @brief execute simplistic marching tets. All edges with one vertex labelled 0 and the other 1/2
     * are split. If m_edge_split_mode=BinarySearch, edges are split according to BVH distance field
     * and the offset target distance (m_params.target_distance). If m_edge_split_mode=Midpoint,
     * edges are split at the midpoint
     */
    void marching_tets();


    //// simplicial embedding stuff
    /**
     * @brief check if the input complex (simplices labelled 1) are simplicially embedded w.r.t. the
     * entire mesh
     */
    bool is_simplicially_embedded() const;

    /**
     * @brief check if a triangle satisfies simpicial embedding criteria w.r.t. input complex
     * (simplices labelled 1)
     */
    bool tri_is_simp_emb(const Tuple& t) const;

    /**
     * @brief make mesh a simplicial embedding of the input complex (simplices labelled 1)
     */
    void simplicial_embedding();
    //// simplicial embedding stuff

    //// variable offset stuff
    /**
     * @brief check if adding a triangle to the offset region does not change the topology of the
     * offset. Returns true if topology would not be changed
     */
    bool tri_consistent_topology(const size_t f_id) const;

    /**
     * @brief check if a triangle is inside the offset (implicitly defined via BVH distance field to
     * input complex) via conservative circle subdivision estimation
     */
    bool tri_is_in_offset_conservative(const size_t f_id, const double threshold_r) const;

    /**
     * @brief grow offset region conservatively using conservative checks while ensuring consistent
     * topology
     */
    void grow_offset_conservative();
    //// variable offset stuff

    /**
     * @brief update 'tags' data for triangles in the offset region (tris labelled 2) based on
     * the given offset tag values in m_params.offset_tag_value
     */
    void set_offset_tri_tags();

    /**
     * @brief verify that the closed offset region (simplices labelled 1 or 2) form a manifold
     * region. This should be true for any offset. This function is for verification
     */
    bool offset_is_manifold();

    //// output stuff
    void write_input_complex(const std::string& path);
    void write_vtu(const std::string& path);
    void write_msh(const std::string& file);
    void write_msh_groups(const std::string& file);
    //// output stuff

private:
    /**
     * @note for all split caches, simplex attributes are inherited from the simplex (of same or
     * higher order) they are 'borne' out of
     */

    struct EdgeSplitCache
    {
        size_t v1_id;
        size_t v2_id;
        VertexAttributes2d new_v;

        // cache edge attributes
        EdgeAttributes2d split_eattr;
        std::map<simplex::Edge, EdgeAttributes2d> existing_eattr;

        // cache face attributes
        std::map<size_t, FaceAttributes2d> opp_v_fattr;
    };
    tbb::enumerable_thread_specific<EdgeSplitCache> edge_split_cache;

    struct FaceSplitCache
    {
        size_t v1_id;
        size_t v2_id;
        size_t v3_id;
        VertexAttributes2d new_v;

        std::map<simplex::Edge, EdgeAttributes2d> existing_eattr; // 3 orig edges
        FaceAttributes2d split_fattr; // split face attributes
    };
    tbb::enumerable_thread_specific<FaceSplitCache> face_split_cache;

private: // helpers
    /**
     * @brief determine if any tag from tag1 is also present in tag2.
     * @note if tag2 is empty (ambient), return true if tag1 is empty, otherwise false (tag2 is
     * ambient, so only 'element' is ambient)
     */
    bool any_tag_present(const std::set<int64_t>& tag1, const std::set<int64_t>& tag2)
    {
        if (tag2.empty()) {
            return tag1.empty();
        }
        if (tag1.empty()) { // tag1 is ambient and tag2 is not
            return false;
        }

        for (const int64_t& i : tag1) {
            if (tag2.find(i) != tag2.end()) {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief sort vector of edge simplices in place by decreasing length
     */
    void sort_edges_by_length(std::vector<simplex::Edge>& edges)
    {
        std::sort(
            edges.begin(),
            edges.end(),
            [this](const simplex::Edge& e1, const simplex::Edge& e2) {
                double len1 = (m_vertex_attribute[e1.vertices()[0]].m_posf -
                               m_vertex_attribute[e1.vertices()[1]].m_posf)
                                  .squaredNorm();
                double len2 = (m_vertex_attribute[e2.vertices()[0]].m_posf -
                               m_vertex_attribute[e2.vertices()[1]].m_posf)
                                  .squaredNorm();
                return len1 > len2;
            });
    }

public: // helpers
    /**
     * @brief get global id of edge from simplex::Edge object
     */
    size_t edge_id_from_simplex(const simplex::Edge& e) const
    {
        const auto& verts = e.vertices();
        const auto incident = simplex_incident_triangles(e);
        const auto& faces = incident.faces();

        assert(!faces.empty()); // throw error here otherwise

        const size_t f_id = tuple_from_simplex(faces.front()).fid(*this);
        const Tuple t_edge = tuple_from_edge(verts[0], verts[1], f_id);
        return t_edge.eid(*this);
    }

    /**
     * @brief get Tuple simplex::Edge object
     */
    Tuple get_tuple_from_edge(const simplex::Edge& e) const
    {
        const auto& v = e.vertices();
        const auto faces = simplex_incident_triangles(e).faces();
        assert(!faces.empty());
        const size_t fid = tuple_from_simplex(faces.front()).fid(*this);
        return tuple_from_edge(v[0], v[1], fid);
    }

    /**
     * @brief get faces (as Tuples) that are edge-adjacent to the given face (as Tuple)
     */
    std::vector<Tuple> get_edge_adjacent_faces(const Tuple& f) const
    {
        std::vector<Tuple> adj_tris;
        auto tri_1 = f.switch_face(*this);
        if (tri_1) {
            adj_tris.push_back(tri_1.value());
        }
        auto tri_2 = f.switch_edge(*this).switch_face(*this);
        if (tri_2) {
            adj_tris.push_back(tri_2.value());
        }
        auto tri_3 = f.switch_vertex(*this).switch_edge(*this).switch_face(*this);
        if (tri_3) {
            adj_tris.push_back(tri_3.value());
        }
        return adj_tris;
    }
};


} // namespace wmtk::components::topological_offset
