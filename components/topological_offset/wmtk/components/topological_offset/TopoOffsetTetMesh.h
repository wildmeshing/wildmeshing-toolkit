#pragma once

#include <igl/Timer.h>
#include <wmtk/TetMesh.h>
#include <wmtk/simplex/RawSimplex.hpp>
#include "Parameters.h"
#include "SimplicialComplexBVH.hpp"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on


namespace wmtk::components::topological_offset {


// for all attributes:
// label: 0=default, 1=input, 2=offset
class VertexAttributes
{
public:
    Vector3d m_posf;
    int label = 0;

    VertexAttributes() {};
    VertexAttributes(const Vector3d& p);
};


class EdgeAttributes
{
public:
    int label = 0;
};


class FaceAttributes
{
public:
    int label = 0;
};


class TetAttributes
{
public:
    int label = 0; // must be zero for all tets
    std::vector<double> tags;
};


class TopoOffsetTetMesh : public wmtk::TetMesh
{
public: // mode for splitting in marching tets
    enum class EdgeSplitMode {
        Midpoint = 0,
        BinarySearch =
            1 // requires that every edge being split has one vertex labelled 0 and the other 1 or 2
    };

public:
    int m_vtu_counter = 0;
    std::array<size_t, 4> m_init_counts = {{0, 0, 0, 0}};
    size_t m_tags_count;
    SimplicialComplexBVH m_input_complex_bvh;
    EdgeSplitMode m_edge_split_mode = EdgeSplitMode::Midpoint;

    Parameters& m_params;

    using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    using EdgeAttCol = wmtk::AttributeCollection<EdgeAttributes>;
    using FaceAttCol = wmtk::AttributeCollection<FaceAttributes>;
    using TetAttCol = wmtk::AttributeCollection<TetAttributes>;
    VertAttCol m_vertex_attribute;
    EdgeAttCol m_edge_attribute;
    FaceAttCol m_face_attribute;
    TetAttCol m_tet_attribute;

    TopoOffsetTetMesh(Parameters& _m_params, int _num_threads = 0)
        : m_params(_m_params)
    {
        NUM_THREADS = _num_threads;
        p_vertex_attrs = &m_vertex_attribute;
        p_edge_attrs = &m_edge_attribute;
        p_face_attrs = &m_face_attribute;
        p_tet_attrs = &m_tet_attribute;
    }

    ~TopoOffsetTetMesh() {}

    /**
     * @brief initialize TetMesh from vertex, tet, and tag data
     * @param V: #V by 3 vertex matrix
     * @param T: #T by 4 tet matrix
     * @param T_tags: #T by #tags tag matrix
     */
    void init_from_image(const MatrixXd& V, const MatrixXi& T, const MatrixXd& T_tags);

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
    void edge_split_binary_search(const size_t v1, const size_t v2, Vector3d& p_new) const;

    //// overriden splits/invariants
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& t) override;
    bool split_face_before(const Tuple& t) override;
    bool split_face_after(const Tuple& t) override;
    bool split_tet_before(const Tuple& t) override;
    bool split_tet_after(const Tuple& t) override;
    bool invariants(const std::vector<Tuple>& tets) override;
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
     * @brief check if a tet satisfies simpicial embedding criteria w.r.t. input complex
     * (simplices labelled 1)
     */
    bool tet_is_simp_emb(const Tuple& t) const;

    /**
     * @brief make mesh a simplicial embedding of the input complex (simplices labelled 1)
     */
    void simplicial_embedding();
    //// simplicial embedding stuff

    //// variable offset stuff
    /**
     * @brief check if adding a tet to the offset region does not change the topology of the
     * offset. Returns true if topology would not be changed
     */
    bool tet_consistent_topology(const size_t t_id) const;

    /**
     * @brief check if a tet is inside the offset (implicitly defined via BVH distance field to
     * input complex) via conservative sphere subdivision estimation
     */
    bool tet_is_in_offset_conservative(const size_t t_id, const double threshold_r) const;

    /**
     * @brief grow offset region conservatively using conservative checks while ensuring consistent
     * topology
     */
    void grow_offset_conservative();
    //// variable offset stuff

    /**
     * @brief update 'tags' data for tets in the offset region (tets labelled 2) based on
     * the given offset tag values in m_params.offset_tag_value
     */
    void set_offset_tet_tags();

    /**
     * @brief verify that the closed offset region (simplices labelled 1 or 2) form a manifold
     * region. This should be true for any offset. This function is for verification.
     * @note We first collect the tets labelled 1 or 2, then extract the boundary of this region
     * and check if it is manifold.
     */
    bool offset_is_manifold();

    //// output stuff
    void write_input_complex(const std::string& path); // write components labeled to be offset
    void write_vtu(const std::string& path); // debugging, write .vtu of tet mesh
    void write_msh(const std::string& file);
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
        VertexAttributes new_v;

        // cache edge attributes
        EdgeAttributes split_e;
        std::map<size_t, EdgeAttributes> internal_e;
        std::map<simplex::Edge, EdgeAttributes> external_e; // edge is boundary edge (not link)
        std::map<simplex::Edge, EdgeAttributes> link_e;

        // cache face attributes
        std::map<size_t, FaceAttributes> split_f;
        std::map<simplex::Edge, FaceAttributes> internal_f;
        std::map<std::pair<simplex::Edge, size_t>, FaceAttributes> external_f;

        // cache tet attributes
        std::map<simplex::Edge, TetAttributes> tets;
    };
    tbb::enumerable_thread_specific<EdgeSplitCache> edge_split_cache;

    struct FaceSplitCache
    {
        size_t v1_id;
        size_t v2_id;
        size_t v3_id;

        // cache edge attributes
        std::map<simplex::Edge, EdgeAttributes> existing_e;

        // cache face attributes
        std::map<simplex::Face, FaceAttributes> existing_f;
        int splitf_label;

        // cache tet attributes
        std::map<size_t, TetAttributes> tets;
    };
    tbb::enumerable_thread_specific<FaceSplitCache> face_split_cache;

    struct TetSplitCache
    {
        std::array<size_t, 4> v_ids;

        // cache retained edge attributes
        std::map<simplex::Edge, EdgeAttributes> existing_e;

        // cache retained face attributes
        std::map<simplex::Face, FaceAttributes> existing_f;

        // cache tet attribute
        TetAttributes tet;
    };
    tbb::enumerable_thread_specific<TetSplitCache> tet_split_cache;

private: // helpers
    /**
     * @brief check if a face is in the intersection region of offset_tags
     */
    bool face_in_tag_intersection(const Tuple& f)
    {
        // get adjacent tet(s) for face
        std::vector<size_t> nb_tids;
        nb_tids.push_back(f.tid(*this));
        auto other = f.switch_tetrahedron(*this);
        if (other) {
            nb_tids.push_back(other.value().tid(*this));
        }

        for (const auto& pair : m_params.offset_tags) {
            bool tag_found = false;
            for (const size_t nb_tid : nb_tids) {
                if (m_tet_attribute[nb_tid].tags[pair[0]] == pair[1]) {
                    tag_found = true;
                    break;
                }
            }

            if (tag_found) {
                continue;
            }
            return false; // no nb tets have this tag
        }
        return true; // all offset_tags have at least one incident tet
    }

    /**
     * @brief check if an edge is in the intersection region of offset_tags
     */
    bool edge_in_tag_intersection(const Tuple& e)
    {
        auto nb_tids = get_incident_tids_for_edge(e); // get incident tets
        for (const auto& pair : m_params.offset_tags) {
            bool tag_found = false;
            for (const size_t& t_id : nb_tids) {
                if (m_tet_attribute[t_id].tags[pair[0]] == pair[1]) {
                    tag_found = true;
                    break;
                }
            }

            if (tag_found) {
                continue;
            }
            return false; // no incident tet has given tag in offset_tags
        }
        return true; // all offset_tags found in at least one incident tet to edge
    }

    /**
     * @brief check if a vertex is in the intersection region of offset_tags
     */
    bool vertex_in_tag_intersection(const Tuple& v)
    {
        auto nb_tids = get_one_ring_tids_for_vertex(v.vid(*this));
        for (const auto& pair : m_params.offset_tags) {
            bool tag_found = false;
            for (const size_t t_id : nb_tids) {
                if (m_tet_attribute[t_id].tags[pair[0]] == pair[1]) {
                    tag_found = true;
                    break;
                }
            }

            if (tag_found) {
                continue;
            }
            return false;
        }
        return true;
    }

    /**
     * @brief sort edge simplices in place by decreasing edge length
     */
    void sort_edges_by_length(std::vector<simplex::Edge>& edges)
    {
        std::sort(
            edges.begin(),
            edges.end(),
            [this](const simplex::Edge& e1, const simplex::Edge& e2) {
                double len1 = (m_vertex_attribute[e1.vertices()[0]].m_posf -
                               m_vertex_attribute[e1.vertices()[1]].m_posf)
                                  .norm();
                double len2 = (m_vertex_attribute[e2.vertices()[0]].m_posf -
                               m_vertex_attribute[e2.vertices()[1]].m_posf)
                                  .norm();
                return len1 > len2;
            });
    }

public: // helpers
    /**
     * @brief get tets (as Tuples) that are face-adjacent to the given tet (as Tuple)
     */
    std::vector<Tuple> get_face_adjacent_tets(const Tuple& t) const
    {
        std::vector<Tuple> adj_tets;
        auto tet_1 = t.switch_tetrahedron(*this);
        if (tet_1) {
            adj_tets.push_back(tet_1.value());
        }
        auto tet_2 = t.switch_face(*this).switch_tetrahedron(*this);
        if (tet_2) {
            adj_tets.push_back(tet_2.value());
        }
        auto tet_3 = t.switch_edge(*this).switch_face(*this).switch_tetrahedron(*this);
        if (tet_3) {
            adj_tets.push_back(tet_3.value());
        }
        auto tet_4 =
            t.switch_vertex(*this).switch_edge(*this).switch_face(*this).switch_tetrahedron(*this);
        if (tet_4) {
            adj_tets.push_back(tet_4.value());
        }
        return adj_tets;
    }
};


} // namespace wmtk::components::topological_offset
