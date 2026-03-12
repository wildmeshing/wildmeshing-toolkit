#pragma once
#include <wmtk/TriMesh.h>
#include <SimpleBVH/BVH.hpp>
#include "Parameters.h"


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
    std::vector<double> tags;
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
    SimpleBVH::BVH m_input_complex_bvh;
    EdgeSplitMode m_edge_split_mode = EdgeSplitMode::Midpoint;

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

    // initialization
    void init_from_image(
        const MatrixXd& V, // V by 3
        const MatrixXi& F, // F by 3
        const MatrixXd& F_tags); // F by N
    void init_input_complex_bvh();

    // splitting
    void edge_split_binary_search(const size_t v1, const size_t v2, Vector2d& p_new) const;
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& t) override;
    bool split_face_before(const Tuple& t) override;
    bool split_face_after(const Tuple& t) override;
    bool invariants(const std::vector<Tuple>& tris) override;

    // simplicial embedding
    bool is_simplicially_embedded() const;
    bool tri_is_simp_emb(const Tuple& t) const;
    void simplicial_embedding();

    // adaptive offset
    bool tri_consistent_topology(const size_t f_id) const;
    bool tri_is_in_offset_conservative(const size_t f_id, const double threshold_r) const;
    void grow_offset_conservative();
    void marching_tets();

    // set tag for offset region
    void set_offset_tri_tags();

    // output
    void write_input_complex(const std::string& path);
    void write_vtu(const std::string& path);
    void write_msh(const std::string& file);

private:
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
     * @brief check whether an edge is in offset intersection (ie, edge is adjacent to at
     * least one face of every [tag num, tag val] pair in offset_tags)
     */
    bool edge_in_tag_intersection(const Tuple& e)
    {
        // collect adjacent face(s)
        std::vector<size_t> nb_fids;
        nb_fids.push_back(e.fid(*this));
        auto other = e.switch_face(*this);
        if (other) {
            nb_fids.push_back(other.value().fid(*this));
        }

        for (const auto& pair : m_params.offset_tags) {
            bool tag_found = false;
            for (size_t nb_fid : nb_fids) {
                if (m_face_attribute[nb_fid].tags[pair[0]] == pair[1]) {
                    tag_found = true;
                    break;
                }
            }

            if (tag_found) {
                continue;
            }
            return false; // one of offset_tags not adjacent to edge. not in intersection region
        }
        return true; // all offset_tags found adjacent to edge
    }

    /**
     * @brief check whether a vertex is in offset intersection (ie, vertex is adjacent
     * to at least one face of every [tag num, tag val] pair in offset_tags)
     */
    bool vertex_in_tag_intersection(const Tuple& v)
    {
        // collect adjacent face(s)
        auto nb_fids = get_one_ring_fids_for_vertex(v.vid(*this));

        for (const auto& pair : m_params.offset_tags) {
            bool tag_found = false;
            for (size_t nb_fid : nb_fids) {
                if (m_face_attribute[nb_fid].tags[pair[0]] == pair[1]) {
                    tag_found = true;
                    break;
                }
            }

            if (tag_found) {
                continue;
            }
            return false; // one of offset_tags not found. vert not in intersection region
        }
        return true; // all offset_tags found, vert is in intersection region
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

    Tuple get_tuple_from_edge(const simplex::Edge& e) const
    {
        const auto& v = e.vertices();
        const auto faces = simplex_incident_triangles(e).faces();
        assert(!faces.empty());
        const size_t fid = tuple_from_simplex(faces.front()).fid(*this);
        return tuple_from_edge(v[0], v[1], fid);
    }

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

public: // spatial queries
    double dist_to_input_complex(const Vector2d& p) const;
};


} // namespace wmtk::components::topological_offset