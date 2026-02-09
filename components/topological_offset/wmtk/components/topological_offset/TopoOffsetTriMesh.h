#pragma once
#include <wmtk/TriMesh.h>
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
public:
    int m_vtu_counter = 0;
    std::array<size_t, 3> m_init_counts = {0, 0, 0};
    std::vector<std::string> m_all_tag_names;
    size_t m_tags_count;
    int m_toi_ind; // index of tag of interest

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

    void init_from_image(
        const MatrixXd& V, // V by 3
        const MatrixXi& F, // F by 3
        const MatrixXd& F_tags, // F by N
        const std::vector<std::string>& all_tag_names);

    // splitting
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& t) override;
    bool split_face_before(const Tuple& t) override;
    bool split_face_after(const Tuple& t) override;
    bool invariants(const std::vector<Tuple>& tets) override;

    // offset functions
    bool is_simplicially_embedded() const;
    bool tri_is_simp_emb(const Tuple& t) const;
    void simplicial_embedding();
    void perform_offset();

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
    int count_sep_tags(const std::set<int> tags) const
    {
        auto sep_tags = m_params.sep_tag_vals;
        return std::count_if(tags.begin(), tags.end(), [&sep_tags](int elem) {
            return (std::find(sep_tags.begin(), sep_tags.end(), elem) != sep_tags.end());
        });
    }

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
};


} // namespace wmtk::components::topological_offset