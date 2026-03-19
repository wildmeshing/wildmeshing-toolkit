#pragma once

#include <igl/Timer.h>
#include <wmtk/TetMesh.h>
#include <wmtk/simplex/RawSimplex.hpp>
#include "Parameters.h"

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


class TopoOffsetMesh : public wmtk::TetMesh
{
public:
    int m_vtu_counter = 0;
    std::array<size_t, 4> m_init_counts = {{0, 0, 0, 0}};
    std::vector<std::string> m_all_tag_names;
    size_t m_tags_count;
    int m_toi_ind; // index of tag of interest

    Parameters& m_params;

    using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    using EdgeAttCol = wmtk::AttributeCollection<EdgeAttributes>;
    using FaceAttCol = wmtk::AttributeCollection<FaceAttributes>;
    using TetAttCol = wmtk::AttributeCollection<TetAttributes>;
    VertAttCol m_vertex_attribute;
    EdgeAttCol m_edge_attribute;
    FaceAttCol m_face_attribute;
    TetAttCol m_tet_attribute;

    TopoOffsetMesh(Parameters& _m_params, int _num_threads = 0)
        : m_params(_m_params)
    {
        NUM_THREADS = _num_threads;
        p_vertex_attrs = &m_vertex_attribute;
        p_edge_attrs = &m_edge_attribute;
        p_face_attrs = &m_face_attribute;
        p_tet_attrs = &m_tet_attribute;
    }

    ~TopoOffsetMesh() {}

    void write_input_complex(const std::string& path); // write components labeled to be offset
    void write_vtu(const std::string& path); // debugging, write .vtu of tet mesh
    void write_msh(const std::string& file);

    // std::string tags_bit_rep(uint64_t tags) { return std::bitset<64>(tags).to_string(); }

    // splitting
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& t) override;
    bool split_face_before(const Tuple& t) override;
    bool split_face_after(const Tuple& t) override;
    bool split_tet_before(const Tuple& t) override;
    bool split_tet_after(const Tuple& t) override;
    bool invariants(const std::vector<Tuple>& tets) override; // this is now automatically checked

private:
    // for edge splitting, new simplices inheret attributes from higher
    // simplex they 'grew' out of
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

public:
    void init_from_image(
        const MatrixXd& V, // V by 3
        const MatrixXi& T, // T by 4
        const MatrixXd& T_tags, // T by N
        const std::vector<std::string>& all_tag_names);

    bool is_simplicially_embedded() const;
    bool tet_is_simp_emb(const Tuple& t) const;
    void simplicial_embedding();
    void perform_offset();

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
};

} // namespace wmtk::components::topological_offset
