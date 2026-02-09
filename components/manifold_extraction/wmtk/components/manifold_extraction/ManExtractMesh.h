#pragma once

#include <igl/Timer.h>
#include <wmtk/TetMesh.h>
#include <wmtk/utils/Morton.h>
#include <wmtk/utils/PartitionMesh.h>
#include <bitset>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/simplex/RawSimplex.hpp>
#include "Parameters.h"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <tbb/concurrent_map.h>
#include <tbb/parallel_sort.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

// #include <igl/remove_unreferenced.h>
// #include <memory>


namespace wmtk::components::manifold_extraction {


// for all attributes:
// label: 0=default, 1=input, 2=offset
class VertexAttributes
{
public:
    Vector3d m_posf;
    int label = 0;
    bool in_out = false;

    VertexAttributes() {};
    VertexAttributes(const Vector3d& p);
};


class EdgeAttributes
{
public:
    bool in_out = false;
    int label = 0;
};


class FaceAttributes
{
public:
    bool in_out = false;
    int label = 0;
};


class TetAttributes
{
public:
    bool in_out = false; // in or out of mesh body
    int label = 0;
    double val = -999; // default unset value
};


class ManExtractMesh : public wmtk::TetMesh
{
public:
    int m_vtu_counter = 0;
    int m_surfvtu_counter = 0;
    std::array<size_t, 4> init_counts = {0, 0, 0, 0};
    bool m_boundary_input_vert_found = false;

    Parameters& m_params;

    using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    using EdgeAttCol = wmtk::AttributeCollection<EdgeAttributes>;
    using FaceAttCol = wmtk::AttributeCollection<FaceAttributes>;
    using TetAttCol = wmtk::AttributeCollection<TetAttributes>;
    VertAttCol m_vertex_attribute;
    EdgeAttCol m_edge_attribute;
    FaceAttCol m_face_attribute;
    TetAttCol m_tet_attribute;

    ManExtractMesh(Parameters& _m_params, int _num_threads = 0)
        : m_params(_m_params)
    {
        NUM_THREADS = _num_threads;
        p_vertex_attrs = &m_vertex_attribute;
        p_edge_attrs = &m_edge_attribute;
        p_face_attrs = &m_face_attribute;
        p_tet_attrs = &m_tet_attribute;
    }

    ~ManExtractMesh() {}

    ////// Attributes related
    void write_input_complex(const std::string& path); // write out components labeled as offset
                                                       // input (non manifold components)
    void write_vtu(const std::string& path); // debugging, write .vtu of tet mesh

    std::string tags_bit_rep(uint64_t tags) { return std::bitset<64>(tags).to_string(); }

    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& t) override;

    bool split_face_before(const Tuple& t) override;
    bool split_face_after(const Tuple& t) override;

    bool split_tet_before(const Tuple& t) override;
    bool split_tet_after(const Tuple& t) override;

    bool invariants(const std::vector<Tuple>& t) override; // this is now automatically checked

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
    // void init_from_image(
    //     const MatrixXd& V,
    //     const MatrixXi& T,
    //     const MatrixXd& T_tags,
    //     const std::map<std::string, int>& tag_label_map);

    void init_from_image(
        const MatrixXd& V, // V by 3
        const MatrixXi& T, // T by 4
        const MatrixXd& T_tag); // T by 1

    std::pair<size_t, size_t> label_non_manifold(); // return # of non manifold edges, verts
    bool edge_is_manifold(const Tuple& t) const;
    void edge_dfs_helper(std::set<size_t>& visited_tids, const Tuple& t) const;
    bool vertex_is_manifold(const Tuple& t) const;
    void vertex_dfs_helper(
        std::set<size_t>& visited_tids,
        const Tuple& t,
        const bool include,
        const std::vector<simplex::Face>& b_out_faces) const;
    bool is_boundary_vertex(size_t vid) const;
    std::vector<simplex::Face> get_boundary_faces_for_out_tets(size_t vid) const;


    bool is_simplicially_embedded() const;
    bool tet_is_simp_emb(const Tuple& t) const;
    void simplicial_embedding();

    void perform_offset();

    void extract_surface_mesh(MatrixXd& V, MatrixXi& F);

    // DEBUGGING
    void label_boundary_verts_1();
};

} // namespace wmtk::components::manifold_extraction
