#pragma once

#include <igl/Timer.h>
#include <wmtk/TetMesh.h>
#include <wmtk/utils/Morton.h>
#include <wmtk/utils/PartitionMesh.h>
#include <bitset>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/simplex/Simplex.hpp>
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
    std::array<size_t, 4> init_counts = {{0, 0, 0, 0}};
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

    // splitting/invariants
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& t) override;
    bool split_face_before(const Tuple& t) override;
    bool split_face_after(const Tuple& t) override;
    bool split_tet_before(const Tuple& t) override;
    bool split_tet_after(const Tuple& t) override;
    bool invariants(const std::vector<Tuple>& t) override; // this is now automatically checked

private:
    /**
     * @brief attributes cache for edge split. All simplices inherent attributes from their parent
     * during the split.
     */
    struct EdgeSplitCache
    {
        size_t v1_id;
        size_t v2_id;
        VertexAttributes new_v;
        EdgeAttributes split_e; // split edge
        std::map<size_t, EdgeAttributes> internal_e; // internal edges
        std::map<simplex::Edge, EdgeAttributes> external_e; // edge is boundary edge (not link)
        std::map<simplex::Edge, EdgeAttributes> link_e;
        std::map<size_t, FaceAttributes> split_f; // faces incident to split edge
        std::map<simplex::Edge, FaceAttributes> internal_f; // new internal faces
        std::map<std::pair<simplex::Edge, size_t>, FaceAttributes> external_f; // retained faces
        std::map<simplex::Edge, TetAttributes> tets; // tets incident to split edge
    };
    tbb::enumerable_thread_specific<EdgeSplitCache> edge_split_cache;

    /**
     * @brief attributes cache for face split. All simplices inheret attributes from their parent
     * during the split.
     */
    struct FaceSplitCache
    {
        size_t v1_id;
        size_t v2_id;
        size_t v3_id;
        std::map<simplex::Edge, EdgeAttributes> existing_e; // retained edges
        std::map<simplex::Face, FaceAttributes> existing_f; // retained faces
        int splitf_label; // split face
        std::map<size_t, TetAttributes> tets; // incident tets
    };
    tbb::enumerable_thread_specific<FaceSplitCache> face_split_cache;

    /**
     * @brief attributes cache for tet split. All simplices inheret attributes from their parent
     * during the split.
     */
    struct TetSplitCache
    {
        std::array<size_t, 4> v_ids;
        std::map<simplex::Edge, EdgeAttributes> existing_e; // retained edges
        std::map<simplex::Face, FaceAttributes> existing_f; // retained faces
        TetAttributes tet; // split tet
    };
    tbb::enumerable_thread_specific<TetSplitCache> tet_split_cache;

public:
    /**
     * @brief initialize mesh from vertices, faces, and tag matrices
     */
    void init_from_image(
        const MatrixXd& V, // V by 3
        const MatrixXi& T, // T by 4
        const MatrixXd& T_tag); // T by 1

    /**
     * @brief label nonmanifold simplices (edges and verts). Corresponding attributes have label set
     * to 1
     */
    std::pair<size_t, size_t> label_non_manifold(); // return # of non manifold edges, verts

    /**
     * @brief check if an edge is manifold, ie if all 'inside' tets adjacent to the edge form
     * single face-reachable connected component
     */
    bool edge_is_manifold(const Tuple& t) const;

    /**
     * @brief dfs helper for edge manifold check
     */
    void edge_dfs_helper(std::set<size_t>& visited_tids, const Tuple& t) const;

    /**
     * @brief check if a vertex is manifold. A vertex is manifold if all 'inside' and all 'outside'
     * tets adjacent to the vertex form single face-reachable connected components, respectively.
     * For vertices on the boundary of the mesh, external space is considered an 'outside' polyhedra
     * that is face connected to all tets on the mesh boundary.
     */
    bool vertex_is_manifold(const Tuple& t) const;

    /**
     * @brief dfs helper for vertex manifold check
     */
    void vertex_dfs_helper(
        std::set<size_t>& visited_tids,
        const Tuple& t,
        const bool include,
        const std::vector<simplex::Face>& b_out_faces) const;

    /**
     * @brief check if a vertex is on the boundary of the mesh
     */
    bool is_boundary_vertex(size_t vid) const;

    /**
     * @brief collect faces on the boundary of the mesh that belong to an 'outside' tet.
     */
    std::vector<simplex::Face> get_boundary_faces_for_out_tets(size_t vid) const;

    /**
     * @brief check if input complex (simplices labeled 1) are simplicially embedded by the
     * background mesh
     */
    bool is_simplicially_embedded() const;

    /**
     * @brief check if a tet meets simplicial embedding criteria (i.e., contains zero/one
     * vertex, one edge, one face, or entire tet in input complex)
     */
    bool tet_is_simp_emb(const Tuple& t) const;

    /**
     * @brief make the background mesh a simplicial embedding via tet/face/edge splits.
     * Since the input complex is just edges and vertices, tets will never be split.
     */
    void simplicial_embedding();

    /**
     * @brief perform offset via "marching tets". All edges between input complex and background are
     * split (ie, edges with one vertex labeled 0 and the other vertex labeled 1)
     */
    void perform_offset();

    /**
     * @brief extract tri mesh separating 'inside' tets from 'outside' tets. Resulting mesh is
     * guaranteed to be manifold.
     */
    void extract_surface_mesh(MatrixXd& V, MatrixXi& F);

    /**
     * @brief write input complex (non manifold components) to vtu file
     */
    void write_input_complex(const std::string& path);

    /**
     * @brief write tet mesh to vtu file
     */
    void write_vtu(const std::string& path); // debugging, write .vtu of tet mesh
};

} // namespace wmtk::components::manifold_extraction
