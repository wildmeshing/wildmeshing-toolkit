#pragma once
#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/simplex/Simplex.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <igl/doublearea.h>
#include <igl/internal_angles.h>
#include <nlohmann/json.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include <fastenvelope/FastEnvelope.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <memory>
#include <queue>
#include <wmtk/Types.hpp>
namespace wmtk::components::cad_remeshing {

struct VertexAttributes
{
    Eigen::Vector3d pos;
    // TODO: in fact, partition id should not be vertex attribute, it is a fixed marker to distinguish tuple/operations.
    size_t partition_id;
    int corner_id = -1;
    bool is_feature = false;
    double tal = -1; // target edge length
};

struct EdgeAttributes
{
    int is_feature = 0;
};

struct FaceAttributes
{
    size_t patch_id = -1;
    double quality = -1;
};

class CadRemeshing : public wmtk::TriMesh
{
public:
    wmtk::SampleEnvelope m_envelope;
    wmtk::SampleEnvelope m_feature_envelope; // envelope for feature edges
    bool m_has_envelope = false;

    using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    VertAttCol vertex_attrs;

    using EdgeAttCol = wmtk::AttributeCollection<EdgeAttributes>;
    EdgeAttCol edge_attrs;

    using FaceAttCol = wmtk::AttributeCollection<FaceAttributes>;
    FaceAttCol face_attrs;

    CadRemeshing(
        std::vector<Eigen::Vector3d> _m_vertex_positions,
        int num_threads = 1,
        bool use_exact = true);

    ~CadRemeshing() {}

    /**
     * @brief Build the mesh and mark the corners.
     *
     * frozen_verts: (vertex id, corner id) pairs; the vertex ids are checked against n_vertices.
     * With freeze_boundary, every vertex on the open boundary of the mesh is frozen as well, with
     * corner id FROZEN_BOUNDARY_CORNER_ID, and so is every vertex a later split creates on the
     * boundary.
     */
    void create_mesh(
        size_t n_vertices,
        const std::vector<std::array<size_t, 3>>& tris,
        const std::vector<std::pair<size_t, int>>& frozen_verts = {},
        bool freeze_boundary = true);

    /// Corner id of a frozen boundary vertex: treated like a corner, but not a CAD corner.
    static constexpr int FROZEN_BOUNDARY_CORNER_ID = 100000;

    /**
     * @brief Mark the feature edges given to set_feature_edges and check that they form curves.
     *
     * Every feature vertex that is not a corner must have exactly two feature edges: a curve
     * that ends anywhere but at a corner, or two curves that meet without a corner, is rejected
     * with an error, since the remeshing relies on the curves being chains.
     */
    void initialize_feature_edges();

    void set_target_edge_length(const double tal);
    /**
     * @brief Set the target edge length to the per-patch average.
     *
     * Smooth the edge length in between vertices to have a difference of at max 1.5.
     */
    void set_per_patch_target_edge_length(const double factor);

    struct SplitInfoCache
    {
        // incident vertices
        size_t v0 = size_t(-1);
        size_t v1 = size_t(-1);
        wmtk::Vector3d v0p;
        wmtk::Vector3d v1p;
        double v0_tal; // v0 target edge length
        double v1_tal; // v1 target edge length

        int partition_id;

        int is_feature_edge = 0;
        bool is_boundary_edge = false;

        std::map<wmtk::simplex::Edge, EdgeAttributes> edge_attrs;
        std::unordered_map<size_t, FaceAttributes> face_attrs;
    };
    wmtk::threading::enumerable_thread_specific<SplitInfoCache> split_info_cache;

    struct SwapInfoCache
    {
        size_t v0 = size_t(-1);
        size_t v1 = size_t(-1);

        // opposite vertices
        size_t v2 = size_t(-1);
        size_t v3 = size_t(-1);
        wmtk::Vector3d v0p;
        wmtk::Vector3d v1p;

        std::map<wmtk::simplex::Edge, EdgeAttributes> ring_edge_attrs;
    };
    wmtk::threading::enumerable_thread_specific<SwapInfoCache> swap_info_cache;


    struct CollapseInfoCache
    {
        Eigen::Vector3d v0p;
        Eigen::Vector3d v1p;
        int partition_id;
        int v0_corner_id;
        int v1_corner_id;

        size_t v0 = size_t(-1);
        size_t v1 = size_t(-1);
        int is_feature_edge = 0;
        std::vector<std::tuple<size_t, int>>
            link_vertices; // store the edge-link vertex v2 and the feature ID of (v0,v2) or (v1,v2)
    };
    wmtk::threading::enumerable_thread_specific<CollapseInfoCache> collapse_info_cache;

    bool cache_edge_positions(const Tuple& t);

    std::vector<std::array<size_t, 2>> get_edges_by_condition(
        std::function<bool(const EdgeAttributes&)> cond) const;

    bool invariants(const std::vector<Tuple>& new_tris) override;

    // TODO: morton should not be here, but inside wmtk
    void partition_mesh_morton();

    size_t get_partition_id(const Tuple& loc) const
    {
        return vertex_attrs[loc.vid(*this)].partition_id;
    }

    bool smooth_all_vertices();

    Eigen::Vector3d smooth(const Tuple& t);


    Eigen::Vector3d tangential_smooth(const Tuple& t);

    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;

    bool swap_edge_before(const Tuple& t) override;
    bool swap_edge_after(const Tuple& t) override;

    std::vector<TriMesh::Tuple> new_edges_after(const std::vector<TriMesh::Tuple>& tris) const;
    std::vector<TriMesh::Tuple> replace_edges_after_split(
        const std::vector<TriMesh::Tuple>& tris,
        const size_t vid_threshold) const;


    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& t) override;

    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;

    double compute_edge_cost_collapse(const Tuple& t) const;
    double compute_edge_cost_split(const Tuple& t) const;
    double compute_vertex_valence(const Tuple& t) const;
    /**
     * @brief Report statistics.
     *
     * Returns a vector with:
     * average_length
     * max length
     * min length
     * average valence
     * max valence
     * min valence
     */
    std::vector<double> average_len_valen();
    bool split_remeshing();
    bool collapse_remeshing();
    bool swap_remeshing();
    bool uniform_remeshing(int interations, bool debug_output = false);

    /**
     * @brief Compute the shape quality.
     *
     * The shape quality is the area divided by the sum of the squared edge lengths.
     * The quality is normalized, i.e., the best quality is 1 and the worst is 0.
     */
    double shape_quality(
        const wmtk::Vector3d& p0,
        const wmtk::Vector3d& p1,
        const wmtk::Vector3d& p2) const;

    /**
     * @brief Get the quality of a single triangle.
     */
    double get_quality(const std::array<size_t, 3>& vs) const;
    double get_quality(const Tuple& t) const;
    /**
     * @brief Update quality of all triangles.
     */
    void update_qualities();

    void set_feature_edges(const std::vector<std::pair<std::array<size_t, 2>, int>>& feature_edges);
    void set_patch_ids(const std::vector<size_t>& patch_ids);
    bool is_feature_edge(const Tuple& t) const;

    /**
     * @brief Write the mesh to <path>.vtu and the feature edges to <path>_edges.vtu.
     *
     * If report_file is not empty, the "after" angle and double-area statistics are added to the
     * JSON report stored in that file.
     */
    void write_vtu(const std::string& path, const std::string& report_file = "") const;

private:
    std::vector<std::pair<std::array<size_t, 2>, int>> m_input_feature_edges;
    bool m_freeze_boundary = false;

    /// The id of the edge between two vertices that are connected by one.
    size_t edge_id(size_t v0, size_t v1) const { return std::get<1>(tuple_from_edge({{v0, v1}})); }

    void check_feature_curves() const;
};

} // namespace wmtk::components::cad_remeshing