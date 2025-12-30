#pragma once
#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/simplex/RawSimplex.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <igl/write_triangle_mesh.h>
#include <igl/doublearea.h>
#include <igl/internal_angles.h>
#include <nlohmann/json.hpp>
#include <tbb/concurrent_priority_queue.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <fastenvelope/FastEnvelope.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <memory>
#include <queue>
#include <wmtk/Types.hpp>
namespace app::remeshing {

struct VertexAttributes
{
    Eigen::Vector3d pos;
    // TODO: in fact, partition id should not be vertex attribute, it is a fixed marker to distinguish tuple/operations.
    size_t partition_id;
    int is_freeze = 0;
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

class UniformRemeshing : public wmtk::TriMesh
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

    int retry_limit = 10;
    UniformRemeshing(
        std::vector<Eigen::Vector3d> _m_vertex_positions,
        int num_threads = 1,
        bool use_exact = true);

    ~UniformRemeshing() {}

    void create_mesh(
        size_t n_vertices,
        const std::vector<std::array<size_t, 3>>& tris,
        const std::vector<std::pair<size_t, int>>& frozen_verts = {},
        bool m_freeze = true,
        double eps = 0);

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

        std::map<wmtk::simplex::Edge, EdgeAttributes> edge_attrs;
        std::unordered_map<size_t, FaceAttributes> face_attrs;
    };
    tbb::enumerable_thread_specific<SplitInfoCache> split_info_cache;

    struct SwapInfoCache
    {
        size_t v0 = size_t(-1);
        size_t v1 = size_t(-1);

        // opposite vertices
        size_t v2 = size_t(-1);
        size_t v3 = size_t(-1);
        wmtk::Vector3d v0p;
        wmtk::Vector3d v1p;

        int is_feature_edge = 0;
        std::map<wmtk::simplex::Edge, EdgeAttributes> ring_edge_attrs;
    };
    tbb::enumerable_thread_specific<SwapInfoCache> swap_info_cache;


    struct PositionInfoCache
    {
        Eigen::Vector3d v1p;
        Eigen::Vector3d v2p;
        int partition_id;
        double v0_tal; // v0 target edge length
        double v1_tal; // v1 target edge length

        size_t v0 = size_t(-1);
        size_t v1 = size_t(-1);
        int is_feature_edge = 0;
    };
    tbb::enumerable_thread_specific<PositionInfoCache> position_cache;

    void cache_edge_positions(const Tuple& t);

    std::vector<std::array<size_t, 2>> get_edges_by_condition(
        std::function<bool(const EdgeAttributes&)> cond) const;

    bool invariants(const std::vector<Tuple>& new_tris) override;

    // TODO: this should not be here
    void partition_mesh();

    // TODO: morton should not be here, but inside wmtk
    void partition_mesh_morton();

    bool smooth_all_vertices();

    Eigen::Vector3d smooth(const Tuple& t);


    Eigen::Vector3d tangential_smooth(const Tuple& t);

    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;

    bool swap_edge_before(const Tuple& t) override;
    bool swap_edge_after(const Tuple& t) override;

    std::vector<TriMesh::Tuple> new_edges_after(const std::vector<TriMesh::Tuple>& tris) const;
    std::vector<TriMesh::Tuple> new_edges_after_swap(const TriMesh::Tuple& t) const;
    std::vector<TriMesh::Tuple> replace_edges_after_split(
        const std::vector<TriMesh::Tuple>& tris,
        const size_t vid_threshold) const;
    std::vector<TriMesh::Tuple> new_sub_edges_after_split(
        const std::vector<TriMesh::Tuple>& tris) const;


    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& t) override;

    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;

    double compute_edge_cost_collapse(const Tuple& t) const;
    double compute_edge_cost_split(const Tuple& t) const;
    double compute_vertex_valence(const Tuple& t) const;
    double compute_swap_energy(const Tuple& t) const;
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
    bool write_triangle_mesh(std::string path);

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

    void set_feature_vertices(const std::vector<size_t>& feature_vertices);
    void set_feature_edges(const std::vector<std::pair<std::array<size_t, 2>, int>>& feature_edges);
    void set_patch_ids(const std::vector<size_t>& patch_ids);
    bool is_feature_vertex(size_t vid) const;
    bool is_feature_edge(const Tuple& t) const;
    bool write_feature_vertices_obj(const std::string& path) const;

    void write_vtu(const std::string& path) const;

private:
    std::vector<std::pair<std::array<size_t, 2>, int>> m_input_feature_edges;
};

} // namespace app::remeshing