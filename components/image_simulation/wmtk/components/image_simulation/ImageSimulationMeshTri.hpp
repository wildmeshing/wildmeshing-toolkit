#pragma once

#include <limits>
#include <unordered_set>

#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <igl/write_triangle_mesh.h>
#include <tbb/concurrent_priority_queue.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <fastenvelope/FastEnvelope.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include "Parameters.h"

namespace wmtk::components::image_simulation::tri {

struct VertexAttributes
{
    Vector2d m_pos;
    bool m_is_on_surface = false;
    std::vector<int> on_bbox_faces;

    double m_sizing_scalar = 1;

    size_t partition_id = 0;

    VertexAttributes() {}
    VertexAttributes(const Vector2d& p)
        : m_pos(p)
    {}
};

class EdgeAttributes
{
public:
    double tag; // TODO: is this used?

    bool m_is_surface_fs = false; // 0; 1
    /**
     * Keep track which bbox side the face is on
     * -1: none
     * 0/1: x min/max
     * 2/3: y min/max
     * 4/5: z min/max
     *
     * This bbox side ID is used to keep the bbox from collapsing.
     */
    int m_is_bbox_fs = -1; //-1; 0~5

    void reset()
    {
        m_is_surface_fs = false;
        m_is_bbox_fs = -1;
    }

    void merge(const EdgeAttributes& attr)
    {
        m_is_surface_fs = m_is_surface_fs || attr.m_is_surface_fs;
        if (attr.m_is_bbox_fs >= 0) m_is_bbox_fs = attr.m_is_bbox_fs;
    }
};

class FaceAttributes
{
public:
    double m_quality;
    double m_winding_number = 0;
    std::vector<int64_t> tags;
    int part_id = -1;
};

class ImageSimulationMeshTri : public wmtk::TriMesh
{
public:
    int m_debug_print_counter = 0;
    size_t m_tags_count = 0;

    const double MAX_ENERGY = std::numeric_limits<double>::max();

    Parameters& m_params;
    std::vector<Vector2d> m_V_envelope;
    std::vector<Vector2i> m_E_envelope;
    std::shared_ptr<SampleEnvelope> m_envelope;
    double m_envelope_eps = -1;

    using VertAttCol = AttributeCollection<VertexAttributes>;
    using EdgeAttCol = AttributeCollection<EdgeAttributes>;
    using FaceAttCol = AttributeCollection<FaceAttributes>;
    VertAttCol m_vertex_attribute;
    EdgeAttCol m_edge_attribute;
    FaceAttCol m_face_attribute;

    bool m_collapse_check_link_condition = false; // classical link condition
    bool m_collapse_check_topology = false; // sanity check
    bool m_collapse_check_manifold = false; // manifoldness check after collapse

    ImageSimulationMeshTri(Parameters& _m_params, double envelope_eps, int _num_threads = 0)
        : m_params(_m_params)
        , m_envelope_eps(envelope_eps)
    {
        NUM_THREADS = _num_threads;
        p_vertex_attrs = &m_vertex_attribute;
        p_edge_attrs = &m_edge_attribute;
        p_face_attrs = &m_face_attribute;
    }

    ~ImageSimulationMeshTri() {}

    // TODO: this should not be here
    void partition_mesh();

    // TODO: morton should not be here, but inside wmtk
    void partition_mesh_morton();

    size_t get_partition_id(const Tuple& loc) const
    {
        return m_vertex_attribute[loc.vid(*this)].partition_id;
    }

    double get_length2(const Tuple& l) const;


public:
    /**
     * @brief Init from meshes image.
     *
     * @param V #Vx3 vertices of the tet mesh
     * @param T #Tx4 vertex IDs for all faces
     * @param T_tags #Tx1 image data represented by the individual faces
     */
    void init_from_image(const MatrixXd& V, const MatrixXi& T, const MatrixXi& T_tags);

    void init_surfaces_and_boundaries();

    void init_envelope(const MatrixXd& V, const MatrixXi& F);

    bool adjust_sizing_field_serial(double max_energy);

    void write_msh(std::string file);

    void write_vtu(const std::string& path) const;

    std::vector<std::array<size_t, 2>> get_edges_by_condition(
        std::function<bool(const EdgeAttributes&)> cond) const;

public:
    void split_all_edges();
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& loc) override;

    void collapse_all_edges(bool is_limit_length = true);
    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;

    size_t swap_all_edges();
    /**
     * @brief The quality improvement of a swap.
     *
     * Used to determine the priority and weight of a swap operation.
     */
    double swap_weight(const Tuple& t) const;
    bool swap_edge_before(const Tuple& t) override;
    bool swap_edge_after(const Tuple& t) override;

    void smooth_all_vertices();
    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;
    //
    /**
     * @brief Inversion check using only floating point numbers.
     */
    bool is_inverted(const std::array<size_t, 3>& vs) const;
    bool is_inverted(const Tuple& loc) const;
    bool is_inverted(const size_t fid) const;
    double get_quality(const std::array<size_t, 3>& vs) const;
    double get_quality(const Tuple& loc) const;
    double get_quality(const size_t fid) const;

    //
    bool is_edge_on_surface(const Tuple& loc) const;
    bool is_edge_on_surface(const std::array<size_t, 2>& vids) const;
    bool is_edge_on_bbox(const Tuple& loc) const;
    bool is_edge_on_bbox(const std::array<size_t, 2>& vids) const;
    //
    void mesh_improvement(int max_its = 80);
    std::tuple<double, double> local_operations(
        const std::array<int, 4>& ops,
        bool collapse_limit_length = true);
    std::tuple<double, double> get_max_avg_energy();

    /**
     * @brief Connected Component according to tag_0
     *
     * @param tag value of tag_0
     * @param faces vector of face ids in the connected component
     * @param surrounding_comp_ids set of distinct connected component ids (in an external list) of
     * all neighboring faces across tag_0 boundaries
     * @param area total area of the connected component
     * @param touches_boundary whether the connected component touches the boundary of the mesh
     */
    struct ConnectedComponent
    {
        int64_t tag = -1;
        std::vector<size_t> faces;
        std::unordered_set<size_t> surrounding_comp_ids;
        double area = 0.0;
        bool touches_boundary = false;
    };

    /**
     * @brief Compute connected components of the mesh according to tag_0, and return a vector of
     * ConnectedComponent structs.
     */
    std::vector<ConnectedComponent> compute_connected_components() const;

    /**
     * @brief Replace a component with id hole_comp_id with another component with id
     * engulfing_comp_id, and keep the components vector consistent. This is used to fill holes in
     * the mesh.
     * @param components vector of ConnectedComponent structs, representing the connected components
     * of the mesh according to tag_0
     * @param hole_comp_id the id of the component to be engulfed (i.e. the hole)
     * @param engulfing_comp_id the id of the component that engulfs the hole
     */
    void engulf_component(
        std::vector<ConnectedComponent>& components,
        const size_t hole_comp_id,
        const size_t engulfing_comp_id);

    /**
     * @brief Replace each component in hole_comp_ids with nearest component from
     * engulfing_comp_ids, and keep the components vector consistent.
     * @param components vector of ConnectedComponent structs, representing the connected components
     * of the mesh according to tag_0
     * @param hole_comp_ids vector of ids of components to be engulfed (i.e. the holes)
     * @param engulfing_comp_ids set of ids of components that can engulf the holes
     */
    void engulf_components(
        std::vector<ConnectedComponent>& components,
        const std::vector<size_t>& hole_comp_ids,
        const std::unordered_set<size_t>& engulfing_comp_ids);

    /**
     * @brief Keep only the largest connected component for each of the distinct tag_0 values, and
     * engulf all other components.
     *
     * @param lcc_tags
     */
    void keep_largest_connected_component(const std::vector<int64_t>& lcc_tags);

    /**
     * @brief A cluster of connected components. Used to store a maximal group of connected
     * non-fill-tag ConnectedComponents for filling holes.
     * @param comp_ids vector of indices into the components vector, representing the connected
     * components in the cluster
     * @param area total area of the cluster
     * @param enclosed whether the cluster is enclosed by fill_tag components (i.e. whether the
     * cluster is a hole that can be filled)
     */
    struct ComponentCluster
    {
        std::vector<size_t> comp_ids; // indices into components vector
        double area = 0.0;
        bool enclosed = true;
    };

    /**
     * @brief Extract clusters of connected components that represent holes in the mesh.
     *
     * @param components vector of ConnectedComponent structs, representing the connected components
     * of the mesh according to tag_0
     * @param tags set of tag_0 values which define the holes to be filled
     * @param hole_clusters vector of clusters of connected components that represent holes in the
     * mesh.
     * @param threshold area threshold for the hole clusters. Clusters with area below the threshold
     * will not be filled.
     */
    void extract_hole_clusters(
        std::vector<ConnectedComponent>& components,
        std::unordered_set<int64_t>& tags,
        std::vector<std::vector<size_t>>& hole_clusters,
        double threshold);

    void recompute_surface_info();

    void fill_holes_topo(
        const std::vector<int64_t>& fill_holes_tags,
        double threshold = std::numeric_limits<double>::infinity());

    void tight_seal_topo(
        const std::vector<std::unordered_set<int64_t>>& tight_seal_tag_sets,
        double threshold = std::numeric_limits<double>::infinity());


    /**
     * @brief Get all edges on the surface that are incident to vid.
     */
    simplex::RawSimplexCollection get_order1_edges_for_vertex(const size_t vid) const;

    size_t get_order_of_edge(const std::array<size_t, 2>& vids) const;
    size_t get_order_of_vertex(const size_t vid) const;

    bool substructure_link_condition(const Tuple& e_tuple) const;

private:
    ////// Operations

    struct SplitInfoCache
    {
        //        VertexAttributes vertex_info;
        size_t v1_id;
        size_t v2_id;
        std::vector<size_t> v1_param_type;
        std::vector<size_t> v2_param_type;

        EdgeAttributes old_e_attrs;

        // std::vector<std::pair<EdgeAttributes, std::array<size_t, 2>>> changed_edges;
        std::map<simplex::Edge, EdgeAttributes> changed_edges;

        /**
         * All faces incident to the splitted edge, identified by the link vertex (the vertex
         * opposite to the splitted edge).
         */
        std::map<size_t, FaceAttributes> faces;
    };
    tbb::enumerable_thread_specific<SplitInfoCache> split_cache;

    struct CollapseInfoCache
    {
        size_t v1_id;
        size_t v2_id;
        double max_energy;
        double edge_length;
        bool is_limit_length;

        std::vector<std::pair<EdgeAttributes, std::array<size_t, 2>>> changed_edges;
        // all faces incident to the delete vertex (v1) that are on the tracked surface
        std::vector<std::array<size_t, 2>> surface_edges;
        std::vector<size_t> changed_fids;
        std::vector<double> changed_energies;
    };
    tbb::enumerable_thread_specific<CollapseInfoCache> collapse_cache;


    struct SwapInfoCache
    {
        double max_energy;
        std::map<simplex::Edge, EdgeAttributes> changed_edges;
        std::vector<int64_t> face_tags;
    };
    tbb::enumerable_thread_specific<SwapInfoCache> swap_cache;

    // When set, split_edge_after binary-searches vmid onto the zero-crossing of this function.
    // Negative = stays on v1 side, positive = stays on v2 side.
    // Set before split_edge(), cleared immediately after.
    std::function<double(const Vector2d&)> m_voronoi_split_fn = nullptr;
};

} // namespace wmtk::components::image_simulation::tri
