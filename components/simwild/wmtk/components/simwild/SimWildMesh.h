#pragma once

#include <igl/Timer.h>
#include <wmtk/SurfaceTagAttributes.h>
#include <wmtk/TetMesh.h>
#include <wmtk/TetOptimizerMesh.h>
#include <wmtk/utils/PartitionMesh.h>
#include <polysolve/nonlinear/Problem.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include <wmtk/threading/parallel_for.hpp>

#include "ConnectedComponent.hpp"
#include "Parameters.h"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <fastenvelope/FastEnvelope.h>
#include <VolumeRemesher/embed.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <igl/remove_unreferenced.h>
#include <atomic>
#include <memory>

#include <wmtk/utils/SurfaceTopology.hpp>
#include <wmtk/utils/partition_utils.hpp>
#include "expression_parser/Expression.hpp"

namespace wmtk::components::simwild {

/// The shared attribute types live on the base; keep the unqualified names working. The
/// simwild copies were identical apart from tetwild's m_is_on_open_boundary, which simwild
/// never sets.
using VertexAttributes = wmtk::TetOptimizerMesh::VertexAttributes;
using FaceAttributes = wmtk::TetOptimizerMesh::FaceAttributes;

class TetAttributes
{
public:
    /**
     * cubed (!) AMIPS quality
     */
    double m_quality;
    /**
     * All image labels. Stored as pairs of image ID and the tag within the image. Using a sparse
     * vector, so 0 entries are ommitted.
     */
    CellTag tags;
};

class SimWildMesh : public wmtk::TetOptimizerMesh
{
public:
    using ExprPtr = expression_parser::ExpressionPtr;

    size_t m_tags_count = 0;
    std::map<int64_t, std::string> m_tag_id_to_name;
    std::map<std::string, int64_t> m_tag_name_to_id;

    /// The base holds only wmtk::OptimizerParameters; this is the same object, typed, for the
    /// simwild-only fields (tags, quality/sizing fields, the operation name).
    Parameters& m_sim_params;
    std::vector<Vector3d> m_V_envelope;
    std::vector<Vector3i> m_F_envelope;
    double m_envelope_eps = -1;

    std::vector<std::tuple<ExprPtr, double>> m_sizing_field;
    std::vector<std::tuple<ExprPtr, double>> m_quality_field;

    bool m_collapse_check_quality = true;

    // for open boundary
    /// Follows m_envelope's use_exact; see where it is built in VolumemesherInsertion.cpp.
    std::shared_ptr<SampleEnvelope> m_order_2_edge_envelope;

    /// Envelope a vertex is pulled toward while smoothing.
    std::shared_ptr<SampleEnvelope> smoothing_energy_envelope(const size_t vid) const override;

    // When set, split_edge_after binary-searches vmid onto the zero-crossing of this function.
    // Negative = stays on v1 side, positive = stays on v2 side.
    std::function<double(const Vector3d&)> m_voronoi_split_fn = nullptr;

    SimWildMesh(Parameters& _m_params, double envelope_eps, int _num_threads = 0)
        : wmtk::TetOptimizerMesh(_m_params, nullptr)
        , m_sim_params(_m_params)
    {
        m_envelope_eps = envelope_eps;
        NUM_THREADS = _num_threads;
        p_tet_attrs = &m_tet_attribute;
        m_collapse_check_link_condition = false;
        m_collapse_check_manifold = false;

        // solver is lazily created on first use

        optimization::deactivate_opt_logger();

        m_s_envelope = 1. / (m_params.eps * m_params.eps);

        double& wa = m_params.w_amips;
        double& we = m_params.w_envelope;
        we = 1 - wa;
        logger().info("w_envelope = {}", we);
    }

    ~SimWildMesh() {}
    using TetAttCol = wmtk::AttributeCollection<TetAttributes>;
    TetAttCol m_tet_attribute;

    double cell_quality(const size_t tid) const override { return m_tet_attribute[tid].m_quality; }
    void set_cell_quality(const size_t tid, const double q) override
    {
        m_tet_attribute[tid].m_quality = q;
    }
    bool allow_surface_swap() const override { return m_sim_params.allow_surface_swap; }
    bool check_surface_topology() const override { return m_sim_params.check_surface_topology; }

    // only used with unit tests
    void create_mesh_attributes(
        const std::vector<VertexAttributes>& _vertex_attribute,
        const std::vector<TetAttributes>& _tet_attribute)
    {
        const size_t n_tet = _tet_attribute.size();
        m_vertex_attribute.resize(_vertex_attribute.size());
        m_face_attribute.resize(4 * n_tet);
        m_tet_attribute.resize(n_tet);

        for (size_t i = 0; i < _vertex_attribute.size(); i++)
            m_vertex_attribute[i] = _vertex_attribute[i];

        // Keep whatever init() reserved. AttributeCollection::resize only ever grows, so the
        // calls above cannot shrink a collection -- but assigning m_attributes directly does,
        // and it used to drop the tet attributes to exactly n_tet. The attributes have to stay
        // at least as large as the connectivity: an operation that creates a new element
        // indexes them by its id, and a 5->6 swap creates one. tetwild hit the same thing (see
        // the note on its copy); there the overrun wrote a double past the end and libc++ let
        // it pass, here it assigns a std::set and segfaults outright.
        const size_t tcap = std::max(n_tet, m_tet_attribute.size());
        m_tet_attribute.m_attributes = std::vector<TetAttributes>(tcap);
        for (size_t i = 0; i < n_tet; i++) m_tet_attribute[i] = _tet_attribute[i];
        for (size_t i = 0; i < n_tet; i++)
            m_tet_attribute[i].m_quality = get_quality(tuple_from_tet(i));
    }

    // TODO This should not be here but inside wmtk
    // TODO This should not be here but inside wmtk
    void init_envelope(const MatrixXd& V, const MatrixXi& F, const bool use_exact);

    CellTag string_set_to_cell_tag(const std::set<std::string>& str_set);

    void set_sizing_field(const nlohmann::json& sizing_field_json);

    void set_quality_field(const nlohmann::json& quality_field_json);

    double target_quality(const size_t tid) const;
    double target_quality(const Tuple& t) const;
    double quality_rel(const size_t tid) const;
    double quality_rel(const Tuple& t) const;
    bool check_mesh_quality(double& max_rel_quality, const bool verbose = false) const;
    std::vector<size_t> active_vertices() const override;


    ////// Attributes related

    void write_msh(std::string file, const bool write_envelope = true);

public:
    void simplify();

    /**
     * @brief Check if all vertices of the mesh are rounded.
     *
     */
    bool all_rounded() const;

    //
    //
    // debug use
    std::atomic<int> cnt_split = 0, cnt_collapse = 0;

protected:
    std::tuple<double, double> optimization_quality_stats() override;
    double optimization_stop_metric() const override { return 1.; }
    bool optimization_stop_at_float() const override { return m_sim_params.stop_at_float; }
    void write_optimization_debug_output(const std::string& path) override { write_vtu(path); }

    bool collapse_before_vertex(size_t v1, size_t v2, double edge_length) override;
    bool collapse_quality_allowed(size_t v1, double quality, double ring_max) const override;
    bool collapse_is_order_2_edge(const std::array<size_t, 2>& e) override;
    bool collapse_after_connectivity(
        size_t v1,
        size_t v2,
        const std::vector<std::array<size_t, 2>>& boundary_edges) override;
    void collapse_after_vertex(size_t v1, size_t v2) override;

    bool split_before_cells(const Tuple& edge, const std::vector<Tuple>& parents) override;
    bool split_after_cells(size_t v1, size_t v2, size_t v_new, const std::vector<Tuple>& children)
        override;
    bool split_adjust_position(size_t v_new, const std::vector<Tuple>& children) override;

    bool swap_before_interior(const std::vector<size_t>& tids) override;
    bool swap_before_surface(
        const std::vector<size_t>& tids,
        size_t a,
        size_t b,
        size_t c,
        size_t d) override;
    bool swap_after_cells(const std::vector<size_t>& tids, bool is_surface_flip) override;

private:
    ////// Operations

    struct SplitTagCache
    {
        size_t v_new = 0;
        /// Parent cell data, keyed by the edge opposite the split edge.
        std::map<simplex::Edge, TetAttributes> tets;
    };
    wmtk::threading::enumerable_thread_specific<SplitTagCache> split_tag_cache;

    struct SwapTagCache
    {
        CellTag tet_tags;
        std::map<size_t, CellTag> ring_tags;
    };
    wmtk::threading::enumerable_thread_specific<SwapTagCache> swap_tag_cache;

public:
    /**
     * @brief Init from meshes image.
     *
     * @param V #Vx3 vertices of the tet mesh
     * @param T #Tx4 vertex IDs for all tets
     * @param T_tags #Tx1 image data represented by the individual tets
     */
    void init_from_image(
        const MatrixXr& V,
        const MatrixXi& T,
        const MatrixSi& T_tags,
        const std::vector<std::string>& tag_names);
    void init_from_image(
        const MatrixXd& V,
        const MatrixXi& T,
        const MatrixSi& T_tags,
        const std::vector<std::string>& tag_names);

    void init_surfaces_and_boundaries();


    /**
     * @brief Escape a stuck max energy by refining the sizing field around the
     * worst elements.
     *
     * Finds the m_params.stuck_refine_num_worst tets with the highest energy,
     * gathers all vertices within m_params.stuck_refine_rings graph rings of
     * them, and multiplies each such vertex's m_sizing_scalar by
     * m_params.stuck_refine_factor (clamped at m_params.stuck_refine_min_scalar).
     * Then runs gradation_smooth_sizing so the refined region blends smoothly
     * into the surrounding resolution. Replaces the old global
     * adjust_sizing_field mechanism. Returns the number of vertices refined.
     */
    size_t refine_sizing_around_worst(double max_metric = 0.) override;


    /**
     * @brief Find open boundary edges of the embedded surface and initialize a BVH for the open
     * boundary.
     *
     * The envelope for the open boundary uses a hack: A boundary edge is represented as a
     * degenerate triangle, e.g., (v0,v1,v0). That way, the standard triangle envelope code can be
     * used.
     *
     */
    void find_order_2_edges();
    /**
     * @brief Checks if an edge COULD be an open boundary edge.
     *
     * The method performs two checks. First, it checks if the two vertices are marked as on the
     * open boundary. Second, it checks if the edge is within the open boundary envelope.
     * Note that these checks are not sufficient to guarantee that an edge is actually on the open
     * boundary! For example, an almost degenerate triangle with two edges on the open boundary
     * could cause a false positive result.
     */
    bool is_order_2_edge(const Tuple& e) const;
    bool is_order_2_edge(const std::array<size_t, 2>& e) const;

    void write_vtu(const std::string& path);

    void write_surface(const std::string& path) const;

public:
    // substructure functions

    /**
     * @brief Compute the vertex order for every vertex.
     */
    void init_vertex_order();

public:
    // Annotations

    double tet_volume(const size_t tid) const;

    /**
     * @brief Find all connected components that contain the `tag_in` tags.
     */
    std::vector<ConnectedComponent> compute_connected_components(const CellTag& tag_in) const;
    std::vector<ConnectedComponent> compute_connected_components(const ExprPtr& expr) const;

    /**
     * @brief Find all regions that do not contain the tags from `tag_in`.
     *
     * The `tag_in` vector represents a list of tag intersections.
     * Example: tag_in = {{1,2},{3}}
     * A face will be considered as a hole if its tags neither include {1,2} or {3}.
     * The following would be holes:
     * {}
     * {1,4}
     * The following would be NOT holes:
     * {1,2,4} <- contains 1 and 2
     * {3} <- contains 3
     * {1,3} <- contains 3
     *
     * The returned vector also contains "holes" that touch the boundary. They should be
     * ommitted in hole filling.
     */
    std::vector<ConnectedComponent> find_holes(const std::vector<CellTag>& tag_in) const;

    /**
     * @brief Compute the boundary of a tag.
     *
     * @param tag A set of tags that must be present in a tet for being considered as
     * tagged.
     * @param V Vertices of the tag boundary.
     * @param F Faces of the tag boundary.
     */
    void compute_tag_boundary(const CellTag& tag, MatrixXd& V, MatrixXi& F) const;

    /**
     * @brief Keep only the largest connected component for each of the distinct tag_0 values,
     * and engulf all other components.
     *
     * @param lcc_tags
     * @param n_lcc The number of largest components that should be kept.
     */
    void keep_largest_connected_component(
        const std::vector<CellTag>& lcc_tags,
        const size_t n_lcc = 1);

    void fill_holes_topo(
        const std::vector<CellTag>& fill_holes_tags,
        double threshold = std::numeric_limits<double>::infinity());

    void seal_connected_components(
        const std::vector<CellTag>& tag_sets,
        const std::vector<ConnectedComponent>& components);

    void tight_seal_topo(
        const std::vector<std::vector<CellTag>>& tight_seal_tag_sets,
        double threshold = std::numeric_limits<double>::infinity());

    void resolve_overlaps(const std::vector<std::array<ExprPtr, 2>>& intersecting_tags);

    void replace_tags(const std::vector<CellTag>& tags_in, const std::vector<CellTag>& tags_out);

    /**
     * @brief Gives tags priority over others.
     *
     * If a tet has multiple tags, only the one with the
     * highest priority will be kept. The priority is determined by the order of the tags in the
     * input vector, e.g., if tag A is before tag B in the vector, then A has higher priority than
     * B.
     *
     * @param tags A vector of tags, where the order determines the priority.
     */
    void tag_priority(const std::vector<int64_t>& tags);
};

} // namespace wmtk::components::simwild
