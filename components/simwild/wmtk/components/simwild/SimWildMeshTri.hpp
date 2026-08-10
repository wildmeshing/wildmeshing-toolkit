#pragma once

#include <limits>
#include <unordered_set>

#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <polysolve/nonlinear/Problem.hpp>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/optimization/solver.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <igl/write_triangle_mesh.h>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include <fastenvelope/FastEnvelope.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include "ConnectedComponent.hpp"
#include "Parameters.h"
#include "expression_parser/Expression.hpp"


namespace wmtk::components::simwild::tri {

struct VertexAttributes
{
    Vector2d m_posf; // position as double
    Vector2r m_pos; // exact position in rational
    /**
     * If a vertex cannot be rounded without inverting an incident face, the exact position
     * must be used. Once the vertex can be rounded to double precision, the rational
     * representation is obsolete.
     */
    bool m_is_rounded = false;

    bool m_is_on_surface = false;
    std::vector<int> on_bbox_faces;

    double m_sizing_scalar = 1;

    size_t partition_id = 0;

    VertexAttributes() {}
    VertexAttributes(const Vector2d& p)
        : m_posf(p)
        , m_pos(to_rational(p))
        , m_is_rounded(true)
    {}
    VertexAttributes(const Vector2r& p)
        : m_posf(to_double(p))
        , m_pos(p)
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
    CellTag tags;
    int part_id = -1;
};

class SimWildMeshTri : public wmtk::TriMesh
{
public:
    using ExprPtr = expression_parser::ExpressionPtr;

    int m_debug_print_counter = 0;
    size_t m_tags_count = 0;
    std::map<int64_t, std::string> m_tag_id_to_name;
    std::map<std::string, int64_t> m_tag_name_to_id;

    /**
     * @brief The sentinel get_quality returns for a face AMIPS2D cannot score.
     *
     * Not an energy: a positively oriented triangle whose area is too small for AMIPS, or one
     * that produces inf/nan, gets this instead. Unlike the 3D mesh this stores AMIPS2D
     * directly, so it surfaces in the logs verbatim as 1e+50.
     *
     * 1e50 rather than double::max, matching tetwild and triwild -- see SimWildMesh::MAX_ENERGY
     * for why the arithmetic headroom matters.
     */
    const double MAX_ENERGY = 1e50;

    Parameters& m_params;
    std::vector<Vector2d> m_V_envelope;
    std::vector<Vector2i> m_E_envelope;
    std::shared_ptr<SampleEnvelope> m_envelope;
    std::shared_ptr<SampleEnvelope> m_envelope_orig;
    double m_envelope_eps = -1;

    std::vector<std::tuple<ExprPtr, double>> m_sizing_field;
    std::vector<std::tuple<ExprPtr, double>> m_quality_field;

    using VertAttCol = AttributeCollection<VertexAttributes>;
    using EdgeAttCol = AttributeCollection<EdgeAttributes>;
    using FaceAttCol = AttributeCollection<FaceAttributes>;
    VertAttCol m_vertex_attribute;
    EdgeAttCol m_edge_attribute;
    FaceAttCol m_face_attribute;

    bool m_collapse_check_link_condition = false; // classical link condition
    bool m_collapse_check_topology = false; // sanity check
    bool m_collapse_check_manifold = false; // manifoldness check after collapse

    wmtk::threading::enumerable_thread_specific<std::unique_ptr<polysolve::nonlinear::Solver>>
        m_solver;

    /// Why smoothing attempts were refused, reported once per pass.
    optimization::SmoothRejectCounters m_smooth_rejects;

    /// Hooks for the shared 2D smoothing driver.
    Vector2d smoothing_position(const size_t vid) const;
    void set_smoothing_position(const size_t vid, const Vector2d& p);
    bool is_inverted_f(const size_t fid) const;
    std::shared_ptr<SampleEnvelope> smoothing_energy_envelope(const size_t vid) const;
    std::shared_ptr<SampleEnvelope> smoothing_containment_envelope(const size_t vid) const;

    /// No 0-dimensional features here, so smoothing is never positionally constrained beyond
    /// the envelope. See TriWildMesh::smoothing_position_is_allowed for the case that is.
    bool smoothing_position_is_allowed(const size_t, const Vector2d&) const { return true; }

    // scaling factors
    double m_s_amips = -1;
    double m_s_envelope = -1;

    SimWildMeshTri(Parameters& _m_params, double envelope_eps, int _num_threads = 0)
        : m_params(_m_params)
        , m_envelope_eps(envelope_eps)
    {
        NUM_THREADS = _num_threads;
        p_vertex_attrs = &m_vertex_attribute;
        p_edge_attrs = &m_edge_attribute;
        p_face_attrs = &m_face_attribute;

        optimization::deactivate_opt_logger();

        m_s_amips = 1.;
        /**
         * eps makes it such that the energy is relative to the envelope thickness. As it's a
         * squared energy, we need eps^2.
         */
        m_s_envelope = 1. / (m_params.eps * m_params.eps);


        double& wa = m_params.w_amips;
        double& we = m_params.w_envelope;
        we = 1 - wa;
        logger().info("w_envelope = {}", we);
    }

    ~SimWildMeshTri() {}

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
     * @param tag_names Names for each tag in T_tags. The size must be the same as the number of
     * columns in T_tags.
     */
    void init_from_image(
        const MatrixXd& V,
        const MatrixXi& T,
        const MatrixSi& T_tags,
        const std::vector<std::string>& tag_names);

    /**
     * @brief Same, from EXACT input positions.
     *
     * Taken when the 2D arrangement produced a vertex with no double representation -- a
     * crossing between two input segments generally has none. Rounds what it can and leaves
     * the rest rational; the optimization loop reclaims them.
     */
    void init_from_image(
        const MatrixXr& V,
        const MatrixXi& T,
        const MatrixSi& T_tags,
        const std::vector<std::string>& tag_names);

    void init_surfaces_and_boundaries();

    void init_envelope(const MatrixXd& V, const MatrixXi& F);

    CellTag string_set_to_cell_tag(const std::set<std::string>& str_set);

    void set_sizing_field(const nlohmann::json& sizing_field_json);

    void set_quality_field(const nlohmann::json& quality_field_json);

    double target_quality(const size_t tid) const;
    double target_quality(const Tuple& t) const;
    double quality_rel(const size_t tid) const;
    double quality_rel(const Tuple& t) const;
    bool check_mesh_quality(double& max_rel_quality, const bool verbose = false) const;

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
    size_t refine_sizing_around_worst();

    /**
     * @brief Monotone (only-decreasing) gradation smoothing of the sizing field.
     *
     * Enforces m_sizing_scalar[v] <= grade * m_sizing_scalar[u] for every edge
     * (u,v), propagating outward from `seeds` with a min-relaxation. It never
     * raises a sizing value, so it only ever spreads more refinement into the
     * halo around already-refined vertices, avoiding sharp resolution jumps.
     */
    void gradation_smooth_sizing(double grade, const std::vector<size_t>& seeds);


    void write_msh(std::string file, const bool write_envelope = true);

    void write_vtu(const std::string& path) const;
    void write_vtu_with_energies(const std::string& path) const;

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

    void smooth_all_vertices(const size_t n_iters = 1);
    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;

    /**
     * @brief A vector containing the vertex position and all positions of the surface neighbors.
     *
     * Returns an empty vector if vertex is not on the surface.
     */
    std::vector<Vector2d> get_surface_assembles(const Tuple& t) const;
    std::shared_ptr<polysolve::nonlinear::Problem> get_envelope_energy(const Tuple& t) const;

    std::vector<std::array<double, 6>> get_amips_assembles(const Tuple& t) const;
    std::shared_ptr<polysolve::nonlinear::Problem> get_amips_energy(const Tuple& t) const;

    /**
     * For debugging purposes.
     */
    void log_total_surface_energy();
    //
    /**
     * @brief Orientation check, exact for the coordinates the vertices actually carry.
     *
     * Takes the floating path when all three vertices are rounded -- igl::predicates::orient2d
     * is exact for the doubles it is handed -- and the Rational cross product otherwise. The
     * rational branch exists because for an un-rounded vertex the double is the wrong number,
     * not because orient2d is imprecise.
     */
    bool is_inverted(const std::array<size_t, 3>& vs) const;
    bool is_inverted(const Tuple& loc) const;
    bool is_inverted(const size_t fid) const;
    double get_quality(const std::array<size_t, 3>& vs) const;
    double get_quality(const Tuple& loc) const;
    double get_quality(const size_t fid) const;

    /**
     * @brief Round a vertex position to floating point.
     *
     * Only rounds the vertex position if it does not invert an incident face.
     *
     * @return True if successful or already rounded, false otherwise.
     */
    bool round(const Tuple& v);

    /**
     * @brief Try to round every un-rounded vertex; returns the number reclaimed.
     *
     * round() is otherwise only attempted as a side effect of another operation, and that
     * never reaches a vertex which only becomes roundable later. Without a sweep such a vertex
     * keeps exact coordinates into the output for no geometric reason.
     *
     * Skipped outright when m_all_rounded says there is nothing to do.
     */
    size_t round_all_vertices();

    /**
     * @brief Run the rounding sweep, then report whether the mesh is now fully rounded.
     *
     * The termination condition of the operation loop, and what makes the exact-rational
     * fallback in split_edge_after safe: a split is the only operation that can un-round a
     * vertex, so the loop only has to outlast the sweep. See SimWildMesh's counterpart.
     */
    bool round_and_check_all_rounded();

    /**
     * @brief True when every vertex is known to be rounded.
     *
     * Only trusted when true, and only round_all_vertices() sets it that way. Any code that
     * leaves a vertex un-rounded must clear it, or the sweep will skip the vertex forever.
     * Atomic because operations that clear it run in parallel.
     */
    std::atomic<bool> m_all_rounded = false;

    /**
     * @brief Splits in the last pass that fell back to the exact rational midpoint.
     *
     * A split is the only operation that can un-round a vertex, so this is exactly how often
     * the mesh acquired exact coordinates during optimization -- the counterpart of the
     * "rounding sweep" line, which says how many were given back.
     */
    size_t m_exact_split_count = 0;

    double triangle_area(const size_t fid) const;

    //
    bool is_edge_on_surface(const Tuple& loc) const;
    bool is_edge_on_surface(const std::array<size_t, 2>& vids) const;
    bool is_edge_on_bbox(const Tuple& loc) const;
    bool is_edge_on_bbox(const std::array<size_t, 2>& vids) const;
    //
    void mesh_improvement(int max_its = 80);
    double local_operations(const std::array<int, 4>& ops, bool collapse_limit_length = true);
    std::tuple<double, double> get_max_avg_energy();

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
     * The returned vector also contains "holes" that touch the boundary. They should be ommitted in
     * hole filling.
     */
    std::vector<ConnectedComponent> find_holes(const std::vector<CellTag>& tag_in) const;

    /**
     * @brief Compute the boundary of a tag.
     *
     * @param tag A set of tags that must be present in a triangle for being considered as tagged.
     * @param V Vertices of the tag boundary.
     * @param E Edges of the tag boundary.
     */
    void compute_tag_boundary(const CellTag& tag, MatrixXd& V, MatrixXi& E) const;

    /**
     * @brief Keep only the largest connected component for each of the distinct tag_0 values, and
     * engulf all other components.
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

    void tag_priority(const std::vector<int64_t>& tags_order);

    bool vertex_is_on_surface(const size_t vid) const override
    {
        return m_vertex_attribute.at(vid).m_is_on_surface ||
               !m_vertex_attribute.at(vid).on_bbox_faces.empty();
    }
    bool edge_is_on_surface(const std::array<size_t, 2>& vids) const override
    {
        if (!vertex_is_on_surface(vids[0]) || !vertex_is_on_surface(vids[1])) {
            return false;
        }

        const auto [_, eid] = tuple_from_edge(vids);
        bool on_surface = m_edge_attribute.at(eid).m_is_surface_fs;
        bool on_bbox = m_edge_attribute.at(eid).m_is_bbox_fs >= 0;
        return on_surface || on_bbox;
    }

private:
    ////// Operations

    struct SplitInfoCache
    {
        //        VertexAttributes vertex_info;
        size_t v_new;
        size_t v1_id;
        size_t v2_id;
        /// Worst quality among the elements incident to the edge BEFORE the split, so
        /// split_edge_after can tell "this split created a degenerate element" from "this
        /// split subdivided a region that was already degenerate".
        double max_quality_before = 0.;

        EdgeAttributes old_e_attrs;

        // std::vector<std::pair<EdgeAttributes, std::array<size_t, 2>>> changed_edges;
        std::map<simplex::Edge, EdgeAttributes> changed_edges;

        /**
         * All faces incident to the splitted edge, identified by the link vertex (the vertex
         * opposite to the splitted edge).
         */
        std::map<size_t, FaceAttributes> faces;
    };
    wmtk::threading::enumerable_thread_specific<SplitInfoCache> split_cache;

    /// Whether the current collapse pass applies the target-length limit; read by
    /// collapse_edge_before, which is where that limit is now enforced.
    bool m_collapse_limit_length = true;

    struct CollapseInfoCache
    {
        size_t v1_id;
        size_t v2_id;
        double max_energy;
        double edge_length;

        std::vector<std::pair<EdgeAttributes, std::array<size_t, 2>>> changed_edges;
        // all faces incident to the delete vertex (v1) that are on the tracked surface
        std::vector<std::array<size_t, 2>> surface_edges;
        std::vector<size_t> changed_fids;
        std::vector<double> changed_energies;
    };
    wmtk::threading::enumerable_thread_specific<CollapseInfoCache> collapse_cache;


    struct SwapInfoCache
    {
        double max_energy;
        std::map<simplex::Edge, EdgeAttributes> changed_edges;
        CellTag face_tags;
    };
    wmtk::threading::enumerable_thread_specific<SwapInfoCache> swap_cache;

    // When set, split_edge_after binary-searches vmid onto the zero-crossing of this function.
    // Negative = stays on v1 side, positive = stays on v2 side.
    // Set before split_edge(), cleared immediately after.
    std::function<double(const Vector2d&)> m_voronoi_split_fn = nullptr;
};

} // namespace wmtk::components::simwild::tri
