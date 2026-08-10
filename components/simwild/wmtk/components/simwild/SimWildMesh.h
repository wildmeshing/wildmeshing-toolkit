#pragma once

#include <igl/Timer.h>
#include <wmtk/TetMesh.h>
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

class VertexAttributes
{
public:
    Vector3r m_pos; // exact position in rational
    Vector3d m_posf; // position as double
    /**
     * If a vertex cannot be rounded without inverting a tet, the exact position must be used. Once
     * the vertex can be rounded to double precision, the rational representation is obsolete.
     */
    bool m_is_rounded = false;

    bool m_is_on_surface = false;
    /**
     * The order of a vertex in a TetMesh is as follows:
     * 0: vertex is not on the surface
     * 1: vertex is on the surface
     * 2: vertex is on the boundary of the surface or a non-manifold edge
     * 3: vertex is at the boundary of a non-manifold edge or a non-manifold vertex
     */
    size_t m_order = 0;
    std::vector<int> on_bbox_faces; // same as is_bbox_fs?

    double m_sizing_scalar = 1;

    /**
     * Required for multi-threading.
     */
    size_t partition_id = 0;

    VertexAttributes() {};
    VertexAttributes(const Vector3r& p);
};


// class EdgeAttributes
// {
// public:
//     bool m_is_on_open_boundary = false;
// };

class FaceAttributes
{
public:
    /**
     * Is this face a part of the surface.
     */
    bool m_is_surface_fs = false;

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

    void merge(const FaceAttributes& attr)
    {
        m_is_surface_fs = m_is_surface_fs || attr.m_is_surface_fs;
        if (attr.m_is_bbox_fs >= 0) m_is_bbox_fs = attr.m_is_bbox_fs;
    }
};

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

class SimWildMesh : public wmtk::TetMesh
{
public:
    using ExprPtr = expression_parser::ExpressionPtr;

    int m_debug_print_counter = 0;
    size_t m_tags_count = 0;
    std::map<int64_t, std::string> m_tag_id_to_name;
    std::map<std::string, int64_t> m_tag_name_to_id;

    double time_env = 0.0;
    igl::Timer isout_timer;
    /**
     * @brief The sentinel get_quality returns for an element AMIPS cannot score.
     *
     * Not an energy: a positively oriented tet whose volume is too small for AMIPS, or one
     * that produces inf/nan, gets this instead. m_quality holds AMIPS^3, so it surfaces in
     * the logs as its cube root, cbrt(1e50) = 4.6e16.
     *
     * 1e50 rather than double::max, matching tetwild and triwild, because every downstream
     * arithmetic on it must stay finite -- avg_energy sums qualities, so a single degenerate
     * tet turned the reported average into inf, and every ratio that divides by the max was
     * meaningless from then on. 1e50 leaves headroom for all of it.
     */
    const double MAX_ENERGY = 1e50;

    Parameters& m_params;
    std::vector<Vector3d> m_V_envelope;
    std::vector<Vector3i> m_F_envelope;
    std::shared_ptr<SampleEnvelope> m_envelope;
    std::shared_ptr<SampleEnvelope> m_envelope_orig;
    double m_envelope_eps = -1;

    std::vector<std::tuple<ExprPtr, double>> m_sizing_field;
    std::vector<std::tuple<ExprPtr, double>> m_quality_field;

    bool m_collapse_check_quality = true;

    // for open boundary
    /// Follows m_envelope's use_exact; see where it is built in VolumemesherInsertion.cpp.
    std::shared_ptr<SampleEnvelope> m_order_2_edge_envelope;

    wmtk::threading::enumerable_thread_specific<std::unique_ptr<polysolve::nonlinear::Solver>>
        m_solver;

    // scaling factors
    double m_s_amips = -1;
    double m_s_envelope = -1;

    /// Why smoothing attempts were refused, reported once per pass.
    optimization::SmoothRejectCounters m_smooth_rejects;

    /// Envelope a vertex is pulled toward while smoothing.
    std::shared_ptr<SampleEnvelope> smoothing_energy_envelope(const size_t vid) const;
    /// Envelope the resulting surface triangles are checked against.
    std::shared_ptr<SampleEnvelope> smoothing_containment_envelope(const size_t vid) const;

    /// No 0-dimensional features here, so smoothing is never positionally constrained beyond
    /// the envelope. See TriWildMesh::smoothing_position_is_allowed for the case that is.
    bool smoothing_position_is_allowed(const size_t, const Vector2d&) const { return true; }

    // When set, split_edge_after binary-searches vmid onto the zero-crossing of this function.
    // Negative = stays on v1 side, positive = stays on v2 side.
    std::function<double(const Vector3d&)> m_voronoi_split_fn = nullptr;

    SimWildMesh(Parameters& _m_params, double envelope_eps, int _num_threads = 0)
        : m_params(_m_params)
        , m_envelope_eps(envelope_eps)
    {
        NUM_THREADS = _num_threads;
        p_vertex_attrs = &m_vertex_attribute;
        p_face_attrs = &m_face_attribute;
        p_tet_attrs = &m_tet_attribute;
        m_collapse_check_link_condition = false;
        m_collapse_check_manifold = false;

        // solver is lazily created on first use

        optimization::deactivate_opt_logger();

        m_s_amips = 1.;
        m_s_envelope = 1. / (m_params.diag_l * m_params.eps * m_params.eps);

        double& wa = m_params.w_amips;
        double& we = m_params.w_envelope;
        we = 1 - wa;
        logger().info("w_envelope = {}", we);
    }

    ~SimWildMesh() {}
    using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    using FaceAttCol = wmtk::AttributeCollection<FaceAttributes>;
    using TetAttCol = wmtk::AttributeCollection<TetAttributes>;
    // using EdgeAttCol = wmtk::AttributeCollection<EdgeAttributes>;
    VertAttCol m_vertex_attribute;
    FaceAttCol m_face_attribute;
    TetAttCol m_tet_attribute;
    // EdgeAttCol m_edge_attribute;

    // only used with unit tests
    void create_mesh_attributes(
        const std::vector<VertexAttributes>& _vertex_attribute,
        const std::vector<TetAttributes>& _tet_attribute)
    {
        auto n_tet = _tet_attribute.size();
        m_vertex_attribute.resize(_vertex_attribute.size());
        m_face_attribute.resize(4 * n_tet);
        m_tet_attribute.resize(n_tet);

        for (auto i = 0; i < _vertex_attribute.size(); i++) {
            m_vertex_attribute[i] = _vertex_attribute[i];
        }
        m_tet_attribute.m_attributes = std::vector<TetAttributes>(_tet_attribute.size());
        for (auto i = 0; i < _tet_attribute.size(); i++) {
            m_tet_attribute[i] = _tet_attribute[i];
        }
        for (auto i = 0; i < _tet_attribute.size(); i++) {
            m_tet_attribute[i].m_quality = get_quality(tuple_from_tet(i));
        }
    }

    // TODO This should not be here but inside wmtk
    void compute_vertex_partition()
    {
        auto partition_id = partition_TetMesh(*this, NUM_THREADS);
        for (auto i = 0; i < vert_capacity(); i++)
            m_vertex_attribute[i].partition_id = partition_id[i];
    }

    // TODO This should not be here but inside wmtk
    void compute_vertex_partition_morton()
    {
        if (NUM_THREADS == 0) {
            return;
        }

        logger().info("Number of parts: {} by morton", NUM_THREADS);

        std::vector<size_t> partition_id;
        wmtk::partition_vertex_morton(
            vert_capacity(),
            [this](size_t i) { return m_vertex_attribute[i].m_posf; },
            NUM_THREADS,
            partition_id);

        for (size_t i = 0; i < partition_id.size(); i++) {
            m_vertex_attribute[i].partition_id = partition_id[i];
        }
    }

    size_t get_partition_id(const Tuple& loc) const
    {
        return m_vertex_attribute[loc.vid(*this)].partition_id;
    }

    void init_envelope(const MatrixXd& V, const MatrixXi& F, const bool use_exact);

    CellTag string_set_to_cell_tag(const std::set<std::string>& str_set);

    void set_sizing_field(const nlohmann::json& sizing_field_json);

    void set_quality_field(const nlohmann::json& quality_field_json);

    double target_quality(const size_t tid) const;
    double target_quality(const Tuple& t) const;
    double quality_rel(const size_t tid) const;
    double quality_rel(const Tuple& t) const;
    bool check_mesh_quality(double& max_rel_quality, const bool verbose = false) const;

    double get_length2(const Tuple& l) const;

    ////// Attributes related

    void write_msh(std::string file, const bool write_envelope = true);
    void output_faces(std::string file, std::function<bool(const FaceAttributes&)> cond);

public:
    void split_all_edges();
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& loc) override;

    void smooth_all_vertices(const size_t n_iters);
    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;

    void collapse_all_edges(bool is_limit_length = true);
    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;

    void simplify();

    size_t swap_all_edges_44();
    bool swap_edge_44_before(const Tuple& t) override;
    double swap_edge_44_energy(const std::vector<std::array<size_t, 4>>& tets, const int op_case)
        override;
    bool swap_edge_44_after(const Tuple& t) override;

    size_t swap_all_edges_56();
    bool swap_edge_56_before(const Tuple& t) override;
    double swap_edge_56_energy(const std::vector<std::array<size_t, 4>>& tets, const int op_case)
        override;
    bool swap_edge_56_after(const Tuple& t) override;

    size_t swap_all_edges_32();
    bool swap_edge_before(const Tuple& t) override;
    bool swap_edge_after(const Tuple& t) override;

    /**
     * @brief Prepare a surface 3->2 edge swap (a surface diagonal flip).
     *
     * Called from swap_edge_before when the swapped edge (a,b) is on the surface
     * and has exactly 3 incident tets. Verifies the local guards that guarantee
     * the flip preserves surface manifoldness / topology, and fills the
     * surface-flip fields of swap_cache. Returns false (rejecting the swap) if
     * any guard fails: open-boundary edge, non-manifold edge (!= 2 surface
     * faces), or one of the two would-be new surface faces already tagged
     * surface. The tets sharing (a,b) are passed in to avoid recomputation.
     */
    bool prepare_surface_flip_32(const Tuple& t, const std::vector<size_t>& incident_tets);

    /// A topological fingerprint of the tracked surface (m_is_surface_fs). See
    /// wmtk/utils/SurfaceTopology.hpp.
    using SurfaceTopoSignature = wmtk::utils::SurfaceTopoSignature;

    SurfaceTopoSignature surface_topology_signature() const
    {
        return wmtk::utils::surface_topology_signature(*this, [this](size_t fid) {
            return m_face_attribute[fid].m_is_surface_fs;
        });
    }

    /**
     * @brief Compare a surface signature against the current one and log an
     * error if it changed. Used (when m_params.check_surface_topology is set) to
     * guard swap passes that can flip surface edges.
     */
    void warn_if_surface_topology_changed(const SurfaceTopoSignature& before, const char* where)
        const
    {
        wmtk::utils::warn_if_surface_topology_changed(before, surface_topology_signature(), where);
    }

    size_t swap_all_faces();
    bool swap_face_before(const Tuple& t) override;
    bool swap_face_after(const Tuple& t) override;

    size_t swap_all_edges_all();

    /**
     * @brief m_quality threshold above which a tet is "active" (worth operating
     * on) for the skip-good-regions filter. m_quality stores AMIPS^3 and the
     * energy is its cube root, so a tet is active when its energy is at least
     * skip_good_regions_margin * stop_energy, i.e. m_quality >=
     * (margin * stop_energy)^3.
     */
    double active_quality_threshold() const
    {
        const double e = m_params.skip_good_regions_margin * m_params.stop_energy;
        return e * e * e;
    }

    /**
     * @brief vids of the vertices incident to at least one "active" tet
     * (m_quality >= active_quality_threshold()). Used by the skip-good-regions
     * filter to restrict smoothing to non-good regions (smoothing a vertex
     * surrounded by good tets does nothing).
     */
    std::vector<size_t> active_vertices() const;

    /**
     * @brief Inversion check using only floating point numbers.
     */
    bool is_inverted_f(const Tuple& loc) const;
    bool is_inverted(const std::array<size_t, 4>& vs) const;
    bool is_inverted(const Tuple& loc) const;
    double get_quality(const std::array<size_t, 4>& vs) const;
    double get_quality(const Tuple& loc) const;

    /**
     * @brief Round a vertex position to floating point.
     *
     * Only rounds the vertex position, if it does not cause inverted elements.
     *
     * @return True if successful or already rounded, false otherwise.
     */
    bool round(const Tuple& loc);

    /**
     * @brief Try to round every un-rounded vertex; returns the number reclaimed.
     *
     * round() is otherwise only attempted as a side effect of another operation
     * (smooth_before on the vertex being smoothed, collapse_edge_after on the merged one),
     * and neither reaches a vertex that only becomes roundable later: smoothing skips
     * "good" regions by default. Without a sweep such a vertex keeps exact coordinates
     * into the output for no geometric reason.
     *
     * Skipped outright when m_all_rounded says there is nothing to do.
     */
    size_t round_all_vertices();

    /**
     * @brief Run the rounding sweep, then report whether the mesh is now fully rounded.
     *
     * The termination condition of the operation loop. Energy alone is not sufficient: a mesh
     * that hits the quality target while some vertex still carries exact coordinates is not
     * finished, because the output is what the caller consumes and rational coordinates in it
     * are a defect regardless of how good the elements are.
     *
     * This is also what makes the exact-rational fallback in split_edge_after safe. A split is
     * the only operation that can un-round a vertex; collapse, the swaps and smoothing never
     * do; and the post-optimization pass is collapse-only.
     *
     * O(1) once m_all_rounded is set, so it is cheap enough to sit on every early-out.
     */
    bool round_and_check_all_rounded();

    /**
     * @brief True when every vertex is known to be rounded.
     *
     * Only trusted when true, and only round_all_vertices() sets it that way. Any code that
     * leaves a vertex un-rounded must clear it, or the sweep will skip the vertex forever.
     * Atomic because operations that clear it run in parallel.
     *
     * Distinct from all_rounded(), which counts. This is the cheap "is there anything to
     * do" flag; that is the answer.
     */
    std::atomic<bool> m_all_rounded = false;

    /**
     * @brief Check if all vertices of the mesh are rounded.
     *
     */
    bool all_rounded() const;

    //
    bool is_edge_on_surface(const Tuple& loc);
    bool is_edge_on_bbox(const Tuple& loc);
    //
    void mesh_improvement(int max_its = 80);
    double local_operations(const std::array<int, 4>& ops, bool collapse_limit_length = true);
    std::tuple<double, double> get_max_avg_energy();

    std::vector<std::array<size_t, 3>> get_faces_by_condition(
        std::function<bool(const FaceAttributes&)> cond) const;

    bool invariants(const std::vector<Tuple>& t) override; // this is now automatically checked

    // debug use
    std::atomic<int> cnt_split = 0, cnt_collapse = 0, cnt_swap = 0;
    // Successful surface diagonal flips (subset of cnt_swap). Diagnostic.
    std::atomic<int> cnt_surface_swap = 0;

private:
    ////// Operations

    struct SplitInfoCache
    {
        //        VertexAttributes vertex_info;
        size_t v_new;
        size_t v1_id;
        size_t v2_id;
        bool is_edge_on_surface = false;
        bool is_edge_open_boundary = false;
        /// Worst quality among the elements incident to the edge BEFORE the split, so
        /// split_edge_after can tell "this split created a degenerate element" from "this
        /// split subdivided a region that was already degenerate".
        double max_quality_before = 0.;

        std::vector<std::pair<FaceAttributes, std::array<size_t, 3>>> changed_faces;

        /**
         * All tets incident to the splitted edge, identified by the link edge (the edge opposite to
         * the splitted one).
         */
        std::map<simplex::Edge, TetAttributes> tets;
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

        std::vector<std::pair<FaceAttributes, std::array<size_t, 3>>> changed_faces;
        // all faces incident to the delete vertex (v1) that are on the tracked surface
        std::vector<std::array<size_t, 3>> surface_faces;
        // all edges incident to the deleted vertex(v1) that are on the open boundary
        std::vector<std::array<size_t, 2>> boundary_edges;
        std::vector<size_t> changed_tids;
        std::vector<double> changed_energies;
    };
    wmtk::threading::enumerable_thread_specific<CollapseInfoCache> collapse_cache;


    struct SwapInfoCache
    {
        double max_energy;
        std::map<std::array<size_t, 3>, FaceAttributes> changed_faces;
        CellTag tet_tags;

        // Surface 3->2 flip bookkeeping (filled by swap_edge_before when the
        // swapped edge (a,b) lies on the surface). a,b are the removed-edge
        // endpoints, c,d are the new surface-edge endpoints, e is the interior
        // apex. sf_face_attr is copied onto the two new surface faces (a,c,d),
        // (b,c,d). is_surface_flip gates the extra handling in swap_edge_after.
        bool is_surface_flip = false;
        size_t sf_a = 0, sf_b = 0, sf_c = 0, sf_d = 0, sf_e = 0;
        FaceAttributes sf_face_attr;
    };
    wmtk::threading::enumerable_thread_specific<SwapInfoCache> swap_cache;

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


    /// The longest edge of each current worst tet (as a sorted {min,max} vid pair).
    /// split_all_edges force-splits exactly these edges (bypasses the length gate),
    /// so a stuck sliver's long edge is split immediately without changing the sizing
    /// field. Populated serially by refine_sizing_around_worst; read-only during the
    /// parallel split pass, then cleared once split_all_edges has consumed it.
    std::set<simplex::Edge> m_force_split_edges;

    /// Count of force-splits taken in the current split pass (atomic_ref from the
    /// parallel split; reset + logged by split_all_edges). Diagnostic only.
    size_t m_force_split_count = 0;
    /**
     * @brief Splits in the last pass that fell back to the exact rational midpoint.
     *
     * A split is the only operation that can un-round a vertex, so this is exactly how often
     * the mesh acquired exact coordinates during optimization -- the counterpart of the
     * "rounded n/m" line, which says how many it still carries. Non-zero on real inputs: on
     * the challenging-model set it ranges from 1 to 44 per run.
     */
    size_t m_exact_split_count = 0;

    /// True iff edge (v1,v2) is a worst tet's longest edge queued for force-split.
    bool is_force_split_edge(size_t v1, size_t v2) const
    {
        return m_force_split_edges.find(simplex::Edge(v1, v2)) != m_force_split_edges.end();
    }

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

    bool vertex_is_on_surface(const size_t vid) const override;

    bool face_is_on_surface(const size_t fid) const override;

    size_t get_order_of_vertex(const size_t vid) const override;
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
