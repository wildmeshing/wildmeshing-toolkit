#pragma once

#include <igl/Timer.h>
#include <wmtk/TetMesh.h>
#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/threading/concurrent_map.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include <wmtk/threading/parallel_for.hpp>

#include "Parameters.h"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <fastenvelope/FastEnvelope.h>
#include <VolumeRemesher/embed.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <igl/remove_unreferenced.h>
#include <memory>
#include <set>
#include <unordered_set>
#include <utility>
#include <wmtk/utils/SurfaceTopology.hpp>
#include <wmtk/utils/partition_utils.hpp>

namespace wmtk::components::tetwild {

// TODO: missing comments on what these attributes are
class VertexAttributes
{
public:
    Vector3r m_pos;
    Vector3d m_posf;
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
    std::vector<int> on_bbox_faces;

    double m_sizing_scalar = 1;

    size_t partition_id = 0;

    // for open boundary
    bool m_is_on_open_boundary = false;

    VertexAttributes() {};
    VertexAttributes(const Vector3r& p);
};


// class EdgeAttributes
// {
// public:
//     bool m_is_on_open_boundary = false;
// };

// TODO: missing comments on what these attributes are
class FaceAttributes
{
public:
    bool m_is_surface_fs = false; // 0; 1
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

// TODO: missing comments on what these attributes are
class TetAttributes
{
public:
    double m_quality;
    double m_winding_number_input = 0; // winding number w.r.t. the input
    double m_winding_number_tracked = 0; // winding number w.r.t. the tracked surface
    std::vector<double> m_winding_number_per_input;
    int part_id = -1; // flood fill ID
};

class TetWildMesh : public wmtk::TetMesh
{
public:
    double time_env = 0.0;
    igl::Timer isout_timer;
    const double MAX_ENERGY = 1e50;

    Parameters& m_params;
    /// Surface envelope: what a surface vertex is pulled toward and checked against.
    std::shared_ptr<SampleEnvelope> m_envelope;
    /// Envelope for order-2 vertices, i.e. those on a surface boundary or a non-manifold
    /// edge. Named for the order rather than for "open boundary" because that is what
    /// TetMesh::compute_vertex_order actually reports, and it is the broader set.
    std::shared_ptr<SampleEnvelope> m_order2_envelope;

    /// Optional per-input names (JSON "input_names"), used to label the per-input
    /// winding-number output fields. Empty => the fields are numbered.
    std::vector<std::string> m_input_names;

    /// Per-thread Newton solver for smoothing; created on first use.
    wmtk::threading::enumerable_thread_specific<std::unique_ptr<polysolve::nonlinear::Solver>>
        m_solver;

    /// Why smoothing attempts were refused, reported once per pass.
    optimization::SmoothRejectCounters m_smooth_rejects;

    /// Scale factors putting AMIPS (dimensionless) and the envelope energy (a squared
    /// distance) on a comparable footing.
    double m_s_amips = 1.;
    double m_s_envelope = -1.;

    /// Envelope a vertex is pulled toward while smoothing.
    std::shared_ptr<SampleEnvelope> smoothing_energy_envelope(const size_t vid) const;
    /// Envelope the resulting surface triangles are checked against.
    std::shared_ptr<SampleEnvelope> smoothing_containment_envelope(const size_t vid) const;

    TetWildMesh(
        Parameters& _m_params,
        std::shared_ptr<SampleEnvelope> _m_envelope,
        int _num_threads = 1)
        : m_params(_m_params)
        , m_envelope(std::move(_m_envelope))
    {
        NUM_THREADS = _num_threads;
        p_vertex_attrs = &m_vertex_attribute;
        p_face_attrs = &m_face_attribute;
        p_tet_attrs = &m_tet_attribute;
        m_collapse_check_link_condition = false;
        m_collapse_check_manifold = false;

        optimization::deactivate_opt_logger();
        m_s_amips = 1.;
        m_s_envelope = 1. / (m_params.diag_l * m_params.eps * m_params.eps);
        m_params.w_envelope = 1. - m_params.w_amips;
    }

    ~TetWildMesh() {}
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

        // new for edge
        // m_edge_attribute.resize(6 * n_tet);

        for (auto i = 0; i < _vertex_attribute.size(); i++)
            m_vertex_attribute[i] = _vertex_attribute[i];
        m_tet_attribute.m_attributes = std::vector<TetAttributes>(_tet_attribute.size());
        for (auto i = 0; i < _tet_attribute.size(); i++) m_tet_attribute[i] = _tet_attribute[i];
        for (auto i = 0; i < _tet_attribute.size(); i++)
            m_tet_attribute[i].m_quality = get_quality(tuple_from_tet(i));
    }

    // TODO This should not be here but inside wmtk
    void compute_vertex_partition()
    {
        auto partition_id = partition_TetMesh(*this, NUM_THREADS);
        for (auto i = 0; i < vert_capacity(); i++)
            m_vertex_attribute[i].partition_id = partition_id[i];
    }

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

    ////// Attributes related

    void output_mesh(std::string file);
    void output_faces(std::string file, std::function<bool(const FaceAttributes&)> cond);

    void init_from_delaunay_box_mesh(const std::vector<Eigen::Vector3d>& vertices);

    void finalize_triangle_insertion(const std::vector<std::array<size_t, 3>>& faces);

    void init_from_input_surface(
        const std::vector<Vector3d>& vertices,
        const std::vector<std::array<size_t, 3>>& faces,
        const std::vector<size_t>& partition_id);
    bool triangle_insertion_before(const std::vector<Tuple>& faces) override;
    bool triangle_insertion_after(const std::vector<std::vector<Tuple>>& new_faces) override;

public:
    void split_all_edges();
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& loc) override;

    void smooth_all_vertices();
    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;

    void collapse_all_edges(bool is_limit_length = true);
    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;

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
     * @brief Inversion check using only floating point numbers.
     */
    bool is_inverted_f(const Tuple& loc) const;
    bool is_inverted(const std::array<size_t, 4>& vs) const;
    bool is_inverted(const Tuple& loc) const;
    double get_quality(const std::array<size_t, 4>& vs) const;
    double get_quality(const Tuple& loc) const;
    bool round(const Tuple& loc);
    /**
     * @brief Try to round every un-rounded vertex; returns the number reclaimed.
     *
     * round() is otherwise only attempted as a side effect of another operation
     * (smooth_before on the vertex being smoothed, collapse_edge_after on the merged
     * one), and neither reaches a vertex that only becomes roundable later: smoothing
     * skips "good" regions by default. Without a sweep such a vertex keeps exact
     * coordinates into the output for no geometric reason.
     *
     * Skipped outright when m_all_rounded says there is nothing to do.
     */
    size_t round_all_vertices();

    /**
     * @brief True when every vertex is known to be rounded.
     *
     * Only trusted when true, and only round_all_vertices() sets it that way. Any code
     * that leaves a vertex un-rounded must clear it, or the sweep will skip the vertex
     * forever. Atomic because operations that clear it run in parallel.
     */
    std::atomic<bool> m_all_rounded = false;
    //
    bool is_edge_on_surface(const Tuple& loc);
    bool is_edge_on_bbox(const Tuple& loc);
    /**
     * brief Check if the vertex has an incident boundary edge.
     * This performs a topological check.
     */
    bool is_vertex_on_boundary(const size_t vid);
    //
    void mesh_improvement(int max_its = 80);
    /**
     * @brief Call the original TetWild code.
     */
    void mesh_improvement_legacy(int max_its = 80);
    std::tuple<double, double> local_operations(
        const std::array<int, 4>& ops,
        bool collapse_limit_length = true);
    std::tuple<double, double> get_max_avg_energy();

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
     * @brief Compute the winding number.
     *
     * If `vertices` and `faces` are empty, compute the winding number for the tracked surface.
     * Otherwise, compute the winding number for the input surface given by `vertices` and `faces`.
     */
    /**
     * @brief Barycenter (row per tet) of each tet in `tets`. Computed once and
     * passed to the winding-number passes so they do not each rebuild it.
     */
    Eigen::MatrixXd tet_barycenters(const std::vector<Tuple>& tets) const;

    void compute_winding_number(
        const std::vector<Tuple>& tets,
        const Eigen::MatrixXd& barycenters,
        const std::vector<Vector3d>& vertices = {},
        const std::vector<std::array<size_t, 3>>& faces = {});

    // `in_vertices`/`in_faces` let the single-input case reuse the already-loaded
    // surface instead of re-reading it from disk.
    void compute_winding_numbers(
        const std::vector<std::string>& input_paths,
        const std::vector<Tuple>& tets,
        const Eigen::MatrixXd& barycenters,
        const std::vector<Vector3d>& in_vertices = {},
        const std::vector<std::array<size_t, 3>>& in_faces = {});

    void filter_with_input_surface_winding_number();
    void filter_with_tracked_surface_winding_number();
    void filter_with_flood_fill();


    std::vector<std::array<size_t, 3>> get_faces_by_condition(
        std::function<bool(const FaceAttributes&)> cond) const;

    bool invariants(const std::vector<Tuple>& t) override; // this is now automatically checked

    double get_length2(const Tuple& loc) const;
    // debug use
    std::atomic<int> cnt_split = 0, cnt_collapse = 0, cnt_swap = 0;
    // Successful surface diagonal flips (subset of cnt_swap). Diagnostic.
    std::atomic<int> cnt_surface_swap = 0;

private:
    // tags: correspondence map from new tet-face node indices to in-triangle ids.
    // built up while triangles are inserted.
    wmtk::threading::concurrent_map<std::array<size_t, 3>, std::vector<int>> tet_face_tags;

    struct TriangleInsertionLocalInfoCache
    {
        // local info: for each face insertion
        int face_id;
        std::vector<std::array<size_t, 3>> old_face_vids;
    };
    wmtk::threading::enumerable_thread_specific<TriangleInsertionLocalInfoCache>
        triangle_insertion_local_cache;

    ////// Operations

    struct SplitInfoCache
    {
        //        VertexAttributes vertex_info;
        size_t v1_id;
        size_t v2_id;
        bool is_edge_on_surface = false;
        bool is_edge_open_boundary = false;
        size_t edge_order = 0;
        std::vector<size_t> v1_param_type;
        std::vector<size_t> v2_param_type;

        std::vector<std::pair<FaceAttributes, std::array<size_t, 3>>> changed_faces;
    };
    wmtk::threading::enumerable_thread_specific<SplitInfoCache> split_cache;

    struct CollapseInfoCache
    {
        size_t v1_id;
        size_t v2_id;
        double max_energy;
        double edge_length;
        bool is_limit_length;

        std::vector<std::pair<FaceAttributes, std::array<size_t, 3>>> changed_faces;
        // all faces incident to the delete vertex (v1) that are on the tracked surface
        std::vector<std::array<size_t, 3>> surface_faces;
        // all edges incident to the deleted vertex(v1) that are on the open boundary
        std::vector<std::array<size_t, 2>> boundary_edges;
        std::vector<size_t> changed_tids;
        std::vector<double> changed_energies;

        std::vector<std::array<size_t, 2>> failed_edges;

        std::map<std::pair<size_t, size_t>, int> edge_link;
        std::map<size_t, int> vertex_link;
        size_t global_nonmani_ver_cnt;

        // debug use
        std::vector<size_t> one_ring_surface_vertices;
        std::vector<std::pair<size_t, size_t>> one_ring_surface_edges;
        std::vector<std::array<size_t, 3>> one_ring_surface;

        // for geometry preservation
        std::vector<size_t> edge_incident_param_type;
    };
    wmtk::threading::enumerable_thread_specific<CollapseInfoCache> collapse_cache;


    struct SwapInfoCache
    {
        double max_energy;
        std::map<std::array<size_t, 3>, FaceAttributes> changed_faces;

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


    // for incremental tetwild
public:
    /**
     * Will be removed as soon as the bug in the faster version is fixed.
     */
    void insertion_by_volumeremesher_old(
        const std::vector<Vector3d>& vertices,
        const std::vector<std::array<size_t, 3>>& faces,
        std::vector<Vector3r>& v_rational,
        std::vector<std::array<size_t, 3>>& facets_after,
        std::vector<bool>& is_v_on_input,
        std::vector<std::array<size_t, 4>>& tets_after,
        std::vector<bool>& tet_face_on_input_surface);

    /**
     * This version of insertion should be faster BUT IS BROKEN!!!
     * DO NOT USE!!!!!
     */
    void insertion_by_volumeremesher(
        const std::vector<Vector3d>& vertices,
        const std::vector<std::array<size_t, 3>>& faces,
        std::vector<Vector3r>& v_rational,
        std::vector<std::array<size_t, 3>>& facets_after,
        std::vector<bool>& is_v_on_input,
        std::vector<std::array<size_t, 4>>& tets_after,
        std::vector<bool>& tet_face_on_input_surface);

    void init_from_Volumeremesher(
        const std::vector<Vector3r>& v_rational,
        const std::vector<std::array<size_t, 3>>& facets,
        const std::vector<bool>& is_v_on_input,
        const std::vector<std::array<size_t, 4>>& tets,
        const std::vector<bool>& tet_face_on_input_surface);

    void init_from_file(std::string input_dir);

    std::vector<std::array<size_t, 3>> triangulate_polygon_face(std::vector<Vector3r> points);
    bool check_polygon_face_validity(std::vector<Vector3r> points);

    bool check_nondegenerate_tets();
    void output_embedded_polygon_mesh(
        std::string output_dir,
        const std::vector<Vector3r>& v_rational,
        const std::vector<std::vector<size_t>>& polygon_faces,
        const std::vector<std::vector<size_t>>& polygon_cells,
        const std::vector<bool>& polygon_faces_on_input_surface);

    void output_embedded_polygon_surface_mesh(
        std::string output_dir,
        const std::vector<Vector3r>& v_rational,
        const std::vector<std::vector<size_t>>& polygon_faces,
        const std::vector<bool>& polygon_faces_on_input_surface);

    void output_tetrahedralized_embedded_mesh(
        std::string output_dir,
        const std::vector<Vector3r>& v_rational,
        const std::vector<std::array<size_t, 3>>& facets,
        const std::vector<std::array<size_t, 4>>& tets,
        const std::vector<bool>& tet_face_on_input_surface);

    void output_init_tetmesh(std::string output_dir);

    void output_tracked_surface(std::string output_file);

    bool adjust_sizing_field_serial(double max_energy);

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
    size_t refine_sizing_around_worst(double max_energy);

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

    /// Per-pass claim for the high-valence split gate: one slot per vertex, reset at the
    /// start of every split pass. A high-valence vertex accepts the first
    /// valence-increasing split of the pass and refuses the rest, so refinement spreads
    /// instead of piling onto the same vertex. Atomic because splits run in parallel; a
    /// plain array of unique_ptr rather than a vector because std::atomic is not movable.
    std::unique_ptr<std::atomic<int>[]> m_high_valence_claim;
    size_t m_high_valence_claim_size = 0;
    /// Splits refused by that gate in the current pass, reported once per pass.
    std::atomic<size_t> m_high_valence_rejects = 0;

    /// True iff edge (v1,v2) is a worst tet's longest edge queued for force-split.
    bool is_force_split_edge(size_t v1, size_t v2) const
    {
        return m_force_split_edges.find(simplex::Edge(v1, v2)) != m_force_split_edges.end();
    }

    // for open boundary
    void find_open_boundary();
    bool is_open_boundary_edge(const Tuple& e);
    bool is_open_boundary_edge(const std::array<size_t, 2>& e);

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
    // debug functions
    int orient3D(
        vol_rem::bigrational px,
        vol_rem::bigrational py,
        vol_rem::bigrational pz,
        vol_rem::bigrational qx,
        vol_rem::bigrational qy,
        vol_rem::bigrational qz,
        vol_rem::bigrational rx,
        vol_rem::bigrational ry,
        vol_rem::bigrational rz,
        vol_rem::bigrational sx,
        vol_rem::bigrational sy,
        vol_rem::bigrational sz);

    // bool checkTrackedFaces(
    //     std::vector<vol_rem::bigrational>& vol_coords,
    //     const std::vector<double>& surf_coords,
    //     std::vector<uint32_t>& facets,
    //     std::vector<uint32_t>& facets_on_input,
    //     const std::vector<uint32_t>& surf_tris);

    int orient3D_wmtk_rational(
        wmtk::Rational px,
        wmtk::Rational py,
        wmtk::Rational pz,
        wmtk::Rational qx,
        wmtk::Rational qy,
        wmtk::Rational qz,
        wmtk::Rational rx,
        wmtk::Rational ry,
        wmtk::Rational rz,
        wmtk::Rational sx,
        wmtk::Rational sy,
        wmtk::Rational sz);

    bool checkTrackedFaces_wmtk_rational(
        std::vector<wmtk::Rational>& vol_coords,
        const std::vector<double>& surf_coords,
        std::vector<uint32_t>& facets,
        std::vector<uint32_t>& facets_on_input,
        const std::vector<uint32_t>& surf_tris);

    bool check_vertex_param_type();

    // for boolean operations
    int flood_fill();

    void save_paraview(const std::string& path, const bool use_hdf5);

    // initialize sizing field (for topology preservation)
    void init_sizing_field();

public:
    struct ExportStruct
    {
        // tet mesh
        MatrixXd V;
        MatrixXi T;
        // tracked surface
        MatrixXi F;
        // attributes
        VectorXd t_amips;
        VectorXd t_winding_number_input;
        VectorXd t_winding_number_tracked;
        MatrixXd t_winding_number_per_input;
        VectorXi t_part;
    };
    // export functionality
    ExportStruct export_mesh_data() const;
};


} // namespace wmtk::components::tetwild
