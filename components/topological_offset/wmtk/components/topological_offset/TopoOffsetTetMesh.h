#pragma once

#include <wmtk/TetMesh.h>
#include <wmtk/TetOptimizerMesh.h>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include "Parameters.h"
#include "SimplicialComplexBVH.hpp"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

using CellTag = std::set<int64_t>;


namespace wmtk::components::topological_offset {


const int64_t TEMP_OFFSET_TET_TAG = -1;
const CellTag TEMP_OFFSET_TET_TAG_SET{TEMP_OFFSET_TET_TAG};


// for all attributes:
// label: 0=default, 1=input, 2=offset

/**
 * @brief Per-vertex data the shared optimizer knows nothing about.
 *
 * Position, rounding, bbox membership, order, sizing and partition all live on
 * wmtk::TetOptimizerMesh::VertexAttributes; this carries only what is the offset's own.
 * Registered with the base's m_vertex_attr_group, so it is resized, protected and rolled back
 * with the shared collection.
 *
 * Note m_is_on_input / m_is_on_offset versus the base's m_is_on_surface: the offset tracks TWO
 * surfaces, and the base's flag is their union. These two say which.
 */
class VertexExtra
{
public:
    int label = 0;
    size_t component_id = 0;
    bool m_is_on_input = false; // is this vertex on the input complex
    bool m_is_on_offset = false; // is this vertex on the offset surface
};


class EdgeAttributes
{
public:
    int label = 0; // label: 0=default, 1=input, 2=offset
};


/**
 * @brief Per-face data the shared optimizer knows nothing about.
 *
 * Whether a face is tracked surface, which surface, and which bbox side it is on all live on
 * wmtk::SurfaceTagAttributes. Only the construction label is the offset's own, and only the
 * pre-optimization phase reads it (marching tets, simplicial embedding, the offset growth in
 * Spatial.cpp) -- optimize_offset() consumes it once and ignores it from then on.
 *
 * Registered with the base's m_face_attr_group, so it survives the mesh changing shape. Its
 * VALUES, though, are only maintained by the offset's own operations: the shared split,
 * collapse and swap propagate face data by copying SurfaceTagAttributes around and never see
 * this. That is sound precisely because the label is dead by the time they run.
 */
class FaceExtra
{
public:
    int label = 0; // label: 0=default, 1=input, 2=offset
};


class TetAttributes
{
public:
    int label = 0; // label: 0=default, 1=input, 2=offset
    CellTag tag;
    double m_quality = 0; // AMIPS energy, kept up to date by smoothing
};


/**
 * @brief one sample of the input complex's implicit offset field, taken near a point on an
 * offset-surface face. `normal` is Vector3d::Zero() if the sample point landed exactly on the
 * input complex, in which case no normal direction could be recovered.
 * @note mirrors a single sample of
 * https://github.com/wildmeshing/topological-offsets/blob/main/components/topological_offsets/wmtk/components/topological_offsets/internal/utils/Quadrics.cpp's
 * get_triangle_samples_and_area()
 */
struct OffsetSurfaceSample
{
    Vector3d point; // the sample point, on (or near a corner of) the face
    Vector3d nearest; // nearest point on the input complex to `point`
    Vector3d normal; // (point - nearest).normalized()
    double weight; // 1 for the face centroid, 0.1 for the 3 near-corner samples
};


/**
 * @brief The offset's tet mesh, on the shared 3D optimizer.
 *
 * The construction phase -- marching tets, simplicial embedding, growing the offset region --
 * is entirely its own and uses TetMesh's face and tet splits directly. The optimization phase
 * that follows is wmtk::TetOptimizerMesh's, with the offset supplying only its own policy
 * through the hooks: which cell quality, which tags to propagate, which surface is
 * envelope-constrained, and how an offset-surface vertex is smoothed.
 *
 * Two surfaces are tracked, not one. The input complex is class 0 and must stay inside
 * m_envelope; the offset boundary is OFFSET_SURFACE_CLASS and is free to move, being defined by
 * the tet labels rather than by input geometry. The base's m_is_surface_fs / m_is_on_surface
 * are the UNION of the two -- that is what must not be torn by an operation -- and
 * VertexExtra::m_is_on_input / m_is_on_offset say which.
 */
class TopoOffsetTetMesh : public wmtk::TetOptimizerMesh
{
public:
    /**
     * @brief SurfaceTagAttributes::m_surface_class for the offset boundary.
     *
     * The input complex keeps the primary class 0, so a face carrying input geometry is
     * envelope-checked by the shared operations exactly as in tetwild and simwild.
     */
    static constexpr int OFFSET_SURFACE_CLASS = 1;

    /**
     * @brief AVERAGE distance error, as a fraction of target_distance, above which a
     * non-converged run with respect_all_topologies is reported as topologically blocked.
     *
     * The discriminator is the AVERAGE, not the max and not max/avg. Topological blocking is
     * WIDESPREAD -- whole stretches of offset cannot advance, so the average climbs toward the
     * max. Every other failure mode is the opposite shape: a few over-constrained vertices pin
     * the max while the rest lands correctly, leaving the average tiny. In 2D avg/delta separated
     * the blocked runs from every other run by more than 150x.
     *
     * Class-scope rather than namespace-scope only because TopoOffsetTriMesh.h already defines a
     * namespace-scope constant of this name and the two headers share a namespace.
     */
    static constexpr double TOPOLOGY_BLOCK_AVG_FRAC = 0.05;

public: // mode for splitting in marching tets
    enum class EdgeSplitMode {
        Midpoint = 0, // used for simplicial embedding steps
        Initial = 1, // this is used to initialize the complex. Its a little hacky

        Optimization = 5 // this is used in the optimization phase of the algorithm
    };

public:
    int m_vtu_counter = 0;
    std::array<size_t, 4> m_init_counts = {{0, 0, 0, 0}};
    size_t m_tags_count;
    SimplicialComplexBVH m_input_complex_bvh;
    EdgeSplitMode m_edge_split_mode = EdgeSplitMode::Midpoint;

    // tag map stuff
    std::map<std::string, int64_t> m_tag_name_to_id;
    std::map<int64_t, std::string> m_tag_id_to_name;
    CellTag m_offset_output_tag_ids;

    // if in 'singlebody' mode
    bool m_singlebody = false;
    int64_t m_single_tag;

    // dont actually use, just for retaining in output
    bool m_has_envelope;
    std::vector<Vector3d> m_V_envelope;
    std::vector<Vector3i> m_F_envelope;
    // m_envelope itself lives on the base, which is what checks tracked-surface triangles
    // against it.
    double m_envelope_eps = -1;

    /// The base holds only wmtk::OptimizerParameters; this is the same object, typed, for the
    /// offset-only fields.
    Parameters& m_offset_params;

    using VertexExtraCol = wmtk::AttributeCollection<VertexExtra>;
    using EdgeAttCol = wmtk::AttributeCollection<EdgeAttributes>;
    using FaceExtraCol = wmtk::AttributeCollection<FaceExtra>;
    using TetAttCol = wmtk::AttributeCollection<TetAttributes>;
    // m_vertex_attribute and m_face_attribute are the base's; these two are registered
    // alongside them in its attribute groups.
    VertexExtraCol m_vertex_extra;
    FaceExtraCol m_face_extra;
    EdgeAttCol m_edge_attribute;
    TetAttCol m_tet_attribute;

    /**
     * @brief A face's shared surface tags together with the offset's own label.
     *
     * The marching-tets splits snapshot a face and write it back onto the pieces it became,
     * and both halves have to travel together.
     */
    struct FaceSnapshot
    {
        FaceAttributes tags;
        FaceExtra extra;
    };
    FaceSnapshot face_snapshot(const size_t fid) const
    {
        return FaceSnapshot{m_face_attribute[fid], m_face_extra[fid]};
    }
    void restore_face(const size_t fid, const FaceSnapshot& s)
    {
        m_face_attribute[fid] = s.tags;
        m_face_extra[fid] = s.extra;
    }

    /**
     * @brief Place a vertex, keeping its exact and rounded coordinates in step.
     *
     * The offset works in doubles throughout -- its construction is driven by a BVH distance
     * field to the input complex, which has no exact form -- so every vertex it places is
     * rounded, and its rational position is just the exact value of the double it carries.
     *
     * Filling m_pos anyway is not bookkeeping for its own sake. The shared split falls back to
     * the EXACT midpoint when the double one would invert an incident tet, which is what lets a
     * degenerate region keep being refined instead of the split being refused; that fallback
     * reads the endpoints' m_pos, and every quality and orientation test around an un-rounded
     * vertex reads its neighbours'. A vertex placed here with a stale m_pos would silently feed
     * those the wrong point.
     */
    void set_vertex_position(const size_t vid, const Vector3d& p)
    {
        m_vertex_attribute[vid].m_posf = p;
        m_vertex_attribute[vid].m_pos = to_rational(p);
        m_vertex_attribute[vid].m_is_rounded = true;
    }

    /**
     * @brief Whether tet `tid` belongs to the closed offset region, read from its TAGS.
     *
     * The 3D counterpart of TopoOffsetTriMesh::face_in_region, and it fixes the same defect.
     * The region is the offset band plus the input complex it wraps, both named by tags -- and
     * tags are what the shared operations propagate. The construction label says the same
     * thing but is derived state maintained alongside them, so it goes stale the moment a
     * split or swap creates a cell the label was never written for, putting holes in the
     * region that offset_is_manifold() then reports as non-manifold.
     */
    bool cell_in_region(const size_t tid) const
    {
        const int l = m_tet_attribute[tid].label;
        return l == 1 || l == 2; // input complex, or the offset band
    }

    /// Whether face `fid` is on the offset boundary (as opposed to the input complex).
    bool face_is_offset(const size_t fid) const
    {
        return m_face_attribute[fid].m_is_surface_fs &&
               m_face_attribute[fid].m_surface_class == OFFSET_SURFACE_CLASS;
    }
    /// Whether face `fid` carries input-complex geometry.
    bool face_is_input(const size_t fid) const
    {
        return m_face_attribute[fid].m_is_surface_fs &&
               m_face_attribute[fid].m_surface_class != OFFSET_SURFACE_CLASS;
    }

    // debug use
    std::atomic<int> cnt_split = 0, cnt_collapse = 0;

    TopoOffsetTetMesh(Parameters& _m_offset_params, int _num_threads = 0)
        : wmtk::TetOptimizerMesh(_m_offset_params, nullptr)
        , m_offset_params(_m_offset_params)
    {
        NUM_THREADS = _num_threads;
        // The base owns the vertex and face slots; register the offset's own data with its
        // groups so it is resized, protected and rolled back with them.
        m_vertex_attr_group.add(&m_vertex_extra);
        m_face_attr_group.add(&m_face_extra);
        p_edge_attrs = &m_edge_attribute;
        p_tet_attrs = &m_tet_attribute;

        m_collapse_check_link_condition = false;
        m_collapse_check_manifold = false;

        optimization::deactivate_opt_logger();
    }

    ~TopoOffsetTetMesh() override = default;

    ////// wmtk::TetOptimizerMesh hooks

    double cell_quality(const size_t tid) const override { return m_tet_attribute[tid].m_quality; }
    void set_cell_quality(const size_t tid, const double q) override
    {
        m_tet_attribute[tid].m_quality = q;
    }

    /**
     * @brief Only the input complex is envelope-constrained.
     *
     * The offset boundary is defined by which tets carry the offset label, not by input
     * geometry, so there is nothing for it to stay inside; the checks that keep it faithful to
     * the implicit offset field are the normal-deviation ones in collapse and swap. Decided
     * from the vertices rather than the face's own class because the shared operations ask
     * about triangles they are creating, whose attributes are not written yet.
     */
    std::shared_ptr<SampleEnvelope> surface_envelope_for_face(
        const std::array<size_t, 3>& vids) const override
    {
        for (const size_t v : vids) {
            if (!m_vertex_extra[v].m_is_on_input) return nullptr;
        }
        return m_envelope;
    }

    /// Surface edges may be flipped, as a topology-preserving diagonal flip. Both tracked
    /// surfaces need it: the offset boundary is re-triangulated constantly, and refusing to
    /// flip its diagonals is what leaves the badly-shaped triangles the sizing field then
    /// chases.
    bool allow_surface_swap() const override { return true; }
    bool check_surface_topology() const override { return m_offset_params.perform_sanity_checks; }

    /// Input-complex vertices never move (smooth_before refuses them), and an offset-surface
    /// vertex is pulled by the quadrics rather than by an envelope, so there is nothing to pull
    /// anything toward.
    std::shared_ptr<SampleEnvelope> smoothing_energy_envelope(const size_t) const override
    {
        return nullptr;
    }

    void write_optimization_debug_output(const std::string& path) override { write_vtu(path); }

    /**
     * @brief The engine's stall-driven sizing refinement, driven by the offset's criterion.
     *
     * mesh_improvement() fires this when optimization_quality_stats()'s max stalls; the
     * override ratchets the sizing down around the worst-placed band vertices only
     * (TetWildMesh::refine_sizing_around_worst with distance error in place of AMIPS), and
     * keeps the paper's 1.5x growth where the surface is flat, in-band and well-shaped.
     */
    size_t refine_sizing_around_worst(double) override;

    /// min: a collapse must not un-refine the sizing field -- see the base declaration.
    double collapse_merged_sizing(double removed, double survivor) const override
    {
        return std::min(removed, survivor);
    }

    /**
     * @brief (max, avg) band-vertex distance error over convergence_target; the engine's stop
     * metric is therefore 1.0. Distance only -- the average-normal-deviation criterion is
     * tested after the loop.
     */
    std::tuple<double, double> optimization_quality_stats() override;
    double optimization_stop_metric() const override { return 1.; }

    /**
     * @brief Which tag the tets a swap creates should carry.
     *
     * A swap must not move the boundary between differently tagged regions, since that
     * boundary IS the offset. Around an interior edge every incident tet already shares a tag
     * (a face between differently tagged tets is a tracked surface face, and the base only
     * takes this path when the edge has none), so this is a cheap safety net there. On a
     * surface flip the ring genuinely spans two tags and the majority one wins.
     *
     * Returns false, refusing the swap, when three or more tags meet: there is then no single
     * answer and any choice would relabel a tet.
     */
    bool swap_before_interior(const std::vector<size_t>& tids) override;
    bool swap_before_surface(
        const std::vector<size_t>& tids,
        size_t a,
        size_t b,
        size_t c,
        size_t d) override;
    bool swap_after_cells(const std::vector<size_t>& tids, bool is_surface_flip) override;

    /**
     * @brief Collapse policy that is the offset's own.
     *
     * collapse_before_vertex refuses to move a vertex off the surface it belongs to (the base
     * only knows the union of the two), to lower the order of a boundary vertex, or to collapse
     * across what looks like a feature of the input complex; it also records how badly aligned
     * the offset surface already was, so its `after` counterpart can tell a regression from a
     * defect that was there before.
     */
    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_before_vertex(size_t v1, size_t v2, double edge_length) override;
    bool collapse_after_connectivity(
        size_t v1,
        size_t v2,
        const std::vector<std::array<size_t, 2>>& boundary_edges) override;
    bool collapse_is_order_2_edge(const std::array<size_t, 2>& e) override
    {
        return is_order_2_edge(e);
    }
    void collapse_after_vertex(size_t v1, size_t v2) override;

    /**
     * @brief Split policy that is the offset's own.
     *
     * The shared split places the vertex, keeps the quality and the shared attributes and
     * checks containment; what it cannot know is which region tag the two child tets inherit,
     * and which of the two tracked surfaces the new vertex joined.
     */
    bool split_before_cells(const Tuple& edge, const std::vector<Tuple>& parents) override;
    bool split_after_cells(size_t v1, size_t v2, size_t v_new, const std::vector<Tuple>& children)
        override;
    /// Writes the new vertex's tracked-surface membership before the shared split's containment
    /// check reads it; returns the base's positioning verdict unchanged. See the definition.
    bool split_adjust_position(size_t v_new, const std::vector<Tuple>& children) override;
    void split_after_vertex(size_t v_new, bool is_edge_open_boundary) override;

    /// The offset's surface can end on a non-manifold or boundary edge of the input complex,
    /// which the base must not flip or split across.
    bool is_open_boundary_edge(const Tuple& e) override { return is_order_2_edge(e); }

private:
    /// What the shared split has to carry across for the offset: the region tag of each parent
    /// tet, keyed by the edge opposite the split one, and which surfaces the edge was on.
    struct OptSplitCache
    {
        bool is_edge_on_input = false;
        bool is_edge_on_offset = false;
        std::map<simplex::Edge, TetAttributes> tets;
    };
    wmtk::threading::enumerable_thread_specific<OptSplitCache> m_opt_split_cache;

public:
private:
    /// The worse of the two endpoints' offset-surface deviation before the collapse.
    wmtk::threading::enumerable_thread_specific<double> m_collapse_nd_before;

public:
private:
    bool swap_capture_tag(const std::vector<size_t>& tids);
    /// The tag swap_after_cells writes onto the tets the swap created, chosen in `before`.
    wmtk::threading::enumerable_thread_specific<CellTag> m_swap_tag;
    /// The construction LABEL shared by every cell of the swap's ring, captured alongside the
    /// tag. Without this a swap leaves its new cells holding whatever label occupied the
    /// recycled tet slot, and the label is what region membership is read from.
    wmtk::threading::enumerable_thread_specific<int> m_swap_label;

public:
    /**
     * @brief initialize TetMesh from vertex, tet, and tag data
     * @param V: #V by 3 vertex matrix
     * @param T: #T by 4 tet matrix
     * @param T_tags: #T by #tags tag matrix
     * @param V_env: #V_env by 3 EnvelopeSurface vertices
     * @param F_env: #F_env by 3 EnvelopeSurface faces
     */
    void init_from_image(
        const MatrixXd& V,
        const MatrixXi& T,
        const MatrixSi& T_tags,
        const MatrixXd& V_env,
        const MatrixXi F_env,
        const std::vector<std::string>& tag_names);

    void init_surfaces_and_boundaries();

    /// Whether edge `loc` lies on the INPUT complex. The base's is_edge_on_input() asks
    /// about the union of the two tracked surfaces, and is_edge_on_bbox() is the base's.
    bool is_edge_on_input(const Tuple& loc);
    bool is_edge_on_offset(const Tuple& loc);

    /**
     * @brief The two simplex sets the optimization may not touch at all.
     *
     * The input simplicial complex is the geometry the offset is measured against, and the
     * domain boundary is the box the background mesh lives in. Neither is something to be
     * improved: an operation that splits, collapses, flips or moves any of their simplices
     * changes the reference the whole result is defined against. This is categorical, not a
     * tolerance -- the shared engine holds the input surface inside m_envelope, which lets it
     * drift by eps, and eps is not zero.
     *
     * Same rule and same wording as TopoOffsetTriMesh::vertex_is_frozen()/edge_is_frozen(). The
     * one difference the dimension forces: 2D reads an edge's bbox membership off the edge
     * itself, while 3D tracks the box on FACES, so the edge test goes through the base's
     * is_edge_on_bbox(), which walks the edge's incident faces. Both are therefore non-const.
     */
    bool vertex_is_frozen(const size_t vid) const
    {
        return m_vertex_extra[vid].m_is_on_input || !m_vertex_attribute[vid].on_bbox_faces.empty();
    }
    bool edge_is_frozen(const Tuple& loc) { return is_edge_on_input(loc) || is_edge_on_bbox(loc); }

    /**
     * @brief check that the ambient tag does not overlap with any other tags
     */
    bool ambient_assert();

    /**
     * @brief label input simplicial complex simplices, as defined in
     * m_offset_params.offset_selection
     */
    void label_input_complex();

    /**
     * @brief check if the input complex is empty. Only valid after calling init_from_image(...).
     * Checks if any vertices (therefore any simplices) are labelled 1, if not returns true
     */
    bool empty_input_complex();

    /**
     * @brief initialize BVH for input complex. Must be called after init_from_image(...)
     */
    void init_input_complex_bvh();


    /**
     * @brief label connected simplicial complex components (simplices labelled 1 or 2)
     */
    size_t flood_fill();

    std::vector<std::array<size_t, 3>> get_faces_by_condition(
        std::function<bool(const FaceAttributes&)> cond) const;

    //// overriden splits/invariants
    /// Dispatch: the optimization phase runs the shared split, everything else the
    /// marching-tets one below.
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& t) override;
    bool marching_split_edge_before(const Tuple& t);
    bool marching_split_edge_after(const Tuple& t);
    bool split_face_before(const Tuple& t) override;
    bool split_face_after(const Tuple& t) override;
    bool split_tet_before(const Tuple& t) override;
    bool split_tet_after(const Tuple& t) override;
    bool invariants(const std::vector<Tuple>& tets) override;
    /// The base rounds and refuses bbox vertices; this additionally freezes the input
    /// complex, and routes offset-surface vertices to the quadrics step below.
    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;
    //// overriden splits/invariants

    /**
     * @brief main function from which all others are called
     */
    void execute_offset(const std::filesystem::path& output_file);

    /**
     * @brief optimize the offset
     */
    void optimize_offset(const std::filesystem::path& output_file);

    //// smoothing

    /**
     * @brief true if face f has exactly one incident tet labelled 2 (offset), i.e. it lies on
     * the boundary between the offset region and the rest of the mesh
     */
    bool is_offset_face(const Tuple& f) const;
    bool is_offset_face(const size_t fid) const;

    /**
     * @brief offset-surface faces (see is_offset_face()) incident to vertex t
     */
    std::vector<Tuple> get_offset_surface_faces_for_vertex(const Tuple& t) const;

    /**
     * @brief the 4 offset-field samples for face f (centroid + one near each corner), following
     * Quadrics.cpp's get_triangle_samples_and_area(). Shared by the Quadrics construction below
     * and by the feature-preserving checks in Collapse.cpp -- a single centroid sample cannot
     * tell a locally-flat patch from one straddling a sharp feature of the input complex, since
     * a single BVH nearest-point query just picks whichever side of the feature is closer.
     * @note a sample is only accepted (non-zero OffsetSurfaceSample::normal) if it lies in the
     * direction of offset_face_outward_normal(f); otherwise it is degenerate for our purposes,
     * same as landing exactly on the input complex.
     */
    std::array<OffsetSurfaceSample, 4> offset_surface_samples(const Tuple& f) const;

    /**
     * @brief quadrics-based smoothing step for offset surface vertices: blends a Laplacian
     * step with a projection onto quadrics built from target_distance-offset samples of the
     * input complex, following
     * https://github.com/wildmeshing/topological-offsets/blob/main/components/topological_offsets/wmtk/components/topological_offsets/internal/OffsetOptimization.cpp#L2226
     * @note skeleton: no bisection fallback toward p0 on rejection.
     */
    bool smooth_after_offset_surface(const Tuple& t);

    /**
     * @brief Why smoothing refused a vertex, one counter per site where it can say no.
     *
     * The base's SmoothRejectCounters only sees the vertices that reach
     * TetOptimizerMesh::smooth_after(), which for the offset is the INTERIOR ones -- an
     * offset-surface vertex is placed by smooth_after_offset_surface() and never touches them.
     * So the whole offset surface, the part that actually carries the distance error, is
     * invisible in the base's "accepted N | rejected: ..." line. These counters cover every
     * site on both paths.
     *
     * Field-for-field the same as TopoOffsetTriMesh::SmoothTrace, including the three counters
     * 3D has no mechanism to raise yet -- they stay in so the two log lines diff cleanly, and
     * so they light up on their own when the mechanism lands. Each is commented below.
     */
    struct SmoothTrace
    {
        std::atomic<int> attempted{0}; ///< smooth_before() entered
        std::atomic<int> before_bbox{0}; ///< base smooth_before said no: on the bounding box
        std::atomic<int> before_unrounded{0}; ///< base smooth_before said no: could not round
        std::atomic<int> before_on_input{0}; ///< input complex, frozen
        std::atomic<int> offset_attempted{0}; ///< dispatched to smooth_after_offset_surface()
        std::atomic<int> offset_no_neighbours{0}; ///< no offset-surface neighbour to average
        std::atomic<int> offset_on_complex{0}; ///< every sample degenerate: no offset direction
        /// The binary search found no legal step at all: every fraction of the move inverts an
        /// incident tet, possibly including the zero step (p0 itself already inverted). This IS
        /// a rejection -- smooth_after_offset_surface() restores p0 and returns false, following
        /// the reference implementation, which refuses rather than accepting a zero-length move.
        /// A vertex in this state received none of its correction this pass.
        std::atomic<int> offset_inverted{0};
        /// Always 0 in 3D: the region-boundary envelope is 2D-only, so nothing on this path can
        /// be rejected by containment. Kept for lock-step with 2D.
        std::atomic<int> offset_envelope{0};
        std::atomic<int> offset_accepted{0};
        /// Accepted, but the binary search had to stop short of the blended target. These are
        /// invisible in offset_accepted, which is the point: a clamped move reports success
        /// while delivering a fraction of the requested correction.
        std::atomic<int> offset_clamped{0}; ///< accepted with the search fraction < 0.99
        /// Always 0 in 3D, for the same reason as offset_envelope.
        std::atomic<int> offset_clamp_env{0};
        std::atomic<int> offset_clamp_inv{0}; ///< ... and an inverted tet is what stopped it
        /// Always 0 until the tangent-PLANE slide lands (the 3D counterpart of 2D's
        /// minimize_distance_along_tangent(); see .claude/CLAUDE.md 3D note 7).
        std::atomic<int> offset_slid{0};
        /// Distance error over the offset vertices this pass actually touched, before and after
        /// the move, in units of 1e-9 so an atomic<int> can accumulate a max/sum.
        std::atomic<long long> offset_err_before_nano{0};
        std::atomic<long long> offset_err_after_nano{0};
        std::atomic<int> offset_err_max_before_nano{0};
        std::atomic<int> offset_err_max_after_nano{0};
        std::atomic<int> interior_attempted{0}; ///< dispatched to the shared AMIPS smoother
        /// Always 0 in 3D: there is no REGION surface class here, only INPUT and OFFSET.
        std::atomic<int> region_attempted{0};

        void reset()
        {
            offset_err_before_nano.store(0);
            offset_err_after_nano.store(0);
            for (std::atomic<int>* c :
                 {&attempted,
                  &before_bbox,
                  &before_unrounded,
                  &before_on_input,
                  &offset_attempted,
                  &offset_no_neighbours,
                  &offset_on_complex,
                  &offset_inverted,
                  &offset_envelope,
                  &offset_accepted,
                  &offset_clamped,
                  &offset_clamp_env,
                  &offset_clamp_inv,
                  &offset_slid,
                  &offset_err_max_before_nano,
                  &offset_err_max_after_nano,
                  &interior_attempted,
                  &region_attempted}) {
                c->store(0);
            }
        }
    };
    SmoothTrace m_smooth_trace;
    void log_smooth_trace() const;


    //// smoothing

    //// collapse

    // max angle (degrees, 0-90, orientation independent) allowed between an offset-surface
    // face's own normal and the input-complex normal it is supposed to approximate, checked by
    // collapse_edge_before/after and offset_swap_normal_deviation_ok(). Exposed to the user as
    // /max_normal_deviation_deg (see Parameters::max_normal_deviation_deg). See
    // face_normal_deviation() and
    // https://github.com/wildmeshing/topological-offsets/blob/main/components/topological_offsets/wmtk/components/topological_offsets/internal/invariants/NormalDeviationAfterInvariant.cpp
    // and .../invariants/OffsetCollapseBeforeInvariant.cpp
    double m_max_normal_deviation_swap_max_deg = 75.0;

    /**
     * @brief Paper Definition 5: max angle (degrees) between the offset FIELD normal at face f's
     * centroid and the field normal at each of its three near-corner samples.
     *
     * Both terms are field normals -- f's own geometric normal does not appear. n() is continuous
     * away from the input complex's features, so shrinking a face drives sigma to zero, which is
     * what makes this something update_sizing_field() can actually satisfy. The earlier version
     * compared f's own normal against the samples, i.e. misorientation, which refinement does not
     * fix and the sizing field could therefore never drive down.
     *
     * Mirrors TopoOffsetTriMesh::edge_normal_deviation(); the only difference is 3 samples
     * instead of 2, because a triangle has three vertices and a segment has two.
     */
    double face_normal_deviation(const Tuple& f) const;

    /**
     * @brief The offset field's normal at an arbitrary point: the unit vector from the nearest
     * point on the input complex out to p. False if p sits on the complex, where it has none.
     *
     * "The offset normal can be computed for any point in space by finding the projection point
     * on the offset and normalizing the vector from the point in space to its projection."
     * (paper, Appendix A.)
     */
    /**
     * @brief Seed the sizing field from the offset surface's CURRENT edge lengths, once, before
     * the first pass. Paper Sec. 5.3.3 Step 1: "initialized with the current length of each edge."
     *
     * Without it the field starts at the base target (a fraction of the bounding box), which is
     * far coarser than the offset surface, so every offset edge is a collapse candidate on pass
     * one and the surface is decimated before any metric is computed.
     */
    void init_offset_sizing_field();

    bool offset_field_normal(const Vector3d& p, Vector3d& n) const;

    /**
     * @brief How far the offset surface is from where it should be: {max, avg} over its vertices.
     *
     * The absolute error |dist(v, input complex) - target_distance|, over the band's OUTER
     * surface only -- a band cell meeting a plain-background cell, not the input complex it
     * wraps, whose interface sits at distance 0 by construction. The MAX is what the optimization
     * converges against: the offset is only as good as its worst-placed vertex.
     *
     * A face with no opposite tet is ON THE DOMAIN BOUNDARY and is counted, not skipped. The 2D
     * version skipped those and under-reported a bbox-clipped offset by 11x -- true error on the
     * clipped stretch reached 52% of the target distance while the report showed 0.5%. Those
     * vertices are frozen on the box and can never be fixed, which is exactly why they must be
     * visible rather than silently dropped.
     */
    std::pair<double, double> compute_distance_deviation() const;

    /**
     * @brief How far the offset surface FACES from where it should be: {max, avg} in DEGREES.
     *
     * face_normal_deviation() over the same band-outer faces compute_distance_deviation() measures
     * vertices over, reported alongside it. Distance alone does not pin the offset down: a surface
     * can have every vertex at exactly target_distance while zig-zagging between them.
     */
    std::pair<double, double> compute_normal_deviation() const;

    /**
     * @brief The band's outer surface, recomputed live rather than read from the cached class.
     *
     * A band cell meeting a cell that is neither band nor input complex. Live because the
     * operations that ask this run between one labelling pass and the next, so a face a split
     * just created carries whatever cached class its recycled slot happened to hold.
     *
     * Returns true for a band face on the domain boundary (no opposite tet) -- see
     * compute_distance_deviation() for why that case must not be dropped.
     */
    bool face_is_offset_surface_live(const Tuple& f) const;
    void diag_offset_bands(const char* tag) const;

    /// Whether tet `tid` is part of the offset BAND (as opposed to the input complex it wraps).
    bool cell_is_offset_band(const size_t tid) const { return m_tet_attribute[tid].label == 2; }

    /// Whether tet `tid` is part of the INPUT complex the band wraps.
    bool cell_is_input_complex(const size_t tid) const { return m_tet_attribute[tid].label == 1; }

    /**
     * @brief Warn if conservative growth ran into the bounding box.
     *
     * A band face on the domain boundary means target_distance exceeded the clearance between the
     * input complex and the box, so the offset is clipped there. Those vertices are frozen and no
     * operation can move them, which makes the target distance unreachable by construction --
     * worth saying out loud rather than leaving as a mysterious plateau in max_dist_err.
     */
    void warn_if_offset_reaches_domain_boundary() const;

    /// The vertex compute_distance_deviation() last found the max at, and a dump of everything
    /// that could be stopping it from moving. Diagnostic only.
    mutable size_t m_worst_dist_vid = static_cast<size_t>(-1);
    void log_worst_dist_vertex() const;

    /// {max_dist_err, avg_dist_err, max_norm_dev, avg_norm_dev} per optimization iteration.
    std::vector<std::array<double, 4>> optimization_metrics;
    /// {splits, collapses, swaps} per optimization iteration, mirroring optimization_metrics.
    std::vector<std::array<int, 3>> op_counts;
    /// Whether the optimization met both convergence criteria before the iteration cap.
    bool m_converged = false;

    /// Per-iteration operation counters, reset before each iteration's operation passes.
    std::atomic<int> iter_cnt_split{0};
    std::atomic<int> iter_cnt_collapse{0};
    std::atomic<int> iter_cnt_swap{0};
    std::atomic<int> iter_cnt_collapse_nd_reject{0};
    std::atomic<int> iter_cnt_collapse_offset_removed{0};
    std::atomic<int> iter_cnt_swap_nd_reject{0};

    /**
     * @brief max face_normal_deviation() over the offset-surface faces incident to vertex vid
     */
    double max_offset_surface_normal_deviation_at_vertex(size_t vid) const;

    /**
     * @brief OffsetCollapseBeforeInvariant analogue: pools offset_surface_samples() normals
     * from every offset-surface face incident to remove_vid and returns the spread
     * (max - min, degrees, orientation dependent via normal_angle_180-style acos) of their
     * angles to the collapsing edge's direction. A small spread means the samples around the
     * survivor agree with each other relative to the edge, i.e. the edge runs along a locally
     * flat/consistent part of the offset surface; a large spread means the true target-normal
     * field disagrees with itself nearby -- a sign of a feature edge -- and collapsing there
     * should be rejected. See
     * https://github.com/wildmeshing/topological-offsets/blob/main/components/topological_offsets/wmtk/components/topological_offsets/internal/invariants/OffsetCollapseBeforeInvariant.cpp
     */
    double collapse_normal_deviation(size_t v_from, size_t v_to, size_t remove_vid) const;
    //// collapse


    //// sizing field
    // Bounds and thresholds for the sizing field are user-configurable; see
    // Parameters::min_sizing_scalar / max_sizing_scalar / sizing_mrm_threshold /
    // sizing_gradation. Refinement stops at min_sizing_scalar; coarsening never exceeds
    // max_sizing_scalar (default: the base target itself, since a sizing_scalar > 1 would mean
    // "coarser than what the user asked for", which update_sizing_field() never has a reason
    // to produce).

    /**
     * @brief unsigned mean ratio metric of a triangle: 2*sqrt(3)*area / (sum of squared edge
     * lengths). 1 for equilateral, -> 0 as the triangle degenerates. See
     * https://github.com/wildmeshing/topological-offsets/blob/main/components/topological_offsets/wmtk/components/topological_offsets/internal/utils/mean_ratio_metric.hpp
     */
    static double mean_ratio_metric(const Vector3d& p0, const Vector3d& p1, const Vector3d& p2);

    /**
     * @brief refine or coarsen the sizing field (VertexAttributes::m_sizing_scalar) based on
     * the mean ratio metric of the offset triangulation (offset-class surface faces), following
     * the reference's compute_target_edge_length(): for every vertex incident to at least one
     * offset-surface face, take the worst (minimum) mean_ratio_metric() among those faces, and
     * halve the sizing scalar if it is below m_offset_params.sizing_mrm_threshold, or multiply it
     * by 1.5 (coarsen) if above -- clamped to [m_offset_params.min_sizing_scalar,
     * m_offset_params.max_sizing_scalar]. Vertices not incident to any offset-surface face are left
     * untouched. The vertices actually refined this pass then seed
     * wmtk::utils::gradation_smooth_sizing() (m_offset_params.sizing_gradation), matching
     * SimWildMesh::gradation_smooth_sizing(), so a newly refined patch doesn't sit right next
     * to an unrelated coarse one. Called once per optimize_offset() iteration, after smoothing.
     * @note skeleton: unlike the reference, this doesn't also factor in normal deviation.
     */
    //// sizing field

    //// swap


    /**
     * @brief OffsetSwapInvariant analogue: for the offset-surface diagonal flip (a,b) -> (c,d)
     * across the two current offset faces (a,b,c)/(a,b,d), reject only a regression -- if the
     * *old* diagonal (a,b) was already poorly aligned (spread >=
     * m_offset_params.max_normal_deviation_deg) with the offset target-normal field sampled on both
     * faces, the flip is not blocked on these grounds; if it was well aligned and the *new*
     * diagonal (c,d) would not be, it is rejected. See
     * https://github.com/wildmeshing/topological-offsets/blob/main/components/topological_offsets/wmtk/components/topological_offsets/internal/invariants/OffsetSwapInvariant.cpp
     */
    bool offset_swap_normal_deviation_ok(
        const Tuple& face_abc,
        const Tuple& face_abd,
        size_t a,
        size_t b,
        size_t c,
        size_t d) const;
    //// swap

    /**
     * @brief execute simplistic marching tets. All edges with one vertex labelled 0 and the other
     * 1/2 are split, at the MIDPOINT (Midpoint) or at a fraction of the way toward the input
     * complex (Initial).
     *
     * The distance-field modes that used to exist here -- BinarySearch, LogRootFind and
     * SphereTracing, which root-found each band-boundary edge onto d(x) = delta at insertion time
     * -- are gone, as they are in 2D. They contradict the paper, which places inserted vertices at
     * the midpoint (Sec. 5.2: "we do not perform an interpolation ... we just place the inserted
     * vertices at the midpoint of the split edges") and leaves the distance entirely to the
     * Step-3 optimization; and they did the optimizer's job with a worse tool, having no error
     * feedback and running none of the inversion guards smooth_after_offset_surface() does.
     */
    void marching_tets();


    //// simplicial embedding stuff
    /**
     * @brief check if the input complex (simplices labelled 1) are simplicially embedded w.r.t. the
     * entire mesh
     */
    bool is_simplicially_embedded() const;

    /**
     * @brief check if a tet satisfies simpicial embedding criteria w.r.t. input complex
     * (simplices labelled 1)
     */
    bool tet_is_simp_emb(const Tuple& t) const;

    /**
     * @brief make mesh a simplicial embedding of the input complex (simplices labelled 1)
     */
    void simplicial_embedding();
    //// simplicial embedding stuff

    //// variable offset stuff
    /**
     * @brief check if removing the tet would change the topology of any label
     */
    bool tag_tet_consistent_topology(size_t t_id, int64_t tag) const;

    /**
     * @brief check if adding a tet to the offset region does not change the topology of the
     * offset. Returns true if topology would not be changed
     */
    bool offset_tet_consistent_topology(const size_t t_id) const;

    /**
     * @brief check if a tet is inside the offset (implicitly defined via BVH distance field to
     * input complex) via conservative sphere subdivision estimation
     */
    bool tet_is_in_offset_conservative(const size_t t_id, const double threshold_r) const;

    /**
     * @brief check if a tet is inside the offset (implicitly defined via BVH distance field to
     * input complex) by check if all its vertices are inside the offset region.
     */
    bool tet_is_in_offset_aggressive(const size_t t_id) const;

    /**
     * @brief grow offset region conservatively using conservative checks while ensuring consistent
     * topology
     */
    void grow_offset_conservative();

    /**
     * @brief Grow offset region aggressively. A tet is considered in the offset if all its vertices
     * are in the offset.
     */
    void grow_offset_aggressive();
    //// variable offset stuff

    /**
     * @brief update 'tags' data for tets in the offset region (tets labelled 2) based on
     * the given offset tag values in m_offset_params.offset_tag_value
     */
    void set_offset_tet_tags();

    /**
     * @brief verify that the closed offset region (simplices labelled 1 or 2) form a manifold
     * region. This should be true for any offset. This function is for verification.
     * @note We first collect the tets labelled 1 or 2, then extract the boundary of this region
     * and check if it is manifold.
     */
    bool offset_is_manifold();

    //// output stuff
    void write_input_complex(const std::string& path); // write components labeled to be offset
    void write_vtu(const std::string& path);
    // void write_msh(const std::string& file);
    void write_msh_groups(const std::string& file);
    //// output stuff

private:
    /**
     * @note for all split caches, simplex attributes are inherited from the simplex (of same or
     * higher order) they are 'borne' out of
     */

    struct EdgeSplitCache
    {
        size_t v1_id;
        size_t v2_id;
        // The marching-tets placement and the offset's own data for the new vertex. The
        // shared attributes (position, rounding, bbox, order) are written in `after`.
        Vector3d new_v_pos;
        VertexExtra new_v_extra;

        bool is_edge_on_input = false;
        bool is_edge_on_offset = false;
        bool is_edge_open_boundary = false;

        std::vector<std::pair<FaceAttributes, std::array<size_t, 3>>> changed_faces;

        // cache edge attributes
        EdgeAttributes split_e;
        std::map<size_t, EdgeAttributes> internal_e;
        std::map<simplex::Edge, EdgeAttributes> external_e; // edge is boundary edge (not link)
        std::map<simplex::Edge, EdgeAttributes> link_e; // link edge around splitted edge

        // cache face attributes
        std::map<size_t, FaceSnapshot> split_f; // splitted faces
        std::map<simplex::Edge, FaceSnapshot> internal_f; // new faces created by split
        std::map<std::pair<simplex::Edge, size_t>, FaceSnapshot>
            external_f; // closed star boundary faces of splitted edge

        // cache tet attributes
        std::map<simplex::Edge, TetAttributes> tets;
    };
    wmtk::threading::enumerable_thread_specific<EdgeSplitCache> edge_split_cache;

    struct FaceSplitCache
    {
        size_t v1_id;
        size_t v2_id;
        size_t v3_id;

        // cache edge attributes
        std::map<simplex::Edge, EdgeAttributes> existing_e;

        // cache face attributes
        std::map<simplex::Face, FaceSnapshot> existing_f;
        int splitf_label;

        // cache tet attributes
        std::map<size_t, TetAttributes> tets;
    };
    wmtk::threading::enumerable_thread_specific<FaceSplitCache> face_split_cache;

    struct TetSplitCache
    {
        std::array<size_t, 4> v_ids;

        // cache retained edge attributes
        std::map<simplex::Edge, EdgeAttributes> existing_e;

        // cache retained face attributes
        std::map<simplex::Face, FaceSnapshot> existing_f;

        // cache tet attribute
        TetAttributes tet;
    };
    wmtk::threading::enumerable_thread_specific<TetSplitCache> tet_split_cache;


public:
    // substructure functions

    bool is_order_2_edge(const Tuple& e) const;
    bool is_order_2_edge(const std::array<size_t, 2>& e) const;

    bool vertex_is_on_surface(const size_t vid) const override;

    bool face_is_on_surface(const size_t fid) const override;

    size_t get_order_of_vertex(const size_t vid) const override;
    /**
     * @brief Compute the vertex order for every vertex.
     */
    void init_vertex_order();

private: // helpers
    /**
     * @brief determine if any tag from tag1 is also present in tag2.
     * @note if tag2 is empty (ambient), return true if tag1 is empty, otherwise false (tag2 is
     * ambient, so only 'element' is ambient)
     */
    bool any_tag_present(const CellTag& tag1, const CellTag& tag2)
    {
        if (tag2.empty()) {
            return tag1.empty();
        }
        if (tag1.empty()) { // tag1 is ambient, tag2 is not
            return false;
        }

        for (const int64_t& i : tag1) {
            if (tag2.find(i) != tag2.end()) {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief sort edge simplices in place by decreasing edge length
     */
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

public: // helpers
    /**
     * @brief assign each vertex a partition id (by spatial Morton order) for the parallel
     * ExecutePass policies (kPartition/kColor) used by smooth_all_vertices()/
     * collapse_all_edges(). A no-op if NUM_THREADS == 0.
     */
    void compute_vertex_partition();

    size_t get_partition_id(const Tuple& loc) const
    {
        return m_vertex_attribute[loc.vid(*this)].partition_id;
    }

    /**
     * @brief get tets (as Tuples) that are face-adjacent to the given tet (as Tuple)
     */
    std::vector<Tuple> get_face_adjacent_tets(const Tuple& t) const
    {
        std::vector<Tuple> adj_tets;
        auto tet_1 = t.switch_tetrahedron(*this);
        if (tet_1) {
            adj_tets.push_back(tet_1.value());
        }
        auto tet_2 = t.switch_face(*this).switch_tetrahedron(*this);
        if (tet_2) {
            adj_tets.push_back(tet_2.value());
        }
        auto tet_3 = t.switch_edge(*this).switch_face(*this).switch_tetrahedron(*this);
        if (tet_3) {
            adj_tets.push_back(tet_3.value());
        }
        auto tet_4 =
            t.switch_vertex(*this).switch_edge(*this).switch_face(*this).switch_tetrahedron(*this);
        if (tet_4) {
            adj_tets.push_back(tet_4.value());
        }
        return adj_tets;
    }

    /**
     * @brief get all one-ring vertices through input simplices (labelled 1)
     */
    std::vector<size_t> connected_components_helper(const size_t& v_id)
    {
        auto onering_v_ids = get_one_ring_vids_for_vertex(v_id);
        std::vector<size_t> ret_v_ids;
        for (const size_t& other_v_id : onering_v_ids) {
            size_t e_id = tuple_from_edge({{v_id, other_v_id}}).eid(*this);
            if (m_edge_attribute[e_id].label != 0) { // edge labelled 1 or 2
                ret_v_ids.push_back(other_v_id);
            }
        }
        return ret_v_ids;
    }

    /**
     * @brief reset connected component assignments.
     */
    void reset_connected_components()
    {
        auto verts = get_vertices();
        for (const Tuple& v : verts) {
            size_t v_id = v.vid(*this);
            m_vertex_extra[v_id].component_id = 0;
        }
    }
};


} // namespace wmtk::components::topological_offset
